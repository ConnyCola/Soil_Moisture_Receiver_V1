/*-----------------------------------------------------------------------
/  Special SDcard Functions
/-----------------------------------------------------------------------*/


#include "sdCard.h"
#include "../defines.h"

void printRES2(const void* message,const void* message2,int res)
{	u_int i;
	printf("%s %s",message,message2);
	for(i=0;i<(30-strlen(message)-strlen(message2));i++)
		printf(" ");
	printf("rc= %i\r\n",res);

}

void printRES(const void* message, int res)
{	u_int i;
	//TODO: for schleifen mit prints ersetzten!
	printf("%s ",message);
	for(i=0;i<(30-strlen(message));i++)
		printf(" ");
	printf("rc= %i\r\n",res);

}


void resetSDcard(void){

	SD_NPWR_PORT |= SD_NPWR_PIN;
	delay_ms2(100);
	SD_NPWR_PORT &= ~SD_NPWR_PIN;
	delay_ms2(100);

}


/**
 * Initialize the sdCard (several times)
 * Mount the existing FileSystem
 */
void initSDcard(void)
{
	printf("\r\n\r\n/-------   SD Card Process  -------/\r\n\r\n");

	// ---------------      Init Filesystem     ------------------
	res = disk_initialize();				//Init Disk
	printRES("Disk Init",res);
	res = disk_initialize();				//Init Disk
	printRES("Disk Init",res);

	res = disk_initialize();				//Init Disk
	res = disk_initialize();				//Init Disk
	res = disk_initialize();				//Init Disk
	printRES("Disk Init",res);


	res = pf_mount(&fs);					//Mount FileSystem
	printRES("Mount FS",res);
}



void writeTAGDataSDcard(ENTRY *e){
	u_int i;
	setToSerial(0);
	setStringPointer(e->str);
	i=printf("#%d §%s $%d",e->nr,e->tag,e->data);

	for(;i<30;i++)
		printf(" ");
	printf("\r\n");

	setToSerial(1);
	res = pf_write(e->str,ENTRY_LEN,&bw);
}

/**
 * Finalize sdCard to write back the last 512 Bytes
 * => the last unfinished sector
 */
void finalizeSDcard()
{
	pf_write(0,0,&bw);
}

/**
 * Open a File on the sdCard to read or write to it
 * Set IndexFile to zero (first Entry)
 */
FRESULT openFileSDcard(const void* Filename)
{
	FileIndex = 0;
	res = pf_open(Filename);
	printRES2("Open File ",Filename, res);
	return res;
}



//---------------  Read Data back  -----------------------------------
void readSDcard(ENTRY *e)
{	u_int i;
	u_int j;
	res = pf_read(&e->str, ENTRY_LEN, &s1);
	//printf("%s",e->str);   						//Dump Content to Serial

	//Eintragsnummer auslesen
	i = 1;
	e->nr = 0;
	while(e->str[i] != 0x20 && e->str[i] != 0x00)
	{
		e->nr = (e->nr * 10) + (e->str[i] - 0x30);
		i++;
	}

	//Tag auslesen
	i = i+2;		//Letzte Zahl der Nummer + 2
	j = i;			// offset bis zu beginn des Tags
	while(e->str[i] != 0x20 && e->str[i] != 0x00)
	{
		e->tag[i-j]= e->str[i];
		i++;
	}
	e->tag[i-j] = 0x00;  	//abschluss des Strings

	//Data auslesen
	i = i+2;		// Letztes Zeichen des Tags + 2
	e->data = 0;
	while(e->str[i] != 0x20 && e->str[i] != 0x00)
	{
		e->data = (e->data * 10) + (e->str[i] - 0x30);
		i++;
	}
}


// Find and set read Pointer
void findEntryNrSdCard_READ(DWORD nr, ENTRY *e)
{	u_int i;
	nr = nr-1;
	int ofs = nr % (512 / ENTRY_LEN);
	nr = nr - ofs;
	res = pf_lseek(nr*ENTRY_LEN);
	//printRES("Set Pointer to SectorEnd",res);

	// Pointer von Cluster anfang zu NR bewegen (sehr aufwändig)
	for(i=0;i<ofs;i++)
		pf_read(e->str,ENTRY_LEN,&s1);
	//printRES("Set new Pointer Position",res);
}

//Find and set write Pointer
void findEntryNrSdCard_WRITE(DWORD nr, ENTRY *e)
{	u_int i;
	nr = nr-1;
	int ofs = nr % (512 / ENTRY_LEN);
	nr = nr - ofs;
	res = pf_lseek(nr*ENTRY_LEN);
	//printRES("Set Pointer to SectorEnd",res);

	// Pointer von Cluster anfang zu NR bewegen (sehr aufwändig)
	for(i=0;i<ofs;i++){
		readSDcard(e);
		e++;
	}
	e = e - ofs;

	res = pf_lseek(nr*ENTRY_LEN);
	for(i=0;i<ofs;i++){
		writeTAGDataSDcard(e);
		e++;
	}

	//printRES("Set new Pointer Position",res);


}

// read INI file on SD card to get the configs
void readINI( u_char *sourceIP,  u_char *gatewayIP,  u_char *subnetMask) {
	char str[32];
	strcpy(str,(const u_char*)"                              ");

	int tmp = 0, i = 0, j = 0, k = 0;
	openFileSDcard("INI.txt");

	// 3 Druchlaufe fuer Source, Gateway und SubnetMask
	for (k = 0; k<3; k++){
		pf_read(&str,30,&s1);
		j = 0;
		// 4 Ziffern der IP Adresse
		for(i = 0; i<4; i++){
			while(str[j] < 0x30 || str[j] > 0x39)
				j++;
			tmp = 0;
			while(str[j] >= 0x30 && str[j] <= 0x39)
			{
				tmp = (tmp * 10) + (str[j] - 0x30);
				j++;
			}
			switch (k) {
				case 0:
					sourceIP[i] = tmp;
					break;
				case 1:
					gatewayIP[i] = tmp;
					break;
				case 2:
					subnetMask[i] = tmp;
					break;
				default:
					break;
			}
		}
	}
    finalizeSDcard();


}

void delay_ms2(u_int time_ms) {
	u_int c = 0;
	while (c++ < time_ms) {
		_delay_cycles(16000);
	}
}
