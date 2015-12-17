#include "mrfi.h"
#include "serial/serial.h"
#include "typedefs.h"
#include "defines.h"

#define msg_ON  0x02
mrfiPacket_t packet;
mrfiPacket_t packetreceived;


void main (void)
{	WDTCTL = WDTPW+WDTHOLD;                   // Stop watchdog timer
	P1OUT  = BIT0;				// P1.0 input+pullup
    P1REN |= BIT0;				// P1.0 input+pullup
    P1IE  |= BIT0;				// P1.0 interrupt enable
    P1IES |= BIT0;				// P1.0 Hi/Lo edge
    P1IFG &= ~BIT0;				// P1.0 IFG Flag cleared

	BSP_Init();
	MRFI_Init();				// Init SPI com with CC2500
    MRFI_SetRFPwr(2);			// RF transmitting power (0 to 2)
	mrfiRadioInterfaceWriteReg(CHANNR,180);	// set channel number

	MRFI_WakeUp();				// wake up the radio
	MRFI_RxOn(); 				// turn into Rx mode

	initSerial();				// init Serial interface
	//printf("\r\n\r\n!!!      Receiver      !!!\r\n\r\n");

	__bis_SR_register(LPM0_bits + GIE);    // Enter LPM0, interrupts enabled
}

int8_t RSSI_calculate(uint8_t rawValue)
{	int16_t rssi;
	if(rawValue >= 128)
		rssi = (int16_t)(rawValue - 256)/2 - 74;
	else
		rssi = (rawValue/2) - 74;

	if(rssi < -128)
		rssi = -128;
	return rssi;
}

void MRFI_RxCompleteISR_new()	// in Components/mrfi/radios/family5/mrfi_radio.c
{	// Data back form Sensor
	MRFI_Receive(&packetreceived);

	int offset = 9;
	int i;

	// Send data back to PC
	if (packetreceived.frame[offset] == CMD_RSSI){
		int rssi = abs(RSSI_calculate(packetreceived.rxMetrics[0]));
		//format the RSSI value
		packetreceived.frame[10] = ' ';
		packetreceived.frame[11] = (rssi < 100) ? ' ' : (rssi/100) +'0';
		packetreceived.frame[12] = (rssi < 10) ? ' ' : (rssi/10)%10 +'0';
		packetreceived.frame[13] = rssi%10 +'0';

	}

	for(i=offset; i < packetreceived.frame[0]; i++)
		printf("%c",packetreceived.frame[i]);
	printf("\n");

}

#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{	//Receive Data from PC UART

	char in_key = UCA0RXBUF;
	packet.frame[0] = 11;
	packet.frame[9] = in_key;
	packet.frame[10] = 0x00;
	packet.frame[11] = 0x00;

	while(MRFI_TX_RESULT_SUCCESS!=MRFI_Transmit(&packet, MRFI_TX_TYPE_FORCED));

}

#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{	P1IFG &= ~BIT0;				// P1.0 IFG Flag cleared
	WDTCTL &= ~WDTHOLD;			// Start watchdog timer
	while(1);					// will reset the programm
}


