#ifndef _TYPEDEFS_H_
#define _TYPEDEFS_H_

typedef unsigned char u_char;
typedef unsigned int u_int;
typedef unsigned long u_long;


typedef struct {
	u_char page;
	u_char value;
} Request;


typedef struct {
	u_char Nr;
	u_char temp;
	u_char hum;
	u_char tempDATA[3];
	u_char humDATA[3];
} Article;

typedef struct {
	u_char Nr;
	u_char temp;
	u_char hum;
	u_char tempDATA[31];
	u_char humDATA[31];
} Node;

typedef struct {
	u_int y;	//Year
	u_char m;	//Month
	u_char d;	//Day
	u_char h;	//Hour
	u_char i;	//Minute
	u_char s;	//Second
} TimeStamp;

typedef struct {
	unsigned long int nr;
	char tag[10];  	//Maximale länge der Tags
	unsigned long int data;
	char str[33];	//Maximale Entry länge 32Byte + 0x00
	unsigned long int writeIndex;
} ENTRY;

typedef struct {
	char cmd;
	int val1;
	int val2;
} CMD;

#endif
