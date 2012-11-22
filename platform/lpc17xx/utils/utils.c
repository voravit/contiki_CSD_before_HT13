/*****************************************************************************
 *camilo 2011-07 KTH
 *
******************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "./utils.h"


//char *findParameter1(char* cmd, char *parameter );



char ReturnChar(){

	return 'a';
}

char* floatToString(float *flo, int *integer, int *decimal){
	char tmpstr1[1024];
	//int integer, decimal;
	integer = (int)*flo;
	*flo -= *integer;
	int decimals=2;
	for(; decimals > 0; decimals--) {
		*flo *= 10;
	}
	decimal = (int)*flo;
	sprintf(tmpstr1, "%3d.%02d ", integer, decimal);
	return tmpstr1;
}
char* join(char *a, char *b){
	strcat(a,b);
	return *a;
}

/**
 * Breaks a float number into integer part and decimal part, which are put
 * to corresponding variables. Dlen is the length of the decimal part. Base
 * is the number base.
 */

/*void break_float1(float f, int integer1, int decimal1, int dlen, int base) {

//	*decimal = 0;
}*/
//time is in miliseconds (aprox)
void delay(unsigned long time)
{
	time*=450;
	volatile unsigned long i;
	for (i = 0; i < time; i++);
}

//camilo
/*
char* strtrim1(char *str){
	//backwards
	int i;
	int j;
	int len=(int)strlen(str);
	int write=0;
	char newStr1[len];
		i=len;
		j=len;

		for (i;i>=0;i--){
			if(str[i]!=' '||write==1){
				newStr1[j]=str[i];
				j--;
				write=1;
			}
		}
	i=0; j=0;
	len=(int)strlen(newStr1);
	write=0;
	char newStr[strlen(newStr1)];
	for (i;i<len;i++){
		if(str[i]!=' '||write==1){
			newStr[j]=newStr1[i];
			j++;
			write=1;
		}
	}

	return *newStr;
}*/
char* strtrim(char *str, char *newStr){
	//backwards
	int i,write;
	int j;

	i=0; j=0;
	int len=(int)strlen(str);
	write=0;
	//char *newStr[len];
	for (i;i<len;i++){
		if(str[i]!=' '||write==1){
			if(str[i+1]!=' '||str[i]!=' '){
				newStr[j]=str[i];
			}
			j++;
			write=1;
		}
	}
	return newStr;
}
