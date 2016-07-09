/*
 * Ublox_Reader_V2.c
 *
 * Created: 08/07/2016 10:16:05 p.m.
 *  Author: Italo Amez
 */ 






/***************DEFINICIONES****************/
#define _OPEN_SYS_ITOA_EXT
#define F_CPU 16000000UL
#define MAX_LENGTH 512
#define  POSLLH_MSG  0x02
#define  SOL_MSG     0x06
#define	 TRUE		 0x01
#define	 FALSE		 0x00
#define sbi(x,y) x |= _BV(y)
#define cbi(x,y) x &= ~(_BV(y))


/***************PUNTEROS****************/
#define LONG(X)    *(long*)(&data[X])
#define ULONG(X)   *(unsigned long*)(&data[X])
#define INT(X)     *(int*)(&data[X])
#define UINT(X)    *(unsigned int*)(&data[X])


/*************LIBRARYS & HEADERS******************/
#include <avr/interrupt.h>
#include <math.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include "Uart_Lord_Anthony.h"
#include "uart.h"

/****************DEFINICIONES GLOBALES***************/

unsigned char  state, lstate, code, id, chk1, chk2, ck1, ck2;
unsigned int  length, idx, cnt, len_cmdBuf;

unsigned char data[MAX_LENGTH];

long lastTime = 0;
int checkOk = 0;

/******************************************************/

void enableMsg (unsigned char id, uint8_t enable) {
	//						MSG   NAV   < length >  NAV
	if (enable == 1){
	uint8_t cmdBuf[] = {0x06, 0x01, 0x03, 0x00, 0x01, id, 0x01};
	len_cmdBuf = sizeof(cmdBuf) / sizeof(cmdBuf[0]);
	sendCmd(len_cmdBuf, cmdBuf);
  }
	else{
	uint8_t cmdBuf[] = {0x06, 0x01, 0x03, 0x00, 0x01, id, 0x00};
	len_cmdBuf = sizeof(cmdBuf) / sizeof(cmdBuf[0]); //Obtengo Tamaño del arreglo
	sendCmd(len_cmdBuf, cmdBuf);
  }
}

void setup_UART_Synchro() {
	Serial_begin(9600); // Arduino PC
	Serial2_begin(38400);	// Arduino GPS
	
			if(FALSE) {  //Nunca entra aqui
					while(!(UCSR2A&(1<<RXC2)));
					lstate = state = 0;
						}			
	
	// Modify these to control which messages are sent from module
	enableMsg(POSLLH_MSG, TRUE);    // Enable position messages
	enableMsg(SOL_MSG, TRUE);       // Enable soluton messages
}




// Convert 1e-7 value packed into long into decimal format
void printLatLon (long val) {
	char buffer[14];
	//PString str(buffer, sizeof(buffer));
	ltoa(val,buffer,10);
	//str.print(val, DEC);
	//char len = str.length();
	int len = sizeof(buffer) / sizeof(buffer[0]);
	char ii = 0;
	while (ii < (len - 7)) {
		Serial_write(buffer[ii++]);
	}
	Serial_write(".");
	while (ii < len) {
		Serial_write(buffer[ii++]);
	}
	Serial_write("\r\n");
}


void sendCmd (unsigned char len, uint8_t data[]) {
	TxByte2(0xB5); //Envia GPS
	TxByte2(0x62);
	unsigned char chk1 = 0, chk2 = 0;
	unsigned char i;
			for ( i = 0; i < len; i++) 
			{ //A traves de todo el arreglo "data[]"
			unsigned char cc = data[i];
			//TxByte(cc);
			TxByte2(cc);
			chk1 += cc;		// 0 5 10 15 20 25
			chk2 += chk1;	// 0 5 15 30 50 75
			}//Fin for
	TxByte2(chk1); //Envia GPS
	TxByte2(chk2);
}


int main(void)
{
	unsigned char cc;

	DDRB = (1<<PB7);
	cbi(PORTB,PB7) ;
	_delay_ms(1000);
	sbi(PORTB,PB7);
	_delay_ms(1000);
	cbi(PORTB,PB7) ;
	_delay_ms(1000);
	
	setup_UART_Synchro(); 
	Serial_write(" acaba DE TERMINAR EL SETUP\r\n");
	state = 0;
	
    while(1)
    {
		/*Serial.write(0xB5);
		Serial.write("\r\n"); */
		
	  if (uart_available(2)) {
			unsigned char cc = Serial2_read_1Byte();
		
			TxBCD(state);
			Serial_write("\r\n");
		
			switch (state) {
			  case 0:    // wait for sync 1 (0xB5)
				ck1 = ck2 = 0;
				if (cc == 0xB5)
				  state++;
				break;
			
			  case 1:    // wait for sync 2 (0x62)
				if (cc == 0x62)
				  state++;
				else
				  state = 0;
				break;
			
			  case 2:    // wait for class code
				code = cc;
				ck1 += cc;
				ck2 += ck1;
				state++;
				break;
			
			  case 3:    // wait for Id
				id = cc;
				ck1 += cc;
				ck2 += ck1;
				state++;
				break;
			
			  case 4:    // wait for length byte 1
				length = cc;
				ck1 += cc;
				ck2 += ck1;
				state++;
				break;
			
			  case 5:    // wait for length byte 2
				length |= (unsigned int) cc << 8;
				ck1 += cc;
				ck2 += ck1;
				idx = 0;
				state++;
				if (length > MAX_LENGTH) { state= 0; }
				break;
			
			  case 6:    // wait for <length> payload bytes
				data[idx++] = cc;
				ck1 += cc;
				ck2 += ck1;
				if (idx >= length) {  state++;	}
				break;
			
			  case 7:    // wait for checksum 1
				chk1 = cc;
				state++;
				break;
			
			  case 8:    // wait for checksum 2
				chk2 = cc;
														/*
					Serial_write("ENTRO AL CASE 8 \r\n");
					Serial_write("\r\n  chk1: ");
					TxBCD(chk1);
					Serial_write("\r\n  ck1: ");
					TxBCD(ck1);
					Serial_write("\r\n lenght: ");
					TxBCD(length);
					Serial_write("\r\n ck2: ");
					TxBCD(ck2);
					Serial_write("\r\n chk2: ");
					TxBCD(chk2);
					*/

						if ((ck1 == chk1)  &&  (ck2 == chk2))
							{	checkOk  = 1;	}
						else {checkOk = 0;}
						
				if (checkOk) {
				
														/*
						Serial_write("ingreso al selector \r\n");
						Serial_write("PRECISION: ");
						TxBCD(ULONG(24));
						Serial_write("LONGITUD: ");
						printLatLon(LONG(4));
						Serial_write("\r\n");
						//Serial_print(F(", lat = "));
						Serial_write("LATITUD: ");
						printLatLon(LONG(8));
						Serial_write("\r\n");
						*/
				
				  switch (code) {
					case 0x01:      // NAV-
					  // Add blank line between time groups
					  if (lastTime != ULONG(0)) 
						{
							lastTime = ULONG(0);
						}
					  //Serial.print("NAV-");
							/*	switch (id) {
								
								  case 0x02:  // NAV-POSLLH
									Serial_write("X: ");
									printLatLon(LONG(8));
									Serial_write("Y: ");
									printLatLon(LONG(4));
								   break;
                           
								  case 0x06:  // NAV-SOL
									unsigned char temp2[20];
									ltoa(ULONG(24),temp2,10);
									Serial_write(temp2);
									Serial_write("\r\n");
								   break;
								}*/							
						break;
					  }
				}
				state = 0;
				break;
		}
	  }
    }
}