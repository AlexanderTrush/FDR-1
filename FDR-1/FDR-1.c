



// Target : M16
// Crystal: 7.3728Mhz

#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <ctype.h>

//подключение заголовочных файлов 1Wire библиотеки
#include "OWI\OWIPolled.h"
#include "OWI\OWIHighLevelFunctions.h"
#include "OWI\OWIBitFunctions.h"
#include "OWI\OWIcrc.h"

//номер вывода, к которому подключен датчик
#define BUS1 OWI_PIN_6
#define BUS2 OWI_PIN_7
//#define BUS3 OWI_PIN_5

//команды датчика DS18B20
#define DS18B20_CONVERT_T                0x44
#define DS18B20_WRITE_SCRATCHPAD         0x4e
#define DS18B20_READ_SCRATCHPAD          0xbe

unsigned short CRC16(unsigned char *puchMsg, unsigned short usDataLen);
void mb_read(unsigned short registr, unsigned short data);
void mb_write(unsigned short registr, unsigned short data);
void sensor_foo(unsigned char* TErr, unsigned char* scratchpad, unsigned char BUS, unsigned char* SENSOR, short* T);



void Write_EEPROM (unsigned int uiAddress, unsigned char ucData)
{
  cli();  
  while(EECR & (1<<EEWE));
  EEAR = uiAddress;     
  EEDR = ucData;        
  EECR|=(1<<EEMWE);//SetBit(EECR,2);    
  EECR|=(1<<EEWE);//SetBit(EECR,1);    
  for (unsigned int T_D_int=0;T_D_int<12000;T_D_int++);//  Wait(5000);
  while(EECR & (1<<EEWE)); 
  EEDR=0;             
  EEAR = uiAddress;     
  EECR |= (1<<EERE);      
  //  if(EEDR==ucData) break; //
  // }//
  EEAR=0x00; 
  sei();
}


unsigned char Read_EEPROM (unsigned int uiAddress)
{
  while(EECR & (1<<EEWE)); 
  EEAR = uiAddress;       
  EECR |= (1<<EERE);        
  EEAR=0x00;              
  return EEDR;            
}

//----------------------------------------------------------------
uint32_t timer0=0;
uint32_t speed_timer=0;
unsigned char ANSWER_BUF[255];
unsigned char ANSWER_LEN=0;
unsigned char ANSWER_POINTER=0;

unsigned char scratchpad1[9];
unsigned char scratchpad2[9];

unsigned char BR = 250;
unsigned char RX = 0;

unsigned char ADDRESS, RS2, ADD1, BOUD;
unsigned char SPEED, SPEEDM;
unsigned char UDRbuf[255];

unsigned char TErr1,TErr2;

unsigned char SENSOR1[]={0x01,0x80};
unsigned char SENSOR2[]={0x01,0x80};
unsigned char SENSOR3[]={0x00,0x00};
unsigned char SETPOINT, SETPOINTm;
unsigned char VALVE, VALVEM;
short TREG=-32767;
short TWAT=-32767;
unsigned char Dig[30]; 
unsigned char cDisp=0;
unsigned char DispVal = 0;
unsigned char DispValCatch = 0;
uint32_t DispValCount = 0;
char Disp1, Disp2, Disp3;
int d1, d2, d3;
unsigned char Num[4]; 


void Dig_init()
{
  Dig[0] = 0xC0;    // 0
  Dig[1] = 0xF9;    // 1    
  Dig[2] = 0xA4;    // 2 
  Dig[3] = 0xB0;    // 3
  Dig[4] = 0x99;    // 4
  Dig[5] = 0x92;    // 5
  Dig[6] = 0x82;    // 6
  Dig[7] = 0xF8;    // 7
  Dig[8] = 0x80;    // 8
  Dig[9] = 0x90;    // 9 
  Dig[10] = 0x40;   // 0.  
  Dig[11] = 0x79;   // 1.  
  Dig[12] = 0x24;   // 2.   
  Dig[13] = 0x30;   // 3.    
  Dig[14] = 0x19;   // 4.  
  Dig[15] = 0x12;   // 5.  
  Dig[16] = 0x02;   // 6.  
  Dig[17] = 0x78;   // 7.  
  Dig[18] = 0x00;   // 8.   
  Dig[19] = 0x10;   // 9.
  Dig[20] = 0xBF;   // - 
  Dig[21] = 0xFF;   //  
  Dig[22] = 0x87;   //  t  
  Dig[23] = 0x86;   //  E  
  Dig[24] = 0xCE;   //  Г   
  Dig[25] = 0xAF;   //  r  
  Dig[26] = 0x9C;   //  o 
  Dig[27] = 0x91;   //  У 
  Dig[28] = 0xAB;   //  n   
  Dig[29] = 0x8E;   //  F 
  
  
  
  Num[1] = 0x77;    // ..8     
  Num[2] = 0x7B;    // .8.  
  Num[3] = 0x7D;    // 8..    
}

// Функция выделяет цифры из трехзначного числа Number

void Display (short unsigned int Number)
{
  Disp1 = Dig[21];
  Disp2 = Dig[21];
  Disp3 = Dig[21];
}

//TIMER0 initialize - prescale:8
// WGM: Normal
// desired value: 100uSec
// actual value: 99,826uSec (0,2%)
void timer0_init(void)
{
  TCCR0 = 0x00; //stop
  TCNT0 = 0xA4; //set count
  OCR0  = 0x5C;  //set compare
  TCCR0 = 0x02; //start timer
}

//OCR0 регистр сравнения
//ICR0 регистр захвата
//TIFR флаг переполнения




//-----------------------------------------------------------------------------
void port_init(void)
{
  PORTA = 0xF1;
  DDRA  = 0x0E;
  PORTB = 0xFF;
  DDRB  = 0x00;
  PORTC = 0x00; //m103 output only
  DDRC  = 0xFF;
  PORTD = 0xFC;
  DDRD  = 0xFC;
  PORTD&=~(1<<PIND2); 
  PORTD&=~(1<<PIND3); 
  PORTD&=~(1<<PIND4);
  PORTD&=~(1<<PIND5);
}


// 8 Data, 1 Stop, No Parity
// USART Receiver: On
// USART Transmitter: On
// USART Mode: Asynchronous
void uart0_init(void)
{
	UCSRA = 0x00;
	
	//---------baud rate---------
	//UBRRL = 0xBF; // 2400
	//UBRRL = 0x5F; // 4800
	//UBRRL = 0x2F; // 9600
	//UBRRL = 0x1F; // 14400
	//UBRRL = 0x17; // 19200
	//UBRRL = 0x0B; // 38400
	//UBRRL = 0x03; // 115200
	
	
	//----------parity--------
	UCSRC = (1<<URSEL) | 0x06;  // Disabled (Отключено)
	//UCSRC = (1<<URSEL) | 0x26;    // Even  (Четность)
	//UCSRC = (1<<URSEL) | 0x36;  // Odd  (Нечетность)
	
	
	UBRRH = 0x00;
	//UCSRB = 0x18;
	UCSRB = (1<<RXCIE) | (0<<TXCIE) | (1<<UDRIE) | (1<<RXEN) | (1<<TXEN);
	
}



void WDT_on(void) // Запустить WDT
{
asm ("wdr"); // Сбросить WDT
WDTCR=0x1F;
WDTCR=0x0F;  // Запустить WDT
//  WDTCR|=(1<<4)|(1<<3);
//  WDTCR|=(1<<3);
}



//call this routine to initialize all peripherals
void init_devices(void)
{
  //stop errant interrupts until set up
  //asm ("CLI"); //disable all interrupts
  WDT_on();
  port_init();
  

  
  Dig_init();
  Display(0);
  timer0_init();
  uart0_init();
  OWI_Init(BUS1);
  OWI_Init(BUS2);
 // OWI_Init(BUS3);  
  MCUCR = 0x00;
  GICR  = 0x00;
  TIMSK = 0x41; //timer interrupt sources
  asm ("SEI"); //re-enable interrupts
   
  //all peripherals are now initialized
}



//-----------------TX Interrupt-------------


ISR(USART_UDRE_vect)
{
	if(ANSWER_LEN>0)
	{
		if(ANSWER_POINTER<ANSWER_LEN)
		{
			UCSRA |= (1 << TXC);
			UDR = ANSWER_BUF[ANSWER_POINTER];
			ANSWER_POINTER++;
		}
		else
		{
			while (!(UCSRA & (1 << TXC)));
			PORTD|=(1<<PIND7); //LED TX OFF
			PORTA&=~(1<<7);
			UCSRB |= (1<<RXEN);
			UCSRB &=~ (1<<TXEN);
		}
	}
}

//-----------------RX Interrupt-------------

ISR(USARTRXC_vect)
{
	
	unsigned short crc;
  PORTD&=~(1<<PIND6); //LED RX ON
  cli();
            
  UDRbuf[RX] = UDR;
  
  
   switch(RX)
   {
	   case 0://address
	   if (UDRbuf[RX]==RS2) RX++;
	   else {RX=0; PORTD|=(1<<PIND6);}
	   break;
	   case 1://func
	   if (UDRbuf[RX]==3 || UDRbuf[RX]==6) RX++;
	   else {RX=0; PORTD|=(1<<PIND6);}
	   break;
	   case 2://registrH
	   if (UDRbuf[RX]==0) RX++;
	   else {RX=0; PORTD|=(1<<PIND6);}
	   break;
	   case 3://registrL
	   if (UDRbuf[RX]<6) RX++;
	   else {RX=0; PORTD|=(1<<PIND6);}
	   break;
	   case 4://dataH
	   if (UDRbuf[1]==6 || UDRbuf[RX]==0) RX++;
	   else {RX=0; PORTD|=(1<<PIND6);}
	   break;
	   case 5://dataL
	   if (UDRbuf[1]==6 || (UDRbuf[3]+UDRbuf[RX])<=6) RX++;
	   else {RX=0; PORTD|=(1<<PIND6);}
	   break;
	   case 6://CRCH
	   RX++;
	   break;
	   case 7:
	   crc = CRC16(UDRbuf, 6);
	   if (((crc & 0xFF)==UDRbuf[7]) && ((crc>>8)==UDRbuf[6]))
	   {
		   if (UDRbuf[1]==3) mb_read((UDRbuf[2]<<8)+UDRbuf[3],(UDRbuf[4]<<8)+UDRbuf[5]);
		   else mb_write((UDRbuf[2]<<8)+UDRbuf[3],(UDRbuf[4]<<8)+UDRbuf[5]);
		   {RX=0; PORTD|=(1<<PIND6);}
	   }
	   else {RX=0; PORTD|=(1<<PIND6);}
	   break;
	   default: {RX=0; PORTD|=(1<<PIND6);}
   } 
  sei();
  
}
//----------------------------------

void mb_read(unsigned short registr, unsigned short data)
{
	unsigned short crc;
	unsigned char DATA_BUF[12];
	DATA_BUF[0]=0;
	DATA_BUF[1]=SPEED;
	DATA_BUF[2]=0;
	DATA_BUF[3]=VALVE;
	DATA_BUF[4]=0;
	DATA_BUF[5]=SETPOINT;
	DATA_BUF[6]=SENSOR1[1];
	DATA_BUF[7]=SENSOR1[0];
	DATA_BUF[8]=SENSOR2[1];
	DATA_BUF[9]=SENSOR2[0];
	DATA_BUF[10]=SENSOR3[1];
	DATA_BUF[11]=SENSOR3[0];
	ANSWER_BUF[0]=RS2;
	ANSWER_BUF[1]=3;
	ANSWER_BUF[2]=data*2;
	for (char an=0; an<data*2; an++) ANSWER_BUF[an+3] = DATA_BUF[an+registr*2];
	crc = CRC16(ANSWER_BUF, 3+data*2);
	ANSWER_BUF[3+data*2] = crc>>8;
	ANSWER_BUF[4+data*2] = crc;
	ANSWER_LEN = 5+data*2;
	ANSWER_POINTER = 0;
	
	// Включение передачи
	PORTA|=(1<<7);  //Enable Tx, Disable Rx
	PORTD&=~(1<<PIND7); //LED TX ON
	UCSRB &=~ (1<<RXEN);
	UCSRB |= (1<<TXEN);
	
}

void mb_write(unsigned short registr, unsigned short data)
{
	
	for(int k = 0; k < 8; k++) ANSWER_BUF[k]=UDRbuf[k];
	ANSWER_LEN = 8;
	ANSWER_POINTER = 0;
	// Включение передачи
	PORTA|=(1<<7);  //Enable Tx, Disable Rx
	PORTD&=~(1<<PIND7); //LED TX ON
	UCSRB &=~ (1<<RXEN);
	UCSRB |= (1<<TXEN);
	
	
	
	if (registr==0)//speed
	{
		if (data<4)  SPEEDM = data;
		else SPEEDM = 3;
	}
	else if (registr == 1)//valve
	{
		if (data<2)  VALVEM = data;
		else VALVEM = 1;
	}
	else if (registr == 2)//stp
	{
		SETPOINTm = data;
	}
	
}



ISR(TIMER0_OVF_vect)
{
	
  //TIMER0 has overflowed
  TCNT0 = 0xA4; //reload counter
  TCCR0 = 0x02; //start timer
  timer0++;
  
  if (speed_timer>0)speed_timer++;
  
  
  if (timer0==70000) sensor_foo(&TErr1, scratchpad1, BUS1, SENSOR1, &TREG);
  if (timer0==140000) sensor_foo(&TErr2, scratchpad2, BUS2, SENSOR2, &TWAT);
  if (timer0>140000)timer0=0;//14sec 
  
  if ((PINA&1)==0)
  {
	  DispValCatch++;
	  //PORTD&=~(1<<PIND6);
  }
  else
  {
	  //PORTD|=(1<<PIND6);
	  if (DispValCatch>3)
	  {
		  DispVal++;
		  if (DispVal>3) DispVal=0;
		  DispValCount=1;
	  }
	  DispValCatch=0;
	  if (DispValCount>0) DispValCount++;
	  if (DispValCount>100000) {DispValCount=0; DispVal=0;}
  }
  
  PORTA|=(0x0E);  //Очистка PA1, PA2, PA3 
  switch (cDisp)
  {
	  case 0:
	  PORTA&=~(1<<PINA1);
	  PORTC = Disp1;
	  cDisp = 1;
	  break;
	  case 1:
	  PORTA&=~(1<<PINA2);
	  PORTC = Disp2;
	  cDisp = 2;
	  break;
	  case 2:
	  PORTA&=~(1<<PINA3);
	  PORTC = Disp3;
	  cDisp = 0;
  }
  //-------------------------
}


//-------------------CRC16-------------------------------------------

__flash   unsigned const char auchCRCHi[]=
{
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
  0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 
  0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 
  0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 
  0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 
  0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 
  0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 
  0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 
  0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
  0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 
  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
  0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 
  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 
  0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 
  0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
  0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
}; 

__flash  unsigned const char auchCRCLo[]=
{
  0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 
  0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 
  0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 
  0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 
  0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4, 
  0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 
  0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 
  0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 
  0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 
  0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 
  0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 
  0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 
  0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 
  0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 
  0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 
  0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 
  0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 
  0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5, 
  0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 
  0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 
  0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 
  0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 
  0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 
  0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C, 
  0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 
  0x43, 0x83, 0x41, 0x81, 0x80, 0x40
}; 

unsigned short CRC16(unsigned char *puchMsg, unsigned short usDataLen)
{
  unsigned char uchCRCHi = 0xFF ;	// high CRC byte initialized
  unsigned char uchCRCLo = 0xFF ;	// low CRC byte initialized
  unsigned char uIndex ;	        // will index into CRC lookup
  // table
  
  while (usDataLen--)		// pass through message buffer
  {
    uIndex = uchCRCHi ^ *puchMsg++ ;	// calculate the CRC
    uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex] ;
    uchCRCLo = auchCRCLo[uIndex] ;
  }
  
  return (uchCRCHi << 8 | uchCRCLo) ;
}
//------------------------------------------------------------------------

 //----------CRC-SENSOR---------------
 unsigned char CRC_Dall (unsigned char * scratchpad)
 {
	 unsigned char j, i, Data, tmpd, CRCd = 0;
	 for (j = 0; j < 8; j++)
	 {
		 Data = scratchpad[j];
		 for (i = 0; i < 8; i++)
		 {
			 tmpd = 1 & (Data ^ CRCd);
			 CRCd >>= 1;
			 Data >>= 1;
			 if (0 != tmpd)
			 {
				 CRCd ^= 0x8c;
			 }
		 }
	 }
	 return CRCd;
 }
 
 short t_calculate(unsigned char * scratchpad)
 {
	 short temperature;
	 if (scratchpad[1] & 0x80)//отрицательное
	 {
		 temperature = (~(scratchpad[1]*256+scratchpad[0]))+1;
		 temperature = -((temperature>>4)*10 + (temperature & 15)*10/16);
	 }
	 else//положительное
	 {
		 temperature = scratchpad[1]*256+scratchpad[0];
		 temperature = (temperature>>4)*10 + (temperature & 15)*10/16;
	 }
	 return temperature;
 }
 
 
 void sensor_foo(unsigned char* TErr, unsigned char* scratchpad, unsigned char BUS, unsigned char* SENSOR, short* T)
 {
	 int i;
	  TErr[0]++;
	  cli();
	  PORTA|=(0x0E);  //Очистка PA1, PA2, PA3 
	  OWI_DetectPresence(BUS);
	  sei();
	  cli();
	  PORTA|=(0x0E);  //Очистка PA1, PA2, PA3 
	  OWI_SkipRom(BUS);
	  sei();
	  cli();
	  PORTA|=(0x0E);  //Очистка PA1, PA2, PA3 
	  OWI_SendByte(DS18B20_CONVERT_T ,BUS);
	  sei();
	  cli();
	  PORTA|=(0x0E);  //Очистка PA1, PA2, PA3 
	  OWI_DetectPresence(BUS);
	  sei();
	  cli();
	  PORTA|=(0x0E);  //Очистка PA1, PA2, PA3 
	  OWI_SkipRom(BUS);
	  sei();
	  cli();
	  PORTA|=(0x0E);  //Очистка PA1, PA2, PA3 
	  OWI_SendByte(DS18B20_READ_SCRATCHPAD, BUS);
	  for (i = 0; i<=8; i++)
	  {
		  scratchpad[i] = OWI_ReceiveByte(BUS);
	  }
	  sei();
	  
	  //--------------------------------
	  if ((CRC_Dall(scratchpad) == scratchpad[8]))
	  {
		  TErr[0] = 0;
		  short temp_t = t_calculate(scratchpad);
		  *T= temp_t;
		  SENSOR[0] = temp_t;
		  SENSOR[1] = temp_t>>8;
	  }
	  
	  if (TErr[0] > 6)
	  {
		  SENSOR[0]=1;
		  SENSOR[1]=128;
		  *T=-32767;
	  }
 }


int main( void )
{  

  	
  init_devices();
  
  PORTD&=~(0x3f);
  PORTD|=(0xc0);
  
  UCSRB = UCSRB | (1<<TXEN);
  UCSRB = UCSRB | (1<<RXEN);
  PORTA&=~(1<<7);
  
  //eeprom = 0;
  SETPOINT = Read_EEPROM (300);
  SETPOINTm = SETPOINT;
  
  TErr1 = 50;
  TErr2 = 50;

 
  
  while(1)
  {         
    ADDRESS=~PINB;
    ADDRESS&=~(0xE0);
    ADDRESS=(ADDRESS<<1);
    ADD1=~PINA;
    ADD1&=~(0xEF);
    ADD1=(ADD1>>4);
    RS2=ADDRESS+ADD1;
    
    asm ("wdr"); // Сбросить WDT
    
    //------BOUD--RATE-------------
    BOUD=~PINA;
    BOUD&=~(0x9F);
    BOUD=(BOUD>>5);
  
	if (BOUD != BR)
	{
		if (BOUD == 1)
		{
			UBRRL = 0x2F; // 9600
		}
		else if (BOUD == 2)
		{
			UBRRL = 0x17; // 19200
		}
		else if (BOUD == 0)
		{
			UBRRL = 0x0B; // 38400
		}
		else if (BOUD == 3)
		{
			UBRRL = 0x03; // 115200
		}
		BR = BOUD;
	}
    

   
    //------------------------------------
 
      
      SENSOR3[0] = !PINB5;
      
    //--------------------------------------
	
	
	
	
	switch(DispVal)
	{
		case 0:
			if (TREG==-32767) {d1 = 23; d2=25; d3=25;}
			else if (TREG<0) 
			{
				d1=20;
				d2=TREG/100;
				d3=(TREG/10)%10;
				if (d2==0) d2=21;
			}
			else
			{
				d1=TREG/100;
				d2=(TREG/10)%10;
				d3=TREG%10;
				d2 += 10;	
				if (d1 == 0) d1=21;
			}
		break;
		case 1:
			if (TWAT==-32767) {d1 = 23; d2=25; d3=25;}
			else if (TWAT<0)
			{
				d1=20;
				d2=TWAT/100;
				d3=(TWAT/10)%10;
				if (d2==0) d2=21;
			}
			else
			{
				d1=TWAT/100;
				d2=(TWAT/10)%10;
				d3=TWAT%10;
				d2 += 10;
				if (d1 == 0) d1=21;
			}
		break;
		case 2:
			d1 = 0;
			if (VALVE==0) 
				{
					d2 = 29;
					d3 = 29;
				}
			else
				{
					d2 = 28;
					d3 = 21;
				}
		break;
		case 3:
			d1=SETPOINT/10;
			d2=SETPOINT%10;
			if (d1 == 0) d1=21;
			d3=26;
		break;		
	}
	cli();
			Disp1=Dig[d1];
			Disp2=Dig[d2];
			Disp3=Dig[d3];
	sei();  
	  
   
    //------------------------------------------------------
    
	
	if (SPEED != SPEEDM)
	{
		if (speed_timer==0){
			PORTD&=~(1<<PIND2);
			PORTD&=~(1<<PIND3);
			PORTD&=~(1<<PIND4);
			speed_timer = 1;
		}
		else if (speed_timer>=5000)
		{
			speed_timer=0;
			SPEED = SPEEDM;
			if (SPEED==1)
			{
				PORTD|=(1<<PIND2); 
			}
			else if (SPEED==2)
			{
				PORTD|=(1<<PIND3); 
			}
			else if (SPEED>=3)
			{
				PORTD|=(1<<PIND4);
			}
		}
	}
	
	if (SETPOINT != SETPOINTm)
	{
		SETPOINT = SETPOINTm;
		Write_EEPROM (300,SETPOINT);
	}
	else if (SETPOINT == 255)
	{
		SETPOINT = 0;
		SETPOINTm = SETPOINT;
		Write_EEPROM (300,SETPOINT);
	}
    
    if (SETPOINT == 0) VALVE = VALVEM;
    else
	{
		if (SPEED>0)
		{
			if (SETPOINT>(TREG/10)) VALVE = 1;
			else VALVE = 0;
		}
		else VALVE = 0;
	}
	
	if (VALVE == 0) PORTD&=~(1<<PIND5);
	else PORTD|=(1<<PIND5);
  }
  return 0; 
}   




