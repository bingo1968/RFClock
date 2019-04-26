/*------------------------------------------------------------------*/
/* --- STC MCU Limited ---------------------------------------------*/
/* --- STC89-90xx Series MCU UART (8-bit/9-bit)Demo ----------------*/
/* --- Mobile: (86)13922805190 -------------------------------------*/
/* --- Fax: 86-0513-55012956,55012947,55012969 ---------------------*/
/* --- Tel: 86-0513-55012928,55012929,55012966----------------------*/
/* --- Web: www.STCMCU.com -----------------------------------------*/
/* --- Web: www.GXWMCU.com -----------------------------------------*/
/* If you want to use the program or the program referenced in the  */
/* article, please specify in which data and procedures from STC    */
/*------------------------------------------------------------------*/

#include "reg51.h"
#include "intrins.h"

typedef unsigned char BYTE;
typedef unsigned int WORD;

#define FOSC 11059200L      //System frequency
#define BAUD 9600           //UART baudrate

/*Define UART parity mode*/
#define NONE_PARITY     0   //None parity
#define ODD_PARITY      1   //Odd parity
#define EVEN_PARITY     2   //Even parity
#define MARK_PARITY     3   //Mark parity
#define SPACE_PARITY    4   //Space parity

#define PARITYBIT NONE_PARITY   //Testing even parity

#define T1MS (65536-FOSC/12/1000)   //1ms timer calculation method in 12T mode


/*define command types*/
#define CMDGETTIME	1
#define CMDSETTIME	2
#define CMDGETNVRAM	3
#define CMDSETNVRAM 4
#define CMDDUMPBUF	5
#define	CMDBEEP			6
#define CMDPRNTIME  7
#define CMDSENDTIME 8

#define BEEPCNT	300

sbit bit9 = P2^2;           //P2.2 show UART data bit9
bit busy;

sbit pinCS = P1^7;
sbit pinAS = P1^6;
sbit pinRW = P1^5;
sbit pinDS = P1^4;

//sbit DIO = P1^3;				//595 io
//sbit RCLK  = P1^2;			//595 rclk
//sbit SCLK = P1^1;				// 595 sclk
//---- lcd1602 pin define
sbit rs = P1 ^ 1;
sbit rw = P1 ^ 2;
sbit ep = P1 ^ 3;

sbit DIO = P3^3;				//595 io
sbit RCLK  = P3^4;			//595 rclk
sbit SCLK = P3^2;				// 595 sclk
sbit pinLED = P1^0;
//sbit pinDebug=P1^1;

BYTE data tasklst[9];
BYTE data bufSrecv[30];// = "CMD 18-12-31 7 01:59:43"
BYTE data buftmp[17];
BYTE data buftmp1[17];
BYTE rcvidx = 0;

/* define variables */
WORD count;                         //1000 times counter
WORD beepcounter;

BYTE code LED_0F[] = 
{// 0	   1	  2	   3	  4	   5	  6	   7	  8	   9	  A	   b	  C    d	  E    F    :
	0xC0,0xF9,0xA4,0xB0,0x99,0x92,0x82,0xF8,0x80,0x90,0x8C,0xBF,0xC6,0xA1,0x86,0xFF,0x7f
};

BYTE data LED[8];		// LED???

void SendData(BYTE dat);
void SendString(char *s);
void appendtask(BYTE tsk);
void led595_Display(void);
void SendUartBytes(BYTE *s, BYTE len);
void lcd_wcmd(BYTE cmd);
void lcd_pos(BYTE pos);
void lcd_wdat(BYTE dat);
void lcd_init();


void Delay100us()		//@11.0592MHz
{
	unsigned char i, j;

	_nop_();
	_nop_();
	i = 2;
	j = 15;
	do
	{
		while (--j);
	} while (--i);
}

void clrsbuf()
{
	BYTE i;
	rcvidx=0;
	for (i=0; i<30; i++) bufSrecv[i]=0x00;
}

void clrbuftmp()
{
	BYTE i;
	for (i=0; i<17; i++) buftmp[i]=0;
}

void clrbuftmp1()
{
	BYTE i;
	for (i=0; i<17; i++) buftmp1[i]=0;
}
	
void ds12887_write(BYTE add, BYTE dat)
{
	pinCS=0;
	pinAS=1;
	pinDS=1;
	pinRW=1;
	P0=add;
	pinAS=0;
	pinRW=0;
	P0=dat;
	pinRW=1;
	pinAS=1;
	pinCS=1;
}

BYTE ds12887_read(BYTE add)
{
	BYTE tmp_dat;
	pinAS=1;
	pinDS=1;
	pinRW=1;
	pinCS=0;
	P0=add;
	pinAS=0;
	pinDS=0;
	P0=0xff;
	tmp_dat=P0;
	pinDS=1;
	pinAS=1;
	pinCS=1;
	return tmp_dat;
}
	
void ds12887_init()
{
	ds12887_write(0x0a,0x20);
	ds12887_write(0x0b,0x02);
}

void ds12887_gettime()
{
	clrbuftmp();
	buftmp[1]=ds12887_read(0x00);
	buftmp[2]=ds12887_read(0x02);
	buftmp[3]=ds12887_read(0x04);
	buftmp[4]=ds12887_read(0x06);
	buftmp[5]=ds12887_read(0x07);
	buftmp[6]=ds12887_read(0x08);
	buftmp[7]=ds12887_read(0x09);
	buftmp[0]=1;
}

void ds12887_settime()
{
	BYTE yy,mm,dd,ww,hh,mn,ss; //"STM 19-02-18 2 14:31:00"	
	yy=bufSrecv[4]-0x30;
	yy<<=4;
	yy=yy+bufSrecv[5]-0x30;
	mm=bufSrecv[7]-0x30;
	mm<<=4;
	mm=mm+bufSrecv[8]-0x30;
	dd=bufSrecv[10]-0x30;
	dd<<=4;
	dd=dd+bufSrecv[11]-0x30;
	ww=bufSrecv[13]-0x30;
	hh=bufSrecv[15]-0x30;
	hh<<=4;
	hh=hh+bufSrecv[16]-0x30;
	mn=bufSrecv[18]-0x30;
	mn<<=4;
	mn=mn+bufSrecv[19]-0x30;
	ss=bufSrecv[21]-0x30;
	ss<<=4;
	ss=ss+bufSrecv[22]-0x30;
	ds12887_write(0x00,ss);
	ds12887_write(0x02,mn);
	ds12887_write(0x04,hh);
	ds12887_write(0x06,ww);
	ds12887_write(0x07,dd);
	ds12887_write(0x08,mm);
	ds12887_write(0x09,yy);
	bufSrecv[1]=ss;
	bufSrecv[2]=mn;
	bufSrecv[3]=hh;
	bufSrecv[4]=ww;
	bufSrecv[5]=dd;
	bufSrecv[6]=mm;
	bufSrecv[7]=yy;
	bufSrecv[8]=0;
	//SendUartBytes(bufSrecv, 8);

}

void ds12887_setnvram()
{		//STR
	BYTE i,tmpadd=bufSrecv[3];
	for (i=4;i<rcvidx-1;i++){
		ds12887_write(tmpadd,bufSrecv[i]);
		tmpadd++;
		SendData(tmpadd);
		SendData(bufSrecv[i]);
	}
}

void ds12887_getnvram()
{		//GTR  read nonvolatile ram
	BYTE i, tmpadd=bufSrecv[3];
	clrbuftmp();
	for (i=1;i<16;i++){
		buftmp[i]=ds12887_read(tmpadd);
		tmpadd++;
		SendData(buftmp[i]);  //for debug
	}
	buftmp[0]=9;
}
 	
void parseScmd()
{
	if ((bufSrecv[0]==0x53)&(bufSrecv[1]==0x54)&(bufSrecv[2]==0x4D)&&(rcvidx>23)) appendtask(CMDSETTIME);   //STM
	if ((bufSrecv[0]==0x53)&(bufSrecv[1]==0x54)&(bufSrecv[2]==0x52)&&(rcvidx>5)) appendtask(CMDSETNVRAM);	//STR
	if ((bufSrecv[0]==0x47)&(bufSrecv[1]==0x54)&(bufSrecv[2]==0x52)&&(rcvidx>4)) appendtask(CMDGETNVRAM);	//GTR
	if ((bufSrecv[0]==0x47)&(bufSrecv[1]==0x54)&(bufSrecv[2]==0x4D)) appendtask(CMDGETTIME);		//GTM
	if ((bufSrecv[0]==0x53)&(bufSrecv[1]==0x54)&(bufSrecv[2]==0x42)) appendtask(CMDBEEP);		//STB
	if ((bufSrecv[0]==0x7B)) buftmp1[16]=0xff;	//just for debug
}

BYTE bcdtobyte(BYTE bcd, BYTE big)
{
	BYTE btmp;
	if (big){
		btmp=bcd>>4;
		btmp=btmp+0x30;
	} else {
		btmp=bcd<<4;
		btmp>>=4;
		btmp=btmp+0x30;
	}
	return btmp;
}

void sendtime()
{
	ds12887_gettime();
	SendData(bcdtobyte(buftmp[7],1));
	SendData(bcdtobyte(buftmp[7],0));
	SendData(bcdtobyte(buftmp[6],1));
	SendData(bcdtobyte(buftmp[6],0));
	SendData(bcdtobyte(buftmp[5],1));
	SendData(bcdtobyte(buftmp[5],0));
	SendData(bcdtobyte(buftmp[3],1));
	SendData(bcdtobyte(buftmp[3],0));
	SendData(bcdtobyte(buftmp[2],1));
	SendData(bcdtobyte(buftmp[2],0));
	SendData(bcdtobyte(buftmp[1],1));
	SendData(bcdtobyte(buftmp[1],0));	
	SendData(bcdtobyte(buftmp[4],0));
	SendData(0x0A);		
	clrbuftmp();
}

void disptime()
{
	BYTE i;
	ds12887_gettime();
	LED[0]=bcdtobyte(buftmp[2],0)-0x30;
	LED[1]=bcdtobyte(buftmp[2],1)-0x30;
	LED[2]=bcdtobyte(buftmp[3],0)-0x30;
	LED[3]=bcdtobyte(buftmp[3],1)-0x30;
	lcd_pos(0);
	lcd_wdat(bcdtobyte(buftmp[3],1));
	lcd_wdat(bcdtobyte(buftmp[3],0));
	lcd_wdat(0x3A);
	lcd_wdat(bcdtobyte(buftmp[2],1));
	lcd_wdat(bcdtobyte(buftmp[2],0));
	lcd_wdat(0x3A);
	lcd_wdat(bcdtobyte(buftmp[1],1));
	lcd_wdat(bcdtobyte(buftmp[1],0));
	lcd_wdat(0x20);
	
	clrbuftmp();
	for (i=0; i<3; i++){
		SendData(0x7f);
		SendData(0x4);
		SendData(LED[3]+0x30);
		SendData(LED[2]+0x30);
		SendData(LED[1]+0x30);
		SendData(LED[0]+0x30);
		Delay100us();
		Delay100us();
		Delay100us();
		Delay100us();
		Delay100us();
		Delay100us();
		Delay100us();
	}
}

/* TASK processing functions*/
void appendtask(BYTE tsk)
{
	tasklst[0]=1;
	tasklst[1]=tsk;
}

void removetask(BYTE idx)
{
	tasklst[idx]=0;
	tasklst[0]=0;
}

void exectasks()
{
	BYTE i;
	while (tasklst[0]>0)
	{
		if (tasklst[1]==CMDGETTIME) {
			sendtime();
			removetask(1);
		}
		if (tasklst[1]==CMDSETTIME) {
			ds12887_settime();
			SendUartBytes(bufSrecv, 8);
			removetask(1);
		}
		if (tasklst[1]==CMDPRNTIME) {
			disptime();
			removetask(1);
		}
		if (tasklst[1]==CMDSETNVRAM){
			ds12887_setnvram();
			removetask(1);
		}
		if (tasklst[1]==CMDGETNVRAM){
			ds12887_getnvram();
			removetask(1);
		}
		if (tasklst[1]==CMDBEEP){
			beepcounter=BEEPCNT;
			removetask(1);
		}
	}
}


/*Main */
void main()
{
	BYTE i;
#if (PARITYBIT == NONE_PARITY)
    SCON = 0x50;            //8-bit variable UART
#elif (PARITYBIT == ODD_PARITY) || (PARITYBIT == EVEN_PARITY) || (PARITYBIT == MARK_PARITY)
    SCON = 0xda;            //9-bit variable UART, parity bit initial to 1
#elif (PARITYBIT == SPACE_PARITY)
    SCON = 0xd2;            //9-bit variable UART, parity bit initial to 0
#endif

    TMOD = 0x21;            //Set Timer1 as 8-bit auto reload mode & set timer0 16bit
    TL0 = T1MS;                     //initial timer0 low byte
    TH0 = T1MS >> 8;                //initial timer0 high byte
    TR0 = 1;                        //timer0 start running
    ET0 = 1;                        //enable timer0 interrupt

		TH1 = TL1 = -(FOSC/12/32/BAUD); //Set auto-reload vaule
    TR1 = 1;                //Timer1 start run
    ES = 1;                 //Enable UART interrupt
    
		EA = 1;                 //Open master interrupt switch
	
	
		for (i=0; i<9; i++) tasklst[i]=0;			//clear task list
		ds12887_init();		
		lcd_init();
		lcd_wcmd(0x38); //8bit,1/16,5x8

    while(1){	
			exectasks();
			//led595_Display();
		}
}

/*----------------------------
UART interrupt service routine
----------------------------*/
void Uart_Isr() interrupt 4 using 1
{
    if (RI)
    {
        RI = 0;             //Clear receive interrupt flag
				if (SBUF==0x7F) {
					clrsbuf();
				} else {
					bufSrecv[rcvidx]=SBUF;
					rcvidx++;
				}
				if ((rcvidx>0)&(bufSrecv[rcvidx-1]==0x0D)){
					parseScmd();
				}
    }
    if (TI)
    {
        TI = 0;             //Clear transmit interrupt flag
        busy = 0;           //Clear transmit busy flag
    }
}

/*----------------------------
Send a byte data to UART
Input: dat (data to be sent)
Output:None
----------------------------*/
void SendData(BYTE dat)
{
    while (busy);           //Wait for the completion of the previous data is sent
    ACC = dat;              //Calculate the even parity bit P (PSW.0)
    if (P)                  //Set the parity bit according to P
    {
#if (PARITYBIT == ODD_PARITY)
        TB8 = 0;            //Set parity bit to 0
#elif (PARITYBIT == EVEN_PARITY)
        TB8 = 1;            //Set parity bit to 1
#endif
    }
    else
    {
#if (PARITYBIT == ODD_PARITY)
        TB8 = 1;            //Set parity bit to 1
#elif (PARITYBIT == EVEN_PARITY)
        TB8 = 0;            //Set parity bit to 0
#endif
    }
    busy = 1;
    SBUF = ACC;             //Send data to UART buffer
}

/*----------------------------
Send a string to UART
Input: s (address of string)
Output:None
----------------------------*/
void SendString(char *s)
{
    while (*s)              //Check the end of the string
    {
        SendData(*s++);     //Send current char and increment string ptr
    }
}

void SendUartBytes(BYTE *s, BYTE len)
{
	BYTE i;
	for (i=0; i<len; i++) SendData(s[i]);
}


/* Timer0 interrupt routine */
void tm0_isr() interrupt 1 using 2
{
    TL0 = T1MS;                     //reload timer0 low byte
    TH0 = T1MS >> 8;                //reload timer0 high byte
    if (count-- == 0)               //1ms * 1000 -> 1s
    {
        count = 1000;               //reset counter
        pinLED = !pinLED;      //work LED flash
				appendtask(CMDPRNTIME);
    }
}


/*74595 write routine*/
void led595_OUT(BYTE X)
{
	BYTE i;
	for(i=8;i>=1;i--)
	{
		if (X&0x80) DIO=1; else DIO=0;
		X<<=1;
		SCLK = 0;
		SCLK = 1;
	}
}

void led595_Display(void)
{
	BYTE code *led_table;          // led table
	BYTE i;
	led_table = LED_0F + LED[0];
	i = *led_table;

	led595_OUT(i);			
	led595_OUT(0x01);		

	RCLK = 0;
	RCLK = 1;
	led_table = LED_0F + LED[1];
	i = *led_table;

	led595_OUT(i);		
	led595_OUT(0x02);		

	RCLK = 0;
	RCLK = 1;
	led_table = LED_0F + LED[2];
	i = *led_table;
	if (pinLED){			//time division dots
		i<<=1;
		i>>=1;
	}
	led595_OUT(i);			
	led595_OUT(0x04);	

	RCLK = 0;
	RCLK = 1;
	led_table = LED_0F + LED[3];
	i = *led_table;

	led595_OUT(i);			
	led595_OUT(0x08);		

	RCLK = 0;
	RCLK = 1;
}



////LCD1602 procs
//---------LCD module
void lcd_wcmd(BYTE cmd)
{ // ???????LCD
	delay100us();
	rs = 0;
	rw = 0;
	ep = 0;
	_nop_();
	_nop_();
	P2 = cmd;
	_nop_();
	_nop_();
	_nop_();
	_nop_();
	ep = 1;
	_nop_();
	_nop_();
	_nop_();
	_nop_();
	ep = 0;
}

void lcd_pos(BYTE pos)
{
	lcd_wcmd(pos | 0x80);
}

void lcd_wdat(BYTE dat)
{ 
	delay100us();
	rs = 1;
	rw = 0;
	ep = 0;
	_nop_();
	_nop_();
	P2 = dat;
	_nop_();
	_nop_();
	_nop_();
	_nop_();
	ep = 1;
	_nop_();
	_nop_();
	_nop_();
	_nop_();
	ep = 0;
}

void lcd_init()
{
	delay100us();		
	lcd_wcmd(0x38); //8bit,1/16,5x8
	delay100us();
	lcd_wcmd(0x06); ///AC auto +1
	delay100us();
	lcd_wcmd(0x0c); //disp enable
	delay100us();
	lcd_wcmd(0x01); //??LCD?????
	delay100us();
}
