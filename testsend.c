#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "reg51.h"
#include "intrins.h"

#define		INT8U		unsigned char
#define		INT16U		unsigned int

#define FOSC 11059200L      //System frequency
#define BAUD 9600           //UART baudrate

#define 	WRITE_BURST     	0x40						//����д��
#define 	READ_SINGLE     	0x80						//��
#define 	READ_BURST      	0xC0						//������
#define 	BYTES_IN_RXFIFO     0x7F  						//���ջ���������Ч�ֽ���
#define 	CRC_OK              0x80 						//CRCУ��ͨ��λ��־

#define		NODE_ACK_WAIT_TIME		0x80					//���ճ�ʱ
//*****************************************************************************************
/*
sbit 	GDO0	=P3^7;
sbit	MISO	=P3^3;
sbit	MOSI	=P3^2;
sbit	SCK		=P3^4;
sbit 	GDO2	=P3^5;  
sbit	CSN		=P3^6;  
sbit  PAC   =P2^7; 
sbit DIO = P1^3;				//595 io
sbit RCLK  = P1^2;			//595 rclk
sbit SCLK = P1^1;				// 595 sclk
*/
sbit 	GDO0	=P1^6;
sbit	MISO	=P1^3;
sbit	MOSI	=P1^1;
sbit	SCK		=P1^2;
sbit 	GDO2	=P1^4;  
sbit	CSN		=P1^7;  
sbit  PAC   =P1^5; 

sbit  P35	= P3^5;
sbit 	P34 = P3^4;

sbit pinLED = P1^0;
//*****************************************************************************************

//***************���๦�ʲ������ÿ���ϸ�ο�DATACC1100Ӣ���ĵ��е�48-49ҳ�Ĳ�����******************
//INT8U PaTabel[8] = {0x04 ,0x04 ,0x04 ,0x04 ,0x04 ,0x04 ,0x04 ,0x04};  //-30dBm   ������С
//INT8U PaTabel[8] = {0x60 ,0x60 ,0x60 ,0x60 ,0x60 ,0x60 ,0x60 ,0x60};  //0dBm
INT8U PaTabel[8] = {0xC0 ,0xC0 ,0xC0 ,0xC0 ,0xC0 ,0xC0 ,0xC0 ,0xC0};   //10dBm     �������

//************** COMM PROTOCOL 
#define PDA_CNT 0x10
INT8U xdata APTable[20];
INT8U xdata PDATable[PDA_CNT];
//************** UART Process 
#define SBUF_SIZE 32
INT8U xdata sUart[SBUF_SIZE];
INT8U xdata RFBuf[SBUF_SIZE];
INT8U xdata sTmp[32];
INT8U data err=0;

INT8U data chnl = 0x00;
INT8U data myaddr = 0x01;

bit busy;

//*****************************************************************************************
void SpiInit(void);
void CpuInit(void);
void RESET_CC1100(void);
void POWER_UP_RESET_CC1100(void);
void halSpiWriteReg(INT8U addr, INT8U value);
void halSpiWriteBurstReg(INT8U addr, INT8U *buffer, INT8U count);
void halSpiStrobe(INT8U strobe);
INT8U halSpiReadReg(INT8U addr);
void halSpiReadBurstReg(INT8U addr, INT8U *buffer, INT8U count);
INT8U halSpiReadStatus(INT8U addr);
void halRfWriteRfSettings(void);
void halRfSendPacket(INT8U *txBuffer, INT8U size); 
INT8U halRfReceivePacket(INT8U *rxBuffer, INT8U *length);  
//*****************************************************************************************
// CC1100 STROBE, CONTROL AND STATUS REGSITER
#define CCxxx0_IOCFG2       0x00        // GDO2 output pin configuration
#define CCxxx0_IOCFG1       0x01        // GDO1 output pin configuration
#define CCxxx0_IOCFG0       0x02        // GDO0 output pin configuration
#define CCxxx0_FIFOTHR      0x03        // RX FIFO and TX FIFO thresholds
#define CCxxx0_SYNC1        0x04        // Sync word, high INT8U
#define CCxxx0_SYNC0        0x05        // Sync word, low INT8U
#define CCxxx0_PKTLEN       0x06        // Packet length
#define CCxxx0_PKTCTRL1     0x07        // Packet automation control
#define CCxxx0_PKTCTRL0     0x08        // Packet automation control
#define CCxxx0_ADDR         0x09        // Device address
#define CCxxx0_CHANNR       0x0A        // Channel number
#define CCxxx0_FSCTRL1      0x0B        // Frequency synthesizer control
#define CCxxx0_FSCTRL0      0x0C        // Frequency synthesizer control
#define CCxxx0_FREQ2        0x0D        // Frequency control word, high INT8U
#define CCxxx0_FREQ1        0x0E        // Frequency control word, middle INT8U
#define CCxxx0_FREQ0        0x0F        // Frequency control word, low INT8U
#define CCxxx0_MDMCFG4      0x10        // Modem configuration
#define CCxxx0_MDMCFG3      0x11        // Modem configuration
#define CCxxx0_MDMCFG2      0x12        // Modem configuration
#define CCxxx0_MDMCFG1      0x13        // Modem configuration
#define CCxxx0_MDMCFG0      0x14        // Modem configuration
#define CCxxx0_DEVIATN      0x15        // Modem deviation setting
#define CCxxx0_MCSM2        0x16        // Main Radio Control State Machine configuration
#define CCxxx0_MCSM1        0x17        // Main Radio Control State Machine configuration
#define CCxxx0_MCSM0        0x18        // Main Radio Control State Machine configuration
#define CCxxx0_FOCCFG       0x19        // Frequency Offset Compensation configuration
#define CCxxx0_BSCFG        0x1A        // Bit Synchronization configuration
#define CCxxx0_AGCCTRL2     0x1B        // AGC control
#define CCxxx0_AGCCTRL1     0x1C        // AGC control
#define CCxxx0_AGCCTRL0     0x1D        // AGC control
#define CCxxx0_WOREVT1      0x1E        // High INT8U Event 0 timeout
#define CCxxx0_WOREVT0      0x1F        // Low INT8U Event 0 timeout
#define CCxxx0_WORCTRL      0x20        // Wake On Radio control
#define CCxxx0_FREND1       0x21        // Front end RX configuration
#define CCxxx0_FREND0       0x22        // Front end TX configuration
#define CCxxx0_FSCAL3       0x23        // Frequency synthesizer calibration
#define CCxxx0_FSCAL2       0x24        // Frequency synthesizer calibration
#define CCxxx0_FSCAL1       0x25        // Frequency synthesizer calibration
#define CCxxx0_FSCAL0       0x26        // Frequency synthesizer calibration
#define CCxxx0_RCCTRL1      0x27        // RC oscillator configuration
#define CCxxx0_RCCTRL0      0x28        // RC oscillator configuration
#define CCxxx0_FSTEST       0x29        // Frequency synthesizer calibration control
#define CCxxx0_PTEST        0x2A        // Production test
#define CCxxx0_AGCTEST      0x2B        // AGC test
#define CCxxx0_TEST2        0x2C        // Various test settings
#define CCxxx0_TEST1        0x2D        // Various test settings
#define CCxxx0_TEST0        0x2E        // Various test settings

// Strobe commands
#define CCxxx0_SRES         0x30        // Reset chip.
#define CCxxx0_SFSTXON      0x31        // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).
                                        // If in RX/TX: Go to a wait state where only the synthesizer is
                                        // running (for quick RX / TX turnaround).
#define CCxxx0_SXOFF        0x32        // Turn off crystal oscillator.
#define CCxxx0_SCAL         0x33        // Calibrate frequency synthesizer and turn it off
                                        // (enables quick start).
#define CCxxx0_SRX          0x34        // Enable RX. Perform calibration first if coming from IDLE and
                                        // MCSM0.FS_AUTOCAL=1.
#define CCxxx0_STX          0x35        // In IDLE state: Enable TX. Perform calibration first if
                                        // MCSM0.FS_AUTOCAL=1. If in RX state and CCA is enabled:
                                        // Only go to TX if channel is clear.
#define CCxxx0_SIDLE        0x36        // Exit RX / TX, turn off frequency synthesizer and exit
                                        // Wake-On-Radio mode if applicable.
#define CCxxx0_SAFC         0x37        // Perform AFC adjustment of the frequency synthesizer
#define CCxxx0_SWOR         0x38        // Start automatic RX polling sequence (Wake-on-Radio)
#define CCxxx0_SPWD         0x39        // Enter power down mode when CSn goes high.
#define CCxxx0_SFRX         0x3A        // Flush the RX FIFO buffer.
#define CCxxx0_SFTX         0x3B        // Flush the TX FIFO buffer.
#define CCxxx0_SWORRST      0x3C        // Reset real time clock.
#define CCxxx0_SNOP         0x3D        // No operation. May be used to pad strobe commands to two
                                        // INT8Us for simpler software.

#define CCxxx0_PARTNUM      0x30
#define CCxxx0_VERSION      0x31
#define CCxxx0_FREQEST      0x32
#define CCxxx0_LQI          0x33
#define CCxxx0_RSSI         0x34
#define CCxxx0_MARCSTATE    0x35
#define CCxxx0_WORTIME1     0x36
#define CCxxx0_WORTIME0     0x37
#define CCxxx0_PKTSTATUS    0x38
#define CCxxx0_VCO_VC_DAC   0x39
#define CCxxx0_TXBYTES      0x3A
#define CCxxx0_RXBYTES      0x3B

#define CCxxx0_PATABLE      0x3E
#define CCxxx0_TXFIFO       0x3F
#define CCxxx0_RXFIFO       0x3F

// RF_SETTINGS is a data structure which contains all relevant CCxxx0 registers
typedef struct S_RF_SETTINGS
{
	INT8U FSCTRL2;		//���Ѽӵ�
    INT8U FSCTRL1;   // Frequency synthesizer control.
    INT8U FSCTRL0;   // Frequency synthesizer control.
    INT8U FREQ2;     // Frequency control word, high INT8U.
    INT8U FREQ1;     // Frequency control word, middle INT8U.
    INT8U FREQ0;     // Frequency control word, low INT8U.
    INT8U MDMCFG4;   // Modem configuration.
    INT8U MDMCFG3;   // Modem configuration.
    INT8U MDMCFG2;   // Modem configuration.
    INT8U MDMCFG1;   // Modem configuration.
    INT8U MDMCFG0;   // Modem configuration.
    INT8U CHANNR;    // Channel number.
    INT8U DEVIATN;   // Modem deviation setting (when FSK modulation is enabled).
    INT8U FREND1;    // Front end RX configuration.
    INT8U FREND0;    // Front end RX configuration.
    INT8U MCSM0;     // Main Radio Control State Machine configuration.
    INT8U FOCCFG;    // Frequency Offset Compensation Configuration.
    INT8U BSCFG;     // Bit synchronization Configuration.
    INT8U AGCCTRL2;  // AGC control.
	INT8U AGCCTRL1;  // AGC control.
    INT8U AGCCTRL0;  // AGC control.
    INT8U FSCAL3;    // Frequency synthesizer calibration.
    INT8U FSCAL2;    // Frequency synthesizer calibration.
	INT8U FSCAL1;    // Frequency synthesizer calibration.
    INT8U FSCAL0;    // Frequency synthesizer calibration.
    INT8U FSTEST;    // Frequency synthesizer calibration control
    INT8U TEST2;     // Various test settings.
    INT8U TEST1;     // Various test settings.
    INT8U TEST0;     // Various test settings.
    INT8U IOCFG2;    // GDO2 output pin configuration
    INT8U IOCFG0;    // GDO0 output pin configuration
    INT8U PKTCTRL1;  // Packet automation control.
    INT8U PKTCTRL0;  // Packet automation control.
    INT8U ADDR;      // Device address.
    INT8U PKTLEN;    // Packet length.
} RF_SETTINGS;

/////////////////////////////////////////////////////////////////
const RF_SETTINGS rfSettings = 
{
	0x00,
    0x08,   // FSCTRL1   Frequency synthesizer control.
    0x00,   // FSCTRL0   Frequency synthesizer control.
    0x10,   // FREQ2     Frequency control word, high byte.
    0xA7,   // FREQ1     Frequency control word, middle byte.
    0x62,   // FREQ0     Frequency control word, low byte.
    0x5B,   // MDMCFG4   Modem configuration.
    0xF8,   // MDMCFG3   Modem configuration.
    0x03,   // MDMCFG2   Modem configuration.
    0x22,   // MDMCFG1   Modem configuration.
    0xF8,   // MDMCFG0   Modem configuration.

    0x00,   // CHANNR    Channel number.
    0x47,   // DEVIATN   Modem deviation setting (when FSK modulation is enabled).
    0xB6,   // FREND1    Front end RX configuration.
    0x10,   // FREND0    Front end RX configuration.
    0x18,   // MCSM0     Main Radio Control State Machine configuration.
    0x1D,   // FOCCFG    Frequency Offset Compensation Configuration.
    0x1C,   // BSCFG     Bit synchronization Configuration.
    0xC7,   // AGCCTRL2  AGC control.
    0x00,   // AGCCTRL1  AGC control.
    0xB2,   // AGCCTRL0  AGC control.

    0xEA,   // FSCAL3    Frequency synthesizer calibration.
    0x2A,   // FSCAL2    Frequency synthesizer calibration.
    0x00,   // FSCAL1    Frequency synthesizer calibration.
    0x11,   // FSCAL0    Frequency synthesizer calibration.
    0x59,   // FSTEST    Frequency synthesizer calibration.
    0x81,   // TEST2     Various test settings.
    0x35,   // TEST1     Various test settings.
    0x09,   // TEST0     Various test settings.
    0x0B,   // IOCFG2    GDO2 output pin configuration.
    0x06,   // IOCFG0D   GDO0 output pin configuration. Refer to SmartRF?Studio User Manual for detailed pseudo register explanation.

    0x06,   // PKTCTRL1  Packet automation control.
    0x05,   // PKTCTRL0  Packet automation control.
    0x00,   // ADDR      Device address.
    0x7F    // PKTLEN    Packet length.
};
//*****************************************************************************************
//��������delay(unsigned int s)
//���룺ʱ��
//�������
//������������ͨ͢ʱ,�ڲ���
//*****************************************************************************************		
static void delay(unsigned int s)
{
	unsigned int i;
	for(i=0; i<s; i++);
	for(i=0; i<s; i++);
}


void halWait(INT16U timeout) {
    do {
        _nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_(); 
    } while (--timeout);
}

/********************************************
UART Functions
********************************************/
void SendUartByte(INT8U c)
{
	TI=0;
	SBUF = c;
	while (!TI);
	TI=0;
}
void SendString(char *s)
{
    while (*s)              //Check the end of the string
    {
        SendUartByte(*s++);     //Send current char and increment string ptr
    }
}

void SendUartBytes(INT8U *s, INT8U len)
{
	INT8U i;
	for (i=0; i<len; i++) SendUartByte(s[i]);
}

void InitUart(void)
{
		ES=0;	
    SCON = 0x50;            //8-bit variable UART
    TMOD = 0x20;            //Set Timer1 as 8-bit auto reload mode
    TH1 = TL1 = -(FOSC/12/32/BAUD); //Set auto-reload vaule
    TR1 = 1;                //Timer1 start run
    ES = 1;                 //Enable UART interrupt
    EA = 1;                 //Open master interrupt switch

//    SendString("STC89-90xx\r\nUart Test !\r\n");
	
}

void Uart_Isr() interrupt 4 using 1
{
    if (RI)
    {
        RI = 0;             //Clear receive interrupt flag
       // P0 = SBUF;          //P0 show UART data
				if (SBUF==0x7F) {
					memset(sUart,0x00,SBUF_SIZE);
					RI=0;
					return;
				}
				if (sUart[0]<SBUF_SIZE){
					sUart[sUart[0]+1]=SBUF;
					sUart[0]++;
					} 
		}
    if (TI)
    {
        TI = 0;             //Clear transmit interrupt flag
        busy = 0;           //Clear transmit busy flag
    }
}

INT8U UartDataOK(void)
{
	if ((sUart[0]>sUart[1]) && (sUart[0])) return 1; 
	return 0;
}

void SpiInit(void)
{
	CSN=0;
	SCK=0;
	CSN=1;
}

/*****************************************************************************************
//��������CpuInit()
//���룺��
//�������
//����������SPI��ʼ������
/*****************************************************************************************/
void CpuInit(void)
{
	SpiInit();
	delay(5000);
}
	

//*****************************************************************************************
//��������SpisendByte(INT8U dat)
//���룺���͵�����
//�������
//����������SPI����һ���ֽ�
//*****************************************************************************************
INT8U SpiTxRxByte(INT8U dat)
{
	INT8U i,temp;
	temp = 0;
	
	SCK = 0;
	for(i=0; i<8; i++)
	{
		if(dat & 0x80)
		{
			MOSI = 1;
		}
		else MOSI = 0;
		dat <<= 1;

		SCK = 1; 
		_nop_();
		_nop_();

		temp <<= 1;
		if(MISO)temp++; 
		SCK = 0;
		_nop_();
		_nop_();	
	}
	return temp;
}

//*****************************************************************************************
//��������void RESET_CC1100(void)
//���룺��
//�������
//������������λCC1100
//*****************************************************************************************
void RESET_CC1100(void) 
{
	CSN = 0; 
	while (MISO);
    SpiTxRxByte(CCxxx0_SRES); 		//д�븴λ����
	while (MISO); 
    CSN = 1; 
}

//*****************************************************************************************
//��������void POWER_UP_RESET_CC1100(void) 
//���룺��
//�������
//�����������ϵ縴λCC1100
//*****************************************************************************************
void POWER_UP_RESET_CC1100(void) 
{
	CSN = 1; 
	halWait(1); 
	CSN = 0; 
	halWait(1); 
	CSN = 1; 
	halWait(41); 
	RESET_CC1100();   		//��λCC1100
}

//*****************************************************************************************
//��������void halSpiWriteReg(INT8U addr, INT8U value)
//���룺��ַ��������
//�������
//����������SPIд�Ĵ���
//*****************************************************************************************
void halSpiWriteReg(INT8U addr, INT8U value) 
{
    CSN = 0;
    while (MISO);
    SpiTxRxByte(addr);		//д��ַ
    SpiTxRxByte(value);		//д������
    CSN = 1;
}

//*****************************************************************************************
//��������void halSpiWriteBurstReg(INT8U addr, INT8U *buffer, INT8U count)
//���룺��ַ��д�뻺������д�����
//�������
//����������SPI����д���üĴ���
//*****************************************************************************************
void halSpiWriteBurstReg(INT8U addr, INT8U *buffer, INT8U count) 
{
    INT8U i, temp;
	temp = addr | WRITE_BURST;
    CSN = 0;
    while (MISO);
    SpiTxRxByte(temp);
    for (i = 0; i < count; i++)
 	{
        SpiTxRxByte(buffer[i]);
    }
    CSN = 1;
}

//*****************************************************************************************
//��������void halSpiStrobe(INT8U strobe)
//���룺����
//�������
//����������SPIд����
//*****************************************************************************************
void halSpiStrobe(INT8U strobe) 
{
    CSN = 0;
    while (MISO);
    SpiTxRxByte(strobe);		//д������
    CSN = 1;
}





//*****************************************************************************************
//��������INT8U halSpiReadReg(INT8U addr)
//���룺��ַ
//������üĴ�����������
//����������SPI���Ĵ���
//*****************************************************************************************
INT8U halSpiReadReg(INT8U addr) 
{
	INT8U temp, value;
    temp = addr|READ_SINGLE;//���Ĵ�������
	CSN = 0;
	while (MISO);
	SpiTxRxByte(temp);
	value = SpiTxRxByte(0);
	CSN = 1;
	return value;
}


//*****************************************************************************************
//��������void halSpiReadBurstReg(INT8U addr, INT8U *buffer, INT8U count)
//���룺��ַ���������ݺ��ݴ�Ļ��������������ø���
//�������
//����������SPI����д���üĴ���
//*****************************************************************************************
void halSpiReadBurstReg(INT8U addr, INT8U *buffer, INT8U count) 
{
    INT8U i,temp;
	temp = addr | READ_BURST;		//д��Ҫ�������üĴ�����ַ�Ͷ�����
    CSN = 0;
    while (MISO);
	SpiTxRxByte(temp);   
    for (i = 0; i < count; i++) 
	{
        buffer[i] = SpiTxRxByte(0);
    }
    CSN = 1;
}


//*****************************************************************************************
//��������INT8U halSpiReadReg(INT8U addr)
//���룺��ַ
//�������״̬�Ĵ�����ǰֵ
//����������SPI��״̬�Ĵ���
//*****************************************************************************************
INT8U halSpiReadStatus(INT8U addr) 
{
    INT8U value,temp;
	temp = addr | READ_BURST;		//д��Ҫ����״̬�Ĵ����ĵ�ַͬʱд�������
    CSN = 0;
    while (MISO);
    SpiTxRxByte(temp);
	value = SpiTxRxByte(0);
	CSN = 1;
	return value;
}
//*****************************************************************************************
//��������void halRfWriteRfSettings(RF_SETTINGS *pRfSettings)
//���룺��
//�������
//��������������CC1100�ļĴ���
//*****************************************************************************************
void halRfWriteRfSettings(void) 
{

	halSpiWriteReg(CCxxx0_FSCTRL0,  rfSettings.FSCTRL2);//���Ѽӵ�
    // Write register settings
    halSpiWriteReg(CCxxx0_FSCTRL1,  rfSettings.FSCTRL1);
    halSpiWriteReg(CCxxx0_FSCTRL0,  rfSettings.FSCTRL0);
    halSpiWriteReg(CCxxx0_FREQ2,    rfSettings.FREQ2);
    halSpiWriteReg(CCxxx0_FREQ1,    rfSettings.FREQ1);
    halSpiWriteReg(CCxxx0_FREQ0,    rfSettings.FREQ0);
    halSpiWriteReg(CCxxx0_MDMCFG4,  rfSettings.MDMCFG4);
    halSpiWriteReg(CCxxx0_MDMCFG3,  rfSettings.MDMCFG3);
    halSpiWriteReg(CCxxx0_MDMCFG2,  rfSettings.MDMCFG2);
    halSpiWriteReg(CCxxx0_MDMCFG1,  rfSettings.MDMCFG1);
    halSpiWriteReg(CCxxx0_MDMCFG0,  rfSettings.MDMCFG0);
    halSpiWriteReg(CCxxx0_CHANNR,   rfSettings.CHANNR);
    halSpiWriteReg(CCxxx0_DEVIATN,  rfSettings.DEVIATN);
    halSpiWriteReg(CCxxx0_FREND1,   rfSettings.FREND1);
    halSpiWriteReg(CCxxx0_FREND0,   rfSettings.FREND0);
    halSpiWriteReg(CCxxx0_MCSM0 ,   rfSettings.MCSM0 );
    halSpiWriteReg(CCxxx0_FOCCFG,   rfSettings.FOCCFG);
    halSpiWriteReg(CCxxx0_BSCFG,    rfSettings.BSCFG);
    halSpiWriteReg(CCxxx0_AGCCTRL2, rfSettings.AGCCTRL2);
	halSpiWriteReg(CCxxx0_AGCCTRL1, rfSettings.AGCCTRL1);
    halSpiWriteReg(CCxxx0_AGCCTRL0, rfSettings.AGCCTRL0);
    halSpiWriteReg(CCxxx0_FSCAL3,   rfSettings.FSCAL3);
	halSpiWriteReg(CCxxx0_FSCAL2,   rfSettings.FSCAL2);
	halSpiWriteReg(CCxxx0_FSCAL1,   rfSettings.FSCAL1);
    halSpiWriteReg(CCxxx0_FSCAL0,   rfSettings.FSCAL0);
    halSpiWriteReg(CCxxx0_FSTEST,   rfSettings.FSTEST);
    halSpiWriteReg(CCxxx0_TEST2,    rfSettings.TEST2);
    halSpiWriteReg(CCxxx0_TEST1,    rfSettings.TEST1);
    halSpiWriteReg(CCxxx0_TEST0,    rfSettings.TEST0);
    halSpiWriteReg(CCxxx0_IOCFG2,   rfSettings.IOCFG2);
    halSpiWriteReg(CCxxx0_IOCFG0,   rfSettings.IOCFG0);    
    halSpiWriteReg(CCxxx0_PKTCTRL1, rfSettings.PKTCTRL1);
    halSpiWriteReg(CCxxx0_PKTCTRL0, rfSettings.PKTCTRL0);
    halSpiWriteReg(CCxxx0_ADDR,     rfSettings.ADDR);
    halSpiWriteReg(CCxxx0_PKTLEN,   rfSettings.PKTLEN);
}

//*****************************************************************************************
//��������int halRfSendPacketAddr(INT8U *txBuffer, INT8U size, INT8U addr)
//���룺���͵Ļ��������������ݸ����� Ŀ�ĵ�ַ
//�����
//����������CC1100����һ������
//*****************************************************************************************
int halRfSendPacketAddr(INT8U *txBuffer, INT8U size, INT8U addr)
{
	unsigned int i=0;
	PAC=1;	//������
	halSpiWriteReg(CCxxx0_TXFIFO, size+1);		//input length of data to fifo buffer
	halSpiWriteReg(CCxxx0_TXFIFO, addr);		//dest to fifo
    halSpiWriteBurstReg(CCxxx0_TXFIFO, txBuffer, size);	//д��Ҫ���͵�����
    halSpiStrobe(CCxxx0_STX);		//���뷢��ģʽ��������

    // Wait for GDO0 to be set -> sync transmitted
	i=0;
    while (!GDO0) {
		if (i>10000) {PAC=0; return -1;}
		halWait(1);
		i++;
	}

    // Wait for GDO0 to be cleared -> end of packet
	i=0;
    while (GDO0){
		if (i>60000) {PAC=0; return -2;}
		halWait(1);
		i++;
	}

	halSpiStrobe(CCxxx0_SFTX);	//flush tx buffer
	PAC=0;		//�ع���
	return 1;
}

void setRxMode(void)
{
    halSpiStrobe(CCxxx0_SRX);		//�������״̬
}

INT8U halRfReceivePacket(INT8U *rxBuffer, INT8U *length) 
{
    INT8U status[2];
    INT8U packetLength;
	INT8U i=(63)*4;  // �������Ҫ����datarate��length������

    halSpiStrobe(CCxxx0_SRX);		//�������״̬
	delay(2);
	while (GDO0)
	{
		delay(2);
		--i;
		if(i<1) {
		   return 0;
		} 	    
	}
    if ((halSpiReadStatus(CCxxx0_RXBYTES) & BYTES_IN_RXFIFO)) //����ӵ��ֽ�����Ϊ0
	{
        packetLength = halSpiReadReg(CCxxx0_RXFIFO);//������һ���ֽڣ����ֽ�Ϊ��֡���ݳ���
        if (packetLength >0) // <= *length) 		//�����Ҫ����Ч���ݳ���С�ڵ��ڽ��յ������ݰ��ĳ���
		{
            halSpiReadBurstReg(CCxxx0_RXFIFO, rxBuffer, packetLength); //�������н��յ�������
            *length = packetLength;				//�ѽ������ݳ��ȵ��޸�Ϊ��ǰ���ݵĳ���
            // Read the 2 appended status bytes (status[0] = RSSI, status[1] = LQI)
            halSpiReadBurstReg(CCxxx0_RXFIFO, status, 2); 	//����CRCУ��λ
			halSpiStrobe(CCxxx0_SFRX);		//��ϴ���ջ�����
            return (status[1] & CRC_OK);			//���У��ɹ����ؽ��ճɹ�
        }
		 else 
		{
            *length = packetLength;
            halSpiStrobe(CCxxx0_SFRX);		//��ϴ���ջ�����
            return 0;
        }
    } 
	else {
 	return 0; }
}


void SetRfChannel(INT8U chn)
{
	halSpiWriteReg(CCxxx0_CHANNR, chn);
}

void SetRfAddr(INT8U addr)
{
	halSpiWriteReg(CCxxx0_ADDR, addr);
}


void RestartCC1100()
{
	POWER_UP_RESET_CC1100();
	halRfWriteRfSettings();
	halSpiWriteBurstReg(CCxxx0_PATABLE, PaTabel, 8);
	SetRfAddr(myaddr);
	SetRfChannel(chnl);

}

void testsend()
{
	INT8U i;
	//memcpy(RFBuf,"ABCDEFGHIJK",16);
	memset(RFBuf,0x00,SBUF_SIZE);
	i=sUart[1];
	memcpy(RFBuf,sUart+2,i);
pinLED=1;
	i=halRfSendPacketAddr(RFBuf,i,01);	// Transmit Tx buffer data
	memset(sUart,0x00,SBUF_SIZE);
pinLED=0;
}

void main(void)
{
	PAC=0;
	InitUart();
	CpuInit();
	memset(sUart,0x00,SBUF_SIZE);
	memset(PDATable,0x00,PDA_CNT);
	PDATable[0]=0x01;
	chnl=1;
	RestartCC1100();
	while(1)
	{		
		P34=1;
		if(!P34){
			memcpy(sUart+2,"0123456789ABCDEF",16);
			sUart[0]=17;
			sUart[1]=17;
		}
		if (UartDataOK()){
//			memcpy(sUart+2,"ABCDEFGHIJKLMNOP",16);
//			sUart[0]=17;
//			sUart[1]=17;
			RestartCC1100();
			testsend();
		}
	}
	
}

