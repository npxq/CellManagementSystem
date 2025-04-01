/*
 * main.c
 *
 * This module demonstrates operation of CRC with the bq769x0 family
 * AFE devices using a MSP430G2553 
 *
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

#include <msp430.h> 
#include <stdbool.h>
#include <bqMaximo_Ctrl_G2553.h>
#include <system_settings.h>
#include <math.h>
#include <float.h>
#include <stdlib.h>
/*
 * main.c
 */

RegisterGroup Registers;

const unsigned int OVPThreshold = 4300;
const unsigned int UVPThreshold = 2500;
const unsigned char SCDDelay = SCD_DELAY_100us;
const unsigned char SCDThresh = SCD_THRESH_89mV_44mV;
const unsigned char OCDDelay = OCD_DELAY_320ms;
const unsigned char OCDThresh = OCD_THRESH_22mV_11mV;
const unsigned char OVDelay = OV_DELAY_2s;
const unsigned char UVDelay = UV_DELAY_8s;

unsigned int CellVoltage[15];
float Gain = 0;
int iGain = 0;

void ClockInitialise()
{
	DCOCTL = 0xE0;	//DCO = 7, MOD = 0

	BCSCTL1 &= 0x8F;  //RSEL = 0, DCOR = 0, 20MHz Clock

	BCSCTL2 = 0x0;

	BCSCTL3 = 0;
}

void TimerAInit()
{
	TA0CTL |= TASSEL1;
	TA0CTL &= ~TASSEL0;

	TA0CTL |= MC1;
	TA0CTL &= ~MC0;

	TA0CTL &= ~TAIE;

}

void I2CInitialise()
{
	P1SEL |= BIT6 + BIT7;                     // Assign I2C pins to USCI_B0, P1.6 for SCL and P1.7 for SDA
	P1SEL2|= BIT6 + BIT7;                     // Assign I2C pins to USCI_B0, P1.6 for SCL and P1.7 for SDA

	ADC10AE0 &= 0x3F;
	CAPD &= 0x3F;

	UCB0CTL1 |= UCSWRST;                      // Enable SW reset, hold USCI logic in reset state
	UCB0CTL0 = UCMODE_3 + UCSYNC;			//set to I2C mode, sync=1
	UCB0BR0 = 20;
	UCB0BR1 = 0;

	UCB0I2CIE = 0;
    IE2 &= ~(UCB0TXIE + UCB0RXIE);	//disable interrupts

    UCB0CTL1 |= UCSSEL_2;
    UCB0CTL1 &= ~UCSWRST;
}

int I2CSendByte(unsigned char I2CSlaveAddress, unsigned char data)
{
	unsigned long int DelayCounter = 0;

	UCB0CTL0 |= UCMST;
	UCB0I2CSA = I2CSlaveAddress;

	 UCB0CTL1 |= UCTR; //data in transmit direction
	 UCB0CTL1 |= UCTXSTT; //Generate Start Condition
	 	 	 	 	 	  //Send Start Byte

	 while(!(IFG2 & UCB0TXIFG))  //if UCB0TXIFG != 0, wait here
	 {
		 DelayCounter ++;
		 if (DelayCounter >= DELAY_LIMIT)
			 break;
	 }

	 if (DelayCounter >= DELAY_LIMIT)
		 return -1;

	 UCB0TXBUF = data;				// send the data

	 DelayCounter = 0;

	 while(DelayCounter < DELAY_LIMIT && !(IFG2 & UCB0TXIFG))
	 {
		 DelayCounter++;
	 }

	 if (DelayCounter >= DELAY_LIMIT)
		 return -1;

	 UCB0CTL1 |= UCTXSTP;			//send stop bit

	 DelayCounter = 0;

	 while(DelayCounter < DELAY_LIMIT && (UCB0CTL1 & UCTXSTP))
	 {
		 DelayCounter++;
	 }

	 if (DelayCounter >= DELAY_LIMIT)	//check if NACK condition occurred
		 return -1;
	 else
		 return 0;

}

int I2CSendBytes(unsigned char I2CSlaveAddress, unsigned char *DataBuffer, unsigned int ByteCount, unsigned int *SentByte)
{
	unsigned long int DelayCounter = 0;
    unsigned int NumberOfBytesSent = 0;
    unsigned char *DataPointer;

	UCB0CTL0 |= UCMST;
	UCB0I2CSA = I2CSlaveAddress;

     DataPointer = DataBuffer;

	 UCB0CTL1 |= UCTR; //data in transmit direction
	 UCB0CTL1 |= UCTXSTT; //Generate Start Condition
	 	 	 	 	 	  //Send Start Byte

	 while(!(IFG2 & UCB0TXIFG))  //if UCTXSTT != 0, wait here
	 {
		 DelayCounter ++;
		 if (DelayCounter > DELAY_LIMIT)
			 break;
	 }

	 if (DelayCounter >= DELAY_LIMIT)   //check if NACK condition occurred
	 {
		 *SentByte = NumberOfBytesSent;
		 UCB0CTL1 |= UCTXSTP;
		 return -1;
	 }

	 for(NumberOfBytesSent = 0; NumberOfBytesSent < ByteCount; NumberOfBytesSent++)
	 {
		 UCB0TXBUF= *DataPointer;

		 DelayCounter = 0;

		 while(DelayCounter < DELAY_LIMIT && (!(IFG2 & UCB0TXIFG) || (UCB0CTL1 & UCTXSTT)))	//check if the byte has been sent
		 {
			 DelayCounter++;
		 }

		 if (DelayCounter >= DELAY_LIMIT)	//check if NACK condition occurred
		 {
			 *SentByte = NumberOfBytesSent;
			 UCB0CTL1 |= UCTXSTP;				//send stop condition
			 return -1;
		 }

		 DataPointer++;
	 }

	 IFG2 &= ~UCB0TXIFG;
	 UCB0CTL1 |= UCTXSTP;		//send stop bit

	 DelayCounter = 0;

	 while(DelayCounter < DELAY_LIMIT && ((UCB0CTL1 & UCTXSTP)))
	 {
		 DelayCounter++;
	 }

	 *SentByte =  NumberOfBytesSent;

	 if (DelayCounter >= DELAY_LIMIT)	//check if NACK condition occurred
	 {
		 UCB0CTL1 |= UCSWRST;
		 return -1;
	 }
	 else
		 return 0;
}

int I2CWriteRegisterByte(unsigned char I2CSlaveAddress, unsigned char Register, unsigned char Data)
{
	unsigned char DataBuffer[2];
	unsigned int SentByte = 0;


	DataBuffer[0] = Register;
	DataBuffer[1] = Data;

	return(I2CSendBytes(I2CSlaveAddress, DataBuffer, 2, &SentByte));
}

int I2CWriteRegisterByteWithCRC(unsigned char I2CSlaveAddress, unsigned char Register, unsigned char Data)
{
	unsigned char DataBuffer[4];
	unsigned int SentByte = 0;

    DataBuffer[0] = I2CSlaveAddress << 1;
	DataBuffer[1] = Register;
	DataBuffer[2] = Data;
	DataBuffer[3] = CRC8(DataBuffer, 3, CRC_KEY);

	return(I2CSendBytes(I2CSlaveAddress, DataBuffer + 1, 3, &SentByte));
}

int I2CWriteRegisterWordWithCRC(unsigned char I2CSlaveAddress, unsigned char Register, unsigned int Data)
{
	unsigned char DataBuffer[6];
	unsigned int SentByte = 0;

    DataBuffer[0] = I2CSlaveAddress << 1;
	DataBuffer[1] = Register;
	DataBuffer[2] = LOW_BYTE(Data);
	DataBuffer[3] = CRC8(DataBuffer, 3, CRC_KEY);
	DataBuffer[4] = HIGH_BYTE(Data);
	DataBuffer[5] = CRC8(DataBuffer + 4, 1, CRC_KEY);

	return(I2CSendBytes(I2CSlaveAddress, DataBuffer + 1, 5, &SentByte));
}

int I2CWriteBlockWithCRC(unsigned char I2CSlaveAddress, unsigned char StartAddress, unsigned char *Buffer, unsigned char Length)
{
	unsigned char *BufferCRC, *Pointer;
	int i;
	unsigned int SentByte = 0;
	int result;

	BufferCRC = (unsigned char*)malloc(2*Length + 2);
	if (NULL == BufferCRC)
		return -1;

	Pointer = BufferCRC;
	*Pointer = I2CSlaveAddress << 1;
	Pointer++;
	*Pointer = StartAddress;
	Pointer++;
	*Pointer = *Buffer;
	Pointer++;
	*Pointer = CRC8(BufferCRC, 3, CRC_KEY);

	for(i = 1; i < Length; i++)
	{
        Pointer++;
        Buffer++;
        *Pointer = *Buffer;
		*(Pointer + 1) = CRC8(Pointer, 1, CRC_KEY);
		Pointer++;
	}

	result = I2CSendBytes(I2CSlaveAddress, BufferCRC + 1, 2*Length + 1, &SentByte);

	free(BufferCRC);
	BufferCRC = NULL;

	return result;
}

int I2CWriteRegisterWord(unsigned char I2CSlaveAddress, unsigned char Register, unsigned int Data)
{
	unsigned char DataBuffer[3];
	unsigned int SentByte = 0;

	DataBuffer[0] = Register;
	DataBuffer[1] = LOWBYTE(Data);
	DataBuffer[2] = HIGHBYTE(Data);

	return(I2CSendBytes(I2CSlaveAddress, DataBuffer, 3, &SentByte));
}

int I2CReadBytes(unsigned char I2CSlaveAddress, unsigned char *DataBuffer, unsigned int ExpectedByteNumber, unsigned int *NumberOfReceivedBytes)
{
	unsigned long int DelayCounter = 0;
    unsigned char *DataPointer;
    unsigned int *NumberOfReceivedBytesPointer;

    NumberOfReceivedBytesPointer = NumberOfReceivedBytes;
    *NumberOfReceivedBytesPointer = 0;

    UCB0CTL0 |= UCMST;
    DataPointer = DataBuffer;
	UCB0I2CSA = I2CSlaveAddress;

	UCB0CTL1 &= ~(UCTR); //data in receive direction

	UCB0CTL1 |= UCTXSTT; //Generate Start Condition

	 while((UCB0CTL1 & UCTXSTT)
			 )  //if UCTXSTT != 0, wait here
	 {
		 DelayCounter ++;
		 if (DelayCounter >= DELAY_LIMIT)
			 break;
	 }

	 if (DelayCounter >= DELAY_LIMIT || UCB0STAT & UCNACKIFG)   //check if NACK condition occurred
		 return -1;

	 for(*NumberOfReceivedBytesPointer = 0; *NumberOfReceivedBytesPointer < ExpectedByteNumber; (*NumberOfReceivedBytesPointer)++)
	 {
		 if(*NumberOfReceivedBytesPointer + 1 == ExpectedByteNumber)
			 UCB0CTL1 |= UCTXSTP;

		 DelayCounter = 0;

		 while(DelayCounter < DELAY_LIMIT && !(IFG2 & UCB0RXIFG))
		 {
			 DelayCounter++;
		 }

		 if(DelayCounter == DELAY_LIMIT)
		 {
			 UCB0CTL1 |= UCSWRST;   //if I2C overtime condition occurred, reset I2C engine
			 return -1;
		 }

		 *DataPointer = UCB0RXBUF;

		 DataPointer++;
	 }

	 DelayCounter = 0;
	 while(DelayCounter < DELAY_LIMIT && (UCB0CTL1 & UCTXSTP))
	 {
		 DelayCounter++;
	 }

	 if(DelayCounter >= DELAY_LIMIT)
	 {
		 UCB0CTL1 |= UCSWRST;
		 return -1;
	 }

	 return 0;

}

int I2CReadRegisterByte(unsigned char I2CSlaveAddress, unsigned char Register, unsigned char *Data)
{
	unsigned char TargetRegister = Register;
	unsigned int SentByte = 0;
	unsigned int ReadDataCount = 0;
    int ReadStatus = 0;
    int WriteStatus = 0;

	WriteStatus = I2CSendBytes(I2CSlaveAddress, &TargetRegister, 1, &SentByte);

	ReadStatus = I2CReadBytes(I2CSlaveAddress, Data, 1, &ReadDataCount);

	if (ReadStatus != 0 || WriteStatus != 0)
	{
		return -1;
	}

	return 0;
}

int I2CReadBlock(unsigned char I2CSlaveAddress, unsigned char StartRegisterAddress, unsigned char *Buffer, unsigned int BlockSize, unsigned int *NumberOfBytes)
{
	unsigned char TargetRegister = StartRegisterAddress;
	unsigned int SentByte = 0;
	int ReadStatus = 0;
	int WriteStatus = 0;

	WriteStatus = I2CSendBytes(I2CSlaveAddress, &TargetRegister, 1, &SentByte);

	ReadStatus = I2CReadBytes(I2CSlaveAddress, Buffer, BlockSize, NumberOfBytes);

	if(ReadStatus != 0 || WriteStatus != 0)
	{
		return -1;
	}

	return 0;
}

unsigned char CRC8(unsigned char *ptr, unsigned char len,unsigned char key)
{
	unsigned char i;
	unsigned char crc=0;
	while(len--!=0)
	{
		for(i=0x80; i!=0; i/=2)
		{
			if((crc & 0x80) != 0)
			{
				crc *= 2;
				crc ^= key;
			}
			else
				crc *= 2;

			if((*ptr & i)!=0)
				crc ^= key;
		}
		ptr++;
	}
	return(crc);
}

int I2CReadRegisterByteWithCRC(unsigned char I2CSlaveAddress, unsigned char Register, unsigned char *Data)
{
	unsigned char TargetRegister = Register;
	unsigned int SentByte = 0;
	unsigned char ReadData[2];
	unsigned int ReadDataCount = 0;
	unsigned char CRCInput[2];
	unsigned char CRC = 0;
    int ReadStatus = 0;
    int WriteStatus = 0;

	WriteStatus = I2CSendBytes(I2CSlaveAddress, &TargetRegister, 1, &SentByte);

	ReadStatus = I2CReadBytes(I2CSlaveAddress, ReadData, 2, &ReadDataCount);

	if (ReadStatus != 0 || WriteStatus != 0)
	{
		return -1;
	}

	CRCInput[0] = (I2CSlaveAddress << 1) + 1;
	CRCInput[1] = ReadData[0];

	CRC = CRC8(CRCInput, 2, CRC_KEY);

	if (CRC != ReadData[1])
		return -1;

	*Data = ReadData[0];
	return 0;
}

int I2CReadRegisterWordWithCRC(unsigned char I2CSlaveAddress, unsigned char Register, unsigned int *Data)
{
	unsigned char TargetRegister = Register;
	unsigned int SentByte = 0;
	unsigned char ReadData[4];
	unsigned int ReadDataCount = 0;
	unsigned char CRCInput[2];
	unsigned char CRC = 0;
    int ReadStatus = 0;
    int WriteStatus = 0;

	WriteStatus = I2CSendBytes(I2CSlaveAddress, &TargetRegister, 1, &SentByte);

	ReadStatus = I2CReadBytes(I2CSlaveAddress, ReadData, 4, &ReadDataCount);

	if (ReadStatus != 0 || WriteStatus != 0)
	{
		return -1;
	}

	CRCInput[0] = (I2CSlaveAddress << 1) + 1;
	CRCInput[1] = ReadData[0];

	CRC = CRC8(CRCInput, 2, CRC_KEY);

	if (CRC != ReadData[1])
		return -1;

	CRC = CRC8(ReadData + 2, 1, CRC_KEY);

	if (CRC != ReadData[3])
		return -1;

	*Data = ReadData[0];

	*Data = (*Data << 8) + ReadData[2];

	return 0;
}

int I2CReadBlockWithCRC(unsigned char I2CSlaveAddress, unsigned char Register, unsigned char *Buffer, unsigned char Length)
{
	unsigned char TargetRegister = Register;
	unsigned int SentByte = 0;
	unsigned char *ReadData = NULL, *StartData = NULL;
	unsigned int ReadDataCount = 0;
	unsigned char CRCInput[2];
	unsigned char CRC = 0;
    int ReadStatus = 0;
    int WriteStatus = 0;
    int i;

    StartData = (unsigned char *)malloc(2 * Length);

    if (NULL == StartData)
    	return -1;

    ReadData = StartData;

	WriteStatus = I2CSendBytes(I2CSlaveAddress, &TargetRegister, 1, &SentByte);

	ReadStatus = I2CReadBytes(I2CSlaveAddress, ReadData, 2 * Length, &ReadDataCount);

	if (ReadStatus != 0 || WriteStatus != 0)
	{
		free(StartData);
		StartData = NULL;

		return -1;
	}

	CRCInput[0] = (I2CSlaveAddress << 1) + 1;
	CRCInput[1] = *ReadData;

	CRC = CRC8(CRCInput, 2, CRC_KEY);

	ReadData++;
	if (CRC != *ReadData)
	{
		free(StartData);
		StartData = NULL;
		return -1;
	}
	else
		*Buffer = *(ReadData - 1);

	for(i = 1; i < Length; i++)
	{
		ReadData++;
		CRC = CRC8(ReadData, 1, CRC_KEY);
		ReadData++;
		Buffer++;

		if (CRC != *ReadData)
		{
			free(StartData);
			StartData = NULL;

			return -1;
		}
		else
			*Buffer = *(ReadData - 1);
	}

	free(StartData);
	StartData = NULL;

	return 0;
}

int GetADCGainOffset()
{
	int result;

	result = I2CReadRegisterByteWithCRC(BQMAXIMO, ADCGAIN1, &(Registers.ADCGain1.ADCGain1Byte));
	result = I2CReadRegisterByteWithCRC(BQMAXIMO, ADCGAIN2, &(Registers.ADCGain2.ADCGain2Byte));
	result = I2CReadRegisterByteWithCRC(BQMAXIMO, ADCOFFSET, &(Registers.ADCOffset));

	return result;
}

int BqIsAlert(void)
{
    if((P1IN&BIT0) == BIT0)
    {
        return 1;
    }

	return 0;
}

int ConfigureBqMaximo()
{
	int result = 0;
	unsigned char bqMaximoProtectionConfig[7];

	result = I2CWriteBlockWithCRC(BQMAXIMO, CELLBAL1, &(Registers.CellBal1.CellBal1Byte), 2);

	result = I2CWriteBlockWithCRC(BQMAXIMO, SYS_CTRL1, &(Registers.SysCtrl1.SysCtrl1Byte), 7);

	result = I2CReadBlockWithCRC(BQMAXIMO, SYS_CTRL1, bqMaximoProtectionConfig, 7);

	if(bqMaximoProtectionConfig[0] != Registers.SysCtrl1.SysCtrl1Byte
			|| bqMaximoProtectionConfig[1] != Registers.SysCtrl2.SysCtrl2Byte
			|| bqMaximoProtectionConfig[2] != Registers.Protect1.Protect1Byte
			|| bqMaximoProtectionConfig[3] != Registers.Protect2.Protect2Byte
			|| bqMaximoProtectionConfig[4] != Registers.Protect3.Protect3Byte
			|| bqMaximoProtectionConfig[5] != Registers.OVTrip
			|| bqMaximoProtectionConfig[6] != Registers.UVTrip)
	{
		result = -1;
	}

	return result;
}

int InitialisebqMaximo()
{
	int result = 0, count = 0;
	//From TIDA-00449 drv_bq76930.c BqInitialisebqMaximo()
	//BQ76930 need 400ms from SHIP-to-NORMAL transition to first read operation, datasheet, page 21.
    //ALERT will be set in approx. 990ms after SHIP-to-NORMAL transition.
    //Stay here for the alert to be set.
    while(1)
    {
        delay_1ms(1);
        count++;
        if(BqIsAlert())
        {
            break;
        }
        else
        {
            if(count >= 1000)
            {
                //MCU is powered but in 1s ALERT is not set....error
                break;
            }
        }
    }
    //dummy write command to BQ
	I2CWriteRegisterByteWithCRC(BQMAXIMO, SYS_STAT, 0xff); 

    //set CC_CFG to recommended value of 0x19
	I2CWriteRegisterByteWithCRC(BQMAXIMO, CC_CFG, 0x19); // set this first before everything else?

	Registers.Protect1.Protect1Bit.SCD_DELAY = SCDDelay;
	Registers.Protect1.Protect1Bit.SCD_THRESH = SCDThresh;
	Registers.Protect2.Protect2Bit.OCD_DELAY = OCDDelay;
	Registers.Protect2.Protect2Bit.OCD_THRESH = OCDThresh;
	Registers.Protect3.Protect3Bit.OV_DELAY = OVDelay;
	Registers.Protect3.Protect3Bit.UV_DELAY = UVDelay;
	
	// OV and UV threshold
	result = GetADCGainOffset();

	Gain = (365 + ((Registers.ADCGain1.ADCGain1Byte & 0x0C) << 1) + ((Registers.ADCGain2.ADCGain2Byte & 0xE0)>> 5)) / 1000.0;
	iGain = 365 + ((Registers.ADCGain1.ADCGain1Byte & 0x0C) << 1) + ((Registers.ADCGain2.ADCGain2Byte & 0xE0)>> 5);

    Registers.OVTrip = (unsigned char)((((unsigned short)((OVPThreshold - Registers.ADCOffset)/Gain + 0.5) - OV_THRESH_BASE) >> 4) & 0xFF);
    Registers.UVTrip = (unsigned char)((((unsigned short)((UVPThreshold - Registers.ADCOffset)/Gain + 0.5) - UV_THRESH_BASE) >> 4) & 0xFF);

    //setup system controls
	// ADC -> Cell Voltage
    Registers.SysCtrl1.SysCtrl1Bit.ADC_EN = 1;
    Registers.SysCtrl1.SysCtrl1Bit.LOAD_PRESENT = 0;                 // read-only bit
    Registers.SysCtrl1.SysCtrl1Bit.SHUT_A = 0;                       // do not go to SHIP mode
    Registers.SysCtrl1.SysCtrl1Bit.SHUT_B = 0;                       // do not go to SHIP mode
    Registers.SysCtrl1.SysCtrl1Bit.TEMP_SEL = 1;                     // 0 = die temp; 1 = thermistor temp
	
	// Coulomb Counter -> Current Sense 
    Registers.SysCtrl2.SysCtrl2Bit.CC_EN = 1;           // 0 = disable continuous reading; 1= enable
    Registers.SysCtrl2.SysCtrl2Bit.CC_ONESHOT = 0;      // 0 = no action; 1 = trigger one shot reading
    Registers.SysCtrl2.SysCtrl2Bit.DELAY_DIS = 0;       // 0 = normal delay; 1 = no delay (250ms)
    Registers.SysCtrl2.SysCtrl2Bit.CHG_ON = 1;          // 0 = CHG FET off; 1 = CHG FET on
    Registers.SysCtrl2.SysCtrl2Bit.DSG_ON = 1;          // 0 = DSG FET off; 1 = DSG FET on
    Registers.SysCtrl2.SysCtrl2Bit.WAKE_EN = 0;         // Not present in BQ76930
    Registers.SysCtrl2.SysCtrl2Bit.WAKE_T = 0;          // Not present in BQ76930

	// intentionally disable cell balancing 
	Registers.CellBal1.CellBal1Bit.CB1 = 0;
	Registers.CellBal1.CellBal1Bit.CB2 = 0;
	Registers.CellBal1.CellBal1Bit.CB3 = 0;
	Registers.CellBal1.CellBal1Bit.CB4 = 0;
	Registers.CellBal1.CellBal1Bit.CB5 = 0;

	Registers.CellBal2.CellBal2Bit.CB6 = 0;
	Registers.CellBal2.CellBal2Bit.CB7 = 0;
	Registers.CellBal2.CellBal2Bit.CB8 = 0;
	Registers.CellBal2.CellBal2Bit.CB9 = 0;
    Registers.CellBal2.CellBal2Bit.CB10 = 0;

	result = ConfigureBqMaximo();

    return result;
}

int UpdateVoltageFromBqMaximo()
{
	int Result = 0, i = 0;
	unsigned char *pRawADCData = NULL;
	unsigned int iTemp = 0;
	unsigned long lTemp = 0;

	Result = I2CReadBlockWithCRC(BQMAXIMO, \
			VC1_HI_BYTE, \
			&(Registers.VCell1.VCell1Byte.VC1_HI), \
			18); // 9 cells x 2 bytes = 18 bytes 

	pRawADCData = &Registers.VCell1.VCell1Byte.VC1_HI;
	for (i = 0; i < 9; i++) // 9 cells
	{
		iTemp = (unsigned int)(*pRawADCData << 8) + *(pRawADCData + 1);
		lTemp = ((unsigned long)iTemp * iGain)/1000;
		lTemp += Registers.ADCOffset;
		CellVoltage[i] = lTemp;
		pRawADCData += 2;
	}

	return Result;
}

void main(void)
{
	int Result;

    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer

    DISABLE_INT;

    ClockInitialise();

    I2CInitialise();

    InitialisebqMaximo();

    while(1)
    {
    	Result = UpdateVoltageFromBqMaximo();
		Result = I2CReadBlockWithCRC(BQMAXIMO,SYS_STAT,&(Registers.SysStatus.StatusByte),1);
    }

	return;
}
