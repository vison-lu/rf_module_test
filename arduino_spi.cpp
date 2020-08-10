#include "arduino_spi.h"
#include <SPI.h>

#define SPI_TYPE	1			//1: select hardware SPI, depond on platform 
								//0: select software GPIO simulate SPI,

/**********************************************************
**Name: 	vSpiInit
**Func: 	Init Spi Config
**Note: 	SpiClk = Fcpu/4
**********************************************************/
void vSpiInit(void)
{
//modify this function to migrate to other platform 
#if SPI_TYPE		
	//SPI_CLOCK_DIV2/4/8/16/32/64/128	default :Fcpu/4
	SPI.setClockDivider(SPI_CLOCK_DIV16);	//1Mhz spi rate
	SPI.begin();
#else
	// can be optimized into single write if port wiring allows
	SOFT_SPI_nSS_DIRSET();
	SOFT_SPI_SCK_DIRSET();
	SOFT_SPI_MOSI_DIRSET();
	SOFT_SPI_MISO_DIRSET(); // always input after POR, can be commented out

	SOFT_SPI_SCK_LO();
	SOFT_SPI_MOSI_LO();
	nCS_HIGH();
#endif
}

/**********************************************************
**Name: 	bSpiTransfer
**Func: 	Transfer One Byte by SPI
**Input:
**Output:  
**********************************************************/
//modify this function to migrate to other platform 
byte bSpiTransfer(byte dat)
{
#if SPI_TYPE
	return SPI.transfer(dat);
#else
  byte i;

  for(i = 0; i < 8; i++)
  {
	  if (dat & 0x80) 
		  SOFT_SPI_MOSI_HI();
	  else 
		  SOFT_SPI_MOSI_LO();
	  
	  delayMicroseconds(1);
	  SOFT_SPI_SCK_HI();
	  delayMicroseconds(1);
	  
	  dat <<= 1;
	   
	  if (SOFT_SPI_MISO_READ()) 
		  dat |= 0x01; // dat++
	   
	  delayMicroseconds(1);
	  SOFT_SPI_SCK_LO();
	  delayMicroseconds(1);
   } 
   return dat;
#endif
}

#if 0
/**********************************************************
**Name:	 	vSpiWrite
**Func: 	SPI Write One word
**Input: 	Write word
**Output:	none
**********************************************************/
void vSpiWrite(word dat)
{
 nCS_LOW();
 bSpiTransfer((byte)(dat>>8)|0x80);
 bSpiTransfer((byte)dat);
 nCS_HIGH();
}

/**********************************************************
**Name:	 	bSpiRead
**Func: 	SPI Read One byte
**Input: 	readout addresss
**Output:	readout byte
**********************************************************/
byte bSpiRead(byte addr)
{
 byte tmp;
 nCS_LOW();
 bSpiTransfer(addr);
 tmp = bSpiTransfer(0x00);
 nCS_HIGH();
 return(tmp);
}

/**********************************************************
**Name:	 	vSpiBurstWirte
**Func: 	burst wirte N byte
**Input: 	array length & start address & head pointer
**Output:	none
**********************************************************/
void vSpiBurstWrite(byte addr, byte ptr[], byte length)
{
 byte i;
 nCS_LOW();
 bSpiTransfer(addr|0x80);
 for(i=0; i<length; i++)
 	bSpiTransfer(*(ptr+i));
 nCS_HIGH();
}

/**********************************************************
**Name:	 	vSpiBurstRead
**Func: 	burst read N byte
**Input: 	array length & start address & head pointer
**Output:	none
**********************************************************/
void vSpiBurstRead(byte addr, byte ptr[], byte length)
{
	if(length!=0)
	{
		byte i;	
		nCS_LOW();
		bSpiTransfer(addr);
		for(i=0; i<length; i++)
		*(ptr+i) = bSpiTransfer(0x00);
		nCS_HIGH();
	}	
}
#endif
