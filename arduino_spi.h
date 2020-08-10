#ifndef HopeDuino_SPI_h
#define HopeDuino_SPI_h

#include <arduino.h>

//Dirver hardware I/O define
#define MISO			12	
#define MOSI			11
#define SCK			    13	
#define nCS			    10

#define SOFT_SPI_nSS_DIRSET()      pinMode(nCS,OUTPUT)
#define nCS_HIGH()				   digitalWrite(nCS,HIGH)
#define nCS_LOW()				   digitalWrite(nCS,LOW)
	
#define SOFT_SPI_MISO_DIRSET()     pinMode(MISO,INPUT_PULLUP)
#define SOFT_SPI_MISO_READ()       digitalRead(MISO)

#define SOFT_SPI_MOSI_DIRSET()     pinMode(MOSI,OUTPUT)
#define SOFT_SPI_MOSI_HI()         digitalWrite(MOSI,HIGH)
#define SOFT_SPI_MOSI_LO()         digitalWrite(MOSI,LOW)

#define SOFT_SPI_SCK_DIRSET()      pinMode(SCK,OUTPUT)
#define SOFT_SPI_SCK_HI()          digitalWrite(SCK,HIGH)
#define SOFT_SPI_SCK_LO()          digitalWrite(SCK,LOW)

void vSpiInit(void);				/** initialize hardware SPI config, SPI_CLK = Fcpu/4 **/	
void vSpiWrite(word dat);			/** SPI send one word **/
byte bSpiRead(byte addr);			/** SPI read one byte **/
void vSpiBurstWrite(byte addr, byte ptr[], byte length);	/** SPI burst send N byte **/
void vSpiBurstRead(byte addr, byte ptr[], byte length);	 	/** SPI burst rend N byte **/
byte bSpiTransfer(byte dat);		/**	SPI send/read one byte **/

#endif
