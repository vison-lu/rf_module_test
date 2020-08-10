#include "rfm26_driver.h"

#define MODE_PIN			2

byte mode = 0;             //0: default receiver mode	, 1: transmitter mode
byte tx_buf[64]={"HopeRF RFM COBRFM26-S"};
byte rx_buf[64];

void setup() 
{
  //system init,
  pinMode(MODE_PIN,INPUT_PULLUP);

  Serial.begin(115200);

  //radio configuration
  RFM26_Config();

	 //determine as receiver or transmitter
	if(digitalRead(MODE_PIN)==0){
    mode = 1;
    RFM26_EntryTx();
  }
  else{
    mode = 0;
    RFM26_EntryRx();    
  }
}

void loop() {
  byte num;
  static unsigned int cnt=0;
  static unsigned int cnt_tx=0;

  if (mode) {
    send_message(tx_buf,21);
    Serial.println("");
    Serial.print(cnt_tx++);
  	Serial.print(" packet had sended");
    delay(2000);
  } else {
    num = receive_message((uint8_t*)rx_buf);
    if (num>0) {
      Serial.println("");
      Serial.print(cnt++);
      Serial.print(" packet received: ");
      Serial.write(rx_buf, 21);
    }
  }
}
