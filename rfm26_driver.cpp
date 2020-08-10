#include "rfm26_driver.h"
#include "arduino_spi.h"

/************************Description************************
                      ________________
                     /                \
                    /      HOPERF      \
                   /____________________\
                  |       DK_RFM26       |
    (MCU)DIO1 --- | GIO0             GND | --- GND
    (MCU)DIO0 --- | GIO2              A7 | --- 
  (MCU)RFData --- | GIO1              A6 | ---  
   (MCU)nIRQ1 --- | GIO3              A5 | --- 
   (MCU)nIRQ0 --- | nIRQ              A4 | --- 
    (MCU)MISO --- | MISO              A3 | --- 
    (MCU)MOSI --- | MOSI              A2 | --- 
     (MCU)SCK --- | SCK               A1 | --- 
     (MCU)nCS --- | NSS               A0 | --- 
  (MCU)PORChk --- | RESET            VCC | --- VCC             
                  |______________________|
                  
          
//  RF module:           RFM26
//  Carry Frequency:     315MHz/434MHz/868MHz/915MHz
//  Bit Rate:            2.4Kbps
//  Tx Power Output:     20dbm/17dbm/14dbm/11dbm
//  Frequency Deviation: +/-35KHz
//  Receive Bandwidth:   150KHz
//  Coding:              NRZ
//  Packet Format:       0x5555555555+0xAA2DD4+"HopeRF RFM COBRFM26-S" (total: 29 bytes)
//  Tx Current:          about 85mA  (RFOP=+20dBm,typ.)
//  Rx Current:          about 14mA  (typ.)                 
**********************************************************/

//WDS3 CONFIGURATION COMMANDS
#define RF_POWER_UP 0x02, 0x01, 0x01, 0x01, 0xC9, 0xC3, 0x80
#define RF_GPIO_PIN_CFG 0x13, 0x5C, 0x53, 0x5B, 0x51, 0x00, 0x00
#define RF_GLOBAL_XO_TUNE_1 0x11, 0x00, 0x01, 0x00, 0x3F
#define RF_GLOBAL_CONFIG_1 0x11, 0x00, 0x01, 0x03, 0x40
#define RF_INT_CTL_ENABLE_2 0x11, 0x01, 0x02, 0x00, 0x01, 0x30
#define RF_FRR_CTL_A_MODE_4 0x11, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00
#define RF_PREAMBLE_TX_LENGTH_9 0x11, 0x10, 0x09, 0x00, 0x08, 0x14, 0x00, 0x0F, 0x31, 0x00, 0x00, 0x00, 0x00
#define RF_SYNC_CONFIG_5 0x11, 0x11, 0x05, 0x00, 0x01, 0xB4, 0x2B, 0x00, 0x00
#define RF_PKT_CRC_CONFIG_1 0x11, 0x12, 0x01, 0x00, 0x80
#define RF_PKT_CONFIG1_1 0x11, 0x12, 0x01, 0x06, 0x02
#define RF_PKT_LEN_3 0x11, 0x12, 0x03, 0x08, 0x00, 0x00, 0x00
#define RF_PKT_FIELD_1_LENGTH_12_8_12 0x11, 0x12, 0x0C, 0x0D, 0x00, 0x40, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
#define RF_PKT_FIELD_4_LENGTH_12_8_8 0x11, 0x12, 0x08, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
//#define RF_MODEM_MOD_TYPE_12 0x11, 0x20, 0x0C, 0x00, 0x02, 0x00, 0x07, 0x00, 0x09, 0x60, 0x00, 0x2D, 0xC6, 0xC0, 0x00, 0x09
#define RF_MODEM_MOD_TYPE_12 0x11, 0x20, 0x0C, 0x00, 0x02, 0x00, 0x07, 0x00, 0x09, 0x60, 0x00, 0x2D, 0xC6, 0xC0, 0x00, 0x04
//#define RF_MODEM_FREQ_DEV_0_1 0x11, 0x20, 0x01, 0x0C, 0x8F
#define RF_MODEM_FREQ_DEV_0_1 0x11, 0x20, 0x01, 0x0C, 0xC7
//#define RF_MODEM_TX_RAMP_DELAY_8 0x11, 0x20, 0x08, 0x18, 0x01, 0x80, 0x08, 0x03, 0x80, 0x00, 0x12, 0x10
#define RF_MODEM_TX_RAMP_DELAY_8 0x11, 0x20, 0x08, 0x18, 0x01, 0x80, 0x08, 0x03, 0xC0, 0x00, 0x12, 0x10
#define RF_MODEM_BCR_OSR_1_9 0x11, 0x20, 0x09, 0x22, 0x04, 0x12, 0x00, 0x7D, 0xD4, 0x00, 0x3F, 0x02, 0xC2
//#define RF_MODEM_AFC_GEAR_7 0x11, 0x20, 0x07, 0x2C, 0x04, 0x36, 0x80, 0x02, 0x75, 0x0B, 0x80
#define RF_MODEM_AFC_GEAR_7 0x11, 0x20, 0x07, 0x2C, 0x04, 0x36, 0x80, 0x01, 0x75, 0x0B, 0x80
#define RF_MODEM_AGC_CONTROL_1 0x11, 0x20, 0x01, 0x35, 0xE2
#define RF_MODEM_AGC_WINDOW_SIZE_9 0x11, 0x20, 0x09, 0x38, 0x11, 0xE4, 0xE4, 0x00, 0x02, 0x74, 0xAA, 0x00, 0x2B
#define RF_MODEM_OOK_CNT1_11 0x11, 0x20, 0x0B, 0x42, 0xA4, 0x02, 0xD6, 0x81, 0x05, 0xEA, 0x01, 0x80, 0xFF, 0x0C, 0x00
#define RF_MODEM_RSSI_COMP_1 0x11, 0x20, 0x01, 0x4E, 0x40
#define RF_MODEM_CLKGEN_BAND_1 0x11, 0x20, 0x01, 0x51, 0x08
#define RF_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12 0x11, 0x21, 0x0C, 0x00, 0xFF, 0xBA, 0x0F, 0x51, 0xCF, 0xA9, 0xC9, 0xFC, 0x1B, 0x1E, 0x0F, 0x01
#define RF_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12 0x11, 0x21, 0x0C, 0x0C, 0xFC, 0xFD, 0x15, 0xFF, 0x00, 0x0F, 0xFF, 0xBA, 0x0F, 0x51, 0xCF, 0xA9
#define RF_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12 0x11, 0x21, 0x0C, 0x18, 0xC9, 0xFC, 0x1B, 0x1E, 0x0F, 0x01, 0xFC, 0xFD, 0x15, 0xFF, 0x00, 0x0F
#define RF_PA_MODE_4 0x11, 0x22, 0x04, 0x00, 0x08, 0x7F, 0x00, 0x0E
#define RF_SYNTH_PFDCP_CPFF_7 0x11, 0x23, 0x07, 0x00, 0x2C, 0x0E, 0x0B, 0x04, 0x0C, 0x73, 0x03
#define RF_MATCH_VALUE_1_12 0x11, 0x30, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
#define RF_FREQ_CONTROL_INTE_8 0x11, 0x40, 0x08, 0x00, 0x3C, 0x08, 0x00, 0x00, 0x00, 0x00, 0x20, 0xFF

#define RADIO_CONFIGURATION_DATA_ARRAY { \
        0x07, RF_POWER_UP, \
        0x07, RF_GPIO_PIN_CFG, \
        0x05, RF_GLOBAL_XO_TUNE_1, \
        0x05, RF_GLOBAL_CONFIG_1, \
        0x06, RF_INT_CTL_ENABLE_2, \
        0x08, RF_FRR_CTL_A_MODE_4, \
        0x0D, RF_PREAMBLE_TX_LENGTH_9, \
        0x09, RF_SYNC_CONFIG_5, \
        0x05, RF_PKT_CRC_CONFIG_1, \
        0x05, RF_PKT_CONFIG1_1, \
        0x07, RF_PKT_LEN_3, \
        0x10, RF_PKT_FIELD_1_LENGTH_12_8_12, \
        0x0C, RF_PKT_FIELD_4_LENGTH_12_8_8, \
        0x10, RF_MODEM_MOD_TYPE_12, \
        0x05, RF_MODEM_FREQ_DEV_0_1, \
        0x0C, RF_MODEM_TX_RAMP_DELAY_8, \
        0x0D, RF_MODEM_BCR_OSR_1_9, \
        0x0B, RF_MODEM_AFC_GEAR_7, \
        0x05, RF_MODEM_AGC_CONTROL_1, \
        0x0D, RF_MODEM_AGC_WINDOW_SIZE_9, \
        0x0F, RF_MODEM_OOK_CNT1_11, \
        0x05, RF_MODEM_RSSI_COMP_1, \
        0x05, RF_MODEM_CLKGEN_BAND_1, \
        0x10, RF_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12, \
        0x10, RF_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12, \
        0x10, RF_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12, \
        0x08, RF_PA_MODE_4, \
        0x0B, RF_SYNTH_PFDCP_CPFF_7, \
        0x10, RF_MATCH_VALUE_1_12, \
        0x0C, RF_FREQ_CONTROL_INTE_8, \
        0x00 \
} 

const uint8_t RFM26_CONFIGURATION_DATA[]=RADIO_CONFIGURATION_DATA_ARRAY;

const uint8_t RFM26FreqTbl[4][8] = {
  {0x03, 0x40, 0x00, 0x0B, 0x3E, 0x08, 0x00, 0x00},  //315MHz
  {0x03, 0x80, 0x00, 0x0A, 0x38, 0x0E, 0xEE, 0xEE},  //434MHz
  {0x03, 0xC0, 0x00, 0x08, 0x38, 0x0E, 0xEE, 0xEE},  //868MHz
  {0x03, 0xC0, 0x00, 0x08, 0x3C, 0x08, 0x00, 0x00},  //915MHz
};

const uint8_t RFM26PowerTbl[4][2] = {
  {0x7F,0x00},                            //20dbm 
  {0x30,0x00},                            //17dbm
  {0x20,0x00},                            //14dbm
  {0x16,0x00},                            //11dbm
};          

byte gb_RxData[64];           

/**********************************************************
**Variable define
**********************************************************/
//uint8_t gb_WaitStableFlag=0;                                    //State stable flag
uint8_t abApi_Write[16];                                        // Write buffer for API communication
uint8_t abApi_Read[16];                                         // Read buffer for API communication

/**********************************************************
**Name:     bSpi_SendDataNoResp
**Function: send data over SPI no response expected
**Input:    bDataInLength, length of data
            *pbDataIn, pointer to the data
**Output:   none
**********************************************************/
uint8_t bSpi_SendDataNoResp(uint16_t bDataInLength, uint8_t *pbDataIn)
{
  uint16_t bCnt;

  for (bCnt=0; bCnt<bDataInLength; bCnt++)                 // Send input data array via SPI
  {
    bSpiTransfer(pbDataIn[bCnt]);
  }
  return 0;
}

/**********************************************************
**Name:     bSpi_SendDataGetResp
**Function: send dummy data over SPI and get the response
**Input:    bDataInLength, length of data to be read
            *pbDataOut, response
**Output:   none
**********************************************************/
uint8_t bSpi_SendDataGetResp(uint8_t bDataOutLength, uint8_t *pbDataOut)  
{
  uint8_t bCnt;

  // send command and get response from the radio IC
  for (bCnt=0; bCnt<bDataOutLength; bCnt++)
  {
    pbDataOut[bCnt] = bSpiTransfer(0x00);                       // Store data uint8_t that came from the radio IC
  }
  return 0;
}

/**********************************************************
**Name:     bApi_SendCommand
**Function: send API command, no response expected
**Input:    bCmdLength , nmbr of u8s to be sent
            *pbCmdData , pointer to the commands
**Output:   none
**********************************************************/
uint8_t bApi_SendCommand(uint8_t bCmdLength, uint8_t *pbCmdData)   
{
  nCS_LOW();					
  bSpi_SendDataNoResp(bCmdLength, pbCmdData);             // Send data array to the radio IC via SPI
	nCS_HIGH();
  return 0;
}

/**********************************************************
**Name:     bApi_WaitforCTS
**Function: wait for CTS
**Input:    None
**Output:   0 , CTS arrived
            1 , CTS didn't arrive within MAX_CTS_RETRY
**********************************************************/
#define MAX_CTS_RETRY   5000
uint8_t bApi_WaitforCTS(void)
{
  uint8_t bCtsValue;
  uint16_t bErrCnt;

  bCtsValue = 0;
  bErrCnt = 0;

  while (bCtsValue!=0xFF)                                // Wait until radio IC is ready with the data
  {
	nCS_LOW();
	bSpiTransfer(0x44);                               // CMD_READ_CMD_BUFF,Read command buffer; send command uint8_t
    bSpi_SendDataGetResp(1, &bCtsValue);                 // Read command buffer; get CTS value
 	nCS_HIGH();
	if(++bErrCnt > MAX_CTS_RETRY)
    {
       return 1;                                          // Error handling; if wrong CTS reads exceeds a limit
    }
  }
  return 0;
}

/**********************************************************
**Name:     bApi_GetResponse
**Function: wait for CTS and get the read u8s from the chip
**Input:    bRespLength , nmbr of u8s to be read
            *pbRespData , pointer to the read data
**Output:   0 , operation successful
            1 , no CTS within MAX_CTS_RETRY
**********************************************************/
uint8_t bApi_GetResponse(uint8_t bRespLength, uint8_t *pbRespData) 
{
  bApi_WaitforCTS();
  bSpi_SendDataGetResp(bRespLength, pbRespData);          // CTS value ok, get the response data from the radio IC
  nCS_HIGH();
  return 0;
}

/**********************************************************
**Name:     bApi_ReadRxDataBuffer
**Function: Read RX FIFO content from the chip
**Input:    bRxFifoLength , one of the fast response registers
            *pbRxFifoData , pointer to the read data
**Output:   0 , operation successful
**********************************************************/
uint8_t bApi_ReadRxDataBuffer(uint8_t bRxFifoLength, uint8_t *pbRxFifoData)
{
  nCS_LOW();
  bSpiTransfer(0x77);                                  // CMD_READ_RX_FIFO,Send Rx read command
  bSpi_SendDataGetResp(bRxFifoLength, pbRxFifoData);      // Write Tx FIFO
  nCS_HIGH();
  return 0;
}

/**********************************************************
**Name:     bApi_WriteTxDataBuffer
**Function: wait for CTS and get the read bytes from the chip
**Input:    bTxFifoLength , nmbr of u8s to be sent
            *pbTxFifoData , pointer to the transmit data
**Output:   0 , operation successful
            1 , no CTS within MAX_CTS_RETRY
**********************************************************/
uint8_t bApi_WriteTxDataBuffer(uint8_t bTxFifoLength, uint8_t *pbTxFifoData) 
{
 nCS_LOW();
  bSpiTransfer(0x66);                                  // CMD_WRITE_TX_FIFO,Send Tx write command
  bSpi_SendDataNoResp(bTxFifoLength, pbTxFifoData);       // Write Tx FIFO
 nCS_HIGH();
  return 0;
}

/**********************************************************
**Name:     RFM26_ClrAllInterrupt
**Function: Read ITs, clear pending ones
**Input:    None
**Output:   None
**********************************************************/
void RFM26_ClrAllInterrupt(void)
{ 
  abApi_Write[0] = 0x20;                                  // CMD_GET_INT_STATUS,Use interrupt status command
  abApi_Write[1] = 0;                                     // Clear PH_CLR_PEND
  abApi_Write[2] = 0;                                     // Clear MODEM_CLR_PEND
  abApi_Write[3] = 0;                                     // Clear CHIP_CLR_PEND
  bApi_SendCommand(4,abApi_Write);                        // Send command to the radio IC
  bApi_GetResponse(8, abApi_Read );                       // Make sure that CTS is ready then get the response
}

/**********************************************************
**Name:     RFM26_IntoSleep
**Function: Put the Radio into SLEEP state
**Input:    None
**Output:   None
**********************************************************/
void RFM26_IntoSleep(void)            
{
  abApi_Write[0] = 0x34;                                  // CMD_CHANGE_STATE,Change state command
  abApi_Write[1] = 0x01;                                  // SLEEP state
  bApi_SendCommand(2,abApi_Write);                        // Send command to the radio IC
}

/**********************************************************
**Name:     RFM26_WakeUp
**Function: Wake up the radio from SLEEP mode
**Input:    None
**Output:   None
**********************************************************/
void RFM26_WakeUp(void)               
{
  abApi_Write[0] = 0x34;                                  // CMD_CHANGE_STATE,Change state command
  abApi_Write[1] = 0x02;                                  // SPI active state
  bApi_SendCommand(2,abApi_Write);                        // Send command to the radio IC
  bApi_WaitforCTS();
}

/**********************************************************
**Name:     RFM26_StartRadio
**Function: start radio
**Input:    None
**Output:   None
**********************************************************/
void RFM26_StartRadio(void)
{ 
  RESET_HIGH();                           
  delay_us(300);											//about 300us
  RESET_LOW();
  delay_ms(5);
  
  // Start the radio
  abApi_Write[0] = 0x02;                                  // CMD_POWER_UP,Use API command to power up the radio IC
  abApi_Write[1] = 0x01;                                  // Write global control registers
  abApi_Write[2] = 0x00;                                  // Write global control registers
  bApi_SendCommand(3,abApi_Write);                        // Send command to the radio IC
  // Wait for boot
  bApi_WaitforCTS();                                       // Wait for CTS
  
  RFM26_ClrAllInterrupt();
}

/**********************************************************
**Name:     ParameterConfig
**Function: Parameter config
**Input:    * configurationTable,Parameter table
**Output:   None
**********************************************************/
void ParameterConfig(uint8_t *configurationTable)
{
  uint16_t Count=0;
  while(configurationTable[Count]!=0)
  {
    bApi_SendCommand(configurationTable[Count],&configurationTable[Count+1]);
    bApi_WaitforCTS();
    Count+=(configurationTable[Count]+1);
  }
}

/**********************************************************
**Name:     RFM26_SetParameter_Freq
**Function: Set frequency
**Input:    *FreqConfig,Frequency table
**Output:   None
**********************************************************/
void RFM26_SetParameter_Freq(uint8_t *FreqConfig)
{ 
  abApi_Write[0] = 0x11;                                  // CMD_SET_PROPERTY
  abApi_Write[1] = 0x20;                                  // PROP_MODEM_GROUP
  abApi_Write[2] = 3;
  abApi_Write[3] = 0x1B;                                  // MODEM_IF_FREQ
  abApi_Write[4] = FreqConfig[0];
  abApi_Write[5] = FreqConfig[1];
  abApi_Write[6] = FreqConfig[2];
  bApi_SendCommand(7,abApi_Write);
  bApi_WaitforCTS();
  
  abApi_Write[0] = 0x11;                                  // CMD_SET_PROPERTY
  abApi_Write[1] = 0x20;                                  // PROP_MODEM_GROUP
  abApi_Write[2] = 1;
  abApi_Write[3] = 0x51;                                  // RF_MODEM_CLKGEN_BAND_1
  abApi_Write[4] = FreqConfig[3];
  bApi_SendCommand(5,abApi_Write);
  bApi_WaitforCTS();
  
  abApi_Write[0] = 0x11;                                  // CMD_SET_PROPERTY
  abApi_Write[1] = 0x40;                                  // PROP_FREQ_CONTROL_GROUP
  abApi_Write[2] = 4;
  abApi_Write[3] = 0x00;    
  abApi_Write[4] = FreqConfig[4];
  abApi_Write[5] = FreqConfig[5];
  abApi_Write[6] = FreqConfig[6];
  abApi_Write[7] = FreqConfig[7];
  bApi_SendCommand(8,abApi_Write);
  bApi_WaitforCTS();
}

/**********************************************************
**Name:     RFM26_SetParameter_Power
**Function: Set out power
**Input:    *PAConfig,Power table
**Output:   None
**********************************************************/
void RFM26_SetParameter_Power(uint8_t *PA)
{
  abApi_Write[0] = 0x11;                                  // CMD_SET_PROPERTY
  abApi_Write[1] = 0x22;                                  // PROP_PA_GROUP
  abApi_Write[2] = 2;
  abApi_Write[3] = 0x01;                                  // PROP_PA_PWR_LVL
  abApi_Write[4] = PA[0];
  abApi_Write[5] = PA[1];
  bApi_SendCommand(6,abApi_Write);
  bApi_WaitforCTS();
} 
/**********************************************************
**Name:     RFM26_Start_Tx
**Function: Start Tx
**Input:    Channel,Channel number
            Condition,Transmit condition
            Tx_Length, Data length
**Output:   None
**********************************************************/
void RFM26_Start_Tx(uint8_t Channel, uint8_t Condition, uint16_t Tx_Length)
{   
  abApi_Write[0] = 0x31;                                  // CMD_START_TX,Use Tx Start command    
  abApi_Write[1] = Channel;                               // Channel number to transmit the packet on 
  abApi_Write[2] = Condition;                             // Set conditions           
  abApi_Write[3] = (uint8_t)((Tx_Length & 0xFF00)>>8);         // Upper byte of Tx length
  abApi_Write[4] = (uint8_t)(Tx_Length & 0x00FF);              // Lower byte of Tx length
  bApi_SendCommand(5,abApi_Write);                        // Send command to the radio IC
  bApi_WaitforCTS();                                      // Wait for CTS 
}

/**********************************************************
**Name:     RFM26_Start_Rx
**Function: Start Rx
**Input:    Channel,Channel number
            Condition,Receiver condition
            Rx_Length, Data length
            State1,RXTIMEOUT_STATE
            State2,RXTIMEOUT_STATE
            State3,RXINVALID_STATE
**Output:   None
**********************************************************/
void RFM26_Start_Rx(uint8_t Channel, uint8_t Condition, uint16_t Rx_Length, uint8_t State1, uint8_t State2, uint8_t State3)
{                               
  abApi_Write[0] = 0x32;                                  // CMD_START_RX,Use start Rx command  
  abApi_Write[1] = Channel;                               // Channel number to transmit the packet on 
  abApi_Write[2] = Condition;                             // Set conditions       
  abApi_Write[3] = (uint8_t)((Rx_Length & 0xFF00)>>8);         // Upper byte of Rx length
  abApi_Write[4] = (uint8_t)(Rx_Length & 0x00FF);              // Lower byte of Rx length 
  abApi_Write[5] = State1;                                // Next state when Preamble Timeout occurs
  abApi_Write[6] = State2;                                // Next state when a valid packet received
  abApi_Write[7] = State3;                                // Next state when invalid packet received (e.g. CRC error).
  bApi_SendCommand(8,abApi_Write);                        // Send API command to the radio IC   
  bApi_WaitforCTS();                                      // Wait for CTS 
}

/**********************************************************
**Name:     RFM26_SetINT_CTL
**Function: Set interrupt enable bit
**Input:    status,INT_CTL
            ctl_PH,INT_CTL_PH
            ctl_modem, INT_CTL_MODEM
            ctl_chip,INT_CTL_CHIP_EN
**Output:   None
**********************************************************/
void RFM26_SetINT_CTL(uint8_t status, uint8_t ctl_PH, uint8_t ctl_modem, uint8_t ctl_chip)
{ 
  abApi_Write[0] = 0x11;                                  // CMD_SET_PROPERTY,Use property command
  abApi_Write[1] = 0x01;                                  // PROP_INT_CTL_GROUP,Select property group
  abApi_Write[2] = 4;                                     // Number of properties to be written
  abApi_Write[3] = 0x00;                                  // PROP_INT_CTL_ENABLE,Specify property
  abApi_Write[4] = status;                                // INT_CTL
  abApi_Write[5] = ctl_PH;                                // INT_CTL_PH
  abApi_Write[6] = ctl_modem;                             // INT_CTL_MODEM
  abApi_Write[7] = ctl_chip;                              // INT_CTL_CHIP_EN
  bApi_SendCommand(8,abApi_Write);                        // Send API command       
  bApi_WaitforCTS();                                      // Wait for CTS
}

/**********************************************************
**Name:     RFM26_ChangeToRxMode
**Function: Change to rx mode
**Input:    length,data length
**Output:   None
**********************************************************/
void RFM26_ChangeToRxMode(uint8_t length)
{   
  RFM26_SetINT_CTL(0x01, 0x10, 0x00, 0x00);             // INT_CTL_PH: PACKET_RX  enabled
  RFM26_ClrAllInterrupt();                              // clear interrupt
  RFM26_Start_Rx(0, 0, length, 0, 0x03, 0x03);          // Start Rx
}

/**********************************************************
**Name:     RFM26_ResetTxFifo
**Function: Reset Tx fifo
**Input:    None
**Output:   None
**********************************************************/
void RFM26_ResetTxFifo(void)
{ 
  abApi_Write[0] = 0x15;                                  // CMD_FIFO_INFO,Use FIFO INFO command
  abApi_Write[1] = 0x01;                                  // Reset Tx FIFO
  bApi_SendCommand(2,abApi_Write);                        // Send API command to the radio IC
  bApi_WaitforCTS();                                      // Wait for CTS
}

/**********************************************************
**Name:     RFM26_ResetRxFifo
**Function: Reset Rx fifo
**Input:    None
**Output:   None
**********************************************************/
void RFM26_ResetRxFifo(void)
{ 
  abApi_Write[0] = 0x15;                                  // CMD_FIFO_INFO,Use FIFO INFO command
  abApi_Write[1] = 0x02;                                  // Reset Tx FIFO
  bApi_SendCommand(2,abApi_Write);                        // Send API command to the radio IC
  bApi_WaitforCTS();                                      // Wait for CTS
}

/**********************************************************
**Name:     RFM26_Config
**Function: Initialize RFM26 & set it entry to standby mode
**Input:    none
**Output:   none
**********************************************************/
void RFM26_Config(void)
{
  //Input_DIO0();                                            
  //Input_DIO1();
  //Input_RFData();
  pinMode(nIRQ0, INPUT);
  pinMode(RESET, OUTPUT);
  digitalWrite(RESET, LOW);

  vSpiInit();

  RFM26_StartRadio();                                          // Start the radio

  ParameterConfig((uint8_t*)RFM26_CONFIGURATION_DATA);         //Set basic parameter
  RFM26_SetParameter_Freq((uint8_t*)RFM26FreqTbl[C_868MHZ]);  //Set frequency parameter  
  RFM26_SetParameter_Power((uint8_t*)RFM26PowerTbl[C_17DBM]); //Set power parameter
                   
  // Configure Fast response registers
  abApi_Write[0] = 0x11;                                   // CMD_SET_PROPERTY,Use property command
  abApi_Write[1] = 0x02;                                   // PROP_FRR_CTL_GROUP,Select property group
  abApi_Write[2] = 4;                                      // Number of properties to be written
  abApi_Write[3] = 0x00;                                   // PROP_FRR_CTL_A_MODE,Specify property (1st)
  abApi_Write[4] = 0x04;                                   // FRR A: PH IT pending
  abApi_Write[5] = 0x06;                                   // FRR B: Modem IT pending
  abApi_Write[6] = 0x0A;                                   // FRR C: Latched RSSI
  abApi_Write[7] = 0x00;                                   // FRR D: disabled
  bApi_SendCommand(8,abApi_Write);                         // Send API command to the radio IC
  bApi_WaitforCTS();                                       // Wait for CTS

  //Set packet content  
  // Set tx preamble length
  abApi_Write[0] = 0x11;                                   // CMD_SET_PROPERTY,Use property command
  abApi_Write[1] = 0x10;                                   // PROP_PREAMBLE_GROUP,Select property group
  abApi_Write[2] = 1;                                      // Number of properties to be written
  abApi_Write[3] = 0x00;                                   // PROP_PREAMBLE_TX_LENGTH,Specify property
  abApi_Write[4] = 0x08;                                   // 8 bytes Tx preamble
  bApi_SendCommand(5,abApi_Write);                         // Send command to the radio IC
  bApi_WaitforCTS();                                       // Wait for CTS

  //Set rx preamble length
  abApi_Write[0] = 0x11;                                   // CMD_SET_PROPERTY,Use property command
  abApi_Write[1] = 0x10;                                   // PROP_PREAMBLE_GROUP,Select property group
  abApi_Write[2] = 1;                                      // Number of properties to be written
  abApi_Write[3] = 0x01;                                   // PROP_PREAMBLE_CONFIG_STD_1,Specify property
  abApi_Write[4] = 8;                                      // 8 bits preamble detection threshold
  bApi_SendCommand(5,abApi_Write);                         // Send API command to the radio IC
  bApi_WaitforCTS();                                       // Wait for CTS

  // Set preamble pattern
  abApi_Write[0] = 0x11;                                   // CMD_SET_PROPERTY,Use property command
  abApi_Write[1] = 0x10;                                   // PROP_PREAMBLE_GROUP,Select property group
  abApi_Write[2] = 1;                                      // Number of properties to be written
  abApi_Write[3] = 0x04;                                   // PROP_PREAMBLE_CONFIG,Specify property
  abApi_Write[4] = 0x31;                                   // Use `1010` pattern, length defined in bytes
  bApi_SendCommand(5,abApi_Write);                         // Send API command to the radio IC
  bApi_WaitforCTS();                                       // Wait for CTS

  // Set sync uint16_t
  abApi_Write[0] = 0x11;                                   // CMD_SET_PROPERTY,Use property command
  abApi_Write[1] = 0x11;                                   // PROP_SYNC_GROUP,Select property group
  abApi_Write[2] = 3;                                      // Number of properties to be written
  abApi_Write[3] = 0x00;                                   // PROP_SYNC_CONFIG,Specify property
  abApi_Write[4] = 0x01;                                   // SYNC_CONFIG: 2 bytes sync word
  abApi_Write[5] = 0xB4;                                   // SYNC_BITS_31_24: 1st sync byte: 0x2D; NOTE: LSB transmitted first!
  abApi_Write[6] = 0x2B;                                   // SYNC_BITS_23_16: 2nd sync byte: 0xD4; NOTE: LSB transmitted first!
  bApi_SendCommand(7,abApi_Write);                         // Send command to the radio IC
  bApi_WaitforCTS();                                       // Wait for CTS

  // General packet config (set bit order)
  abApi_Write[0] = 0x11;                                   // CMD_SET_PROPERTY,Use property command
  abApi_Write[1] = 0x12;                                   // PROP_PKT_GROUP,Select property group
  abApi_Write[2] = 1;                                      // Number of properties to be written
  abApi_Write[3] = 0x06;                                   // PROP_PKT_CONFIG1,Specify property
  abApi_Write[4] = 0x00;                                   // Payload data goes MSB first
  bApi_SendCommand(5,abApi_Write);                         // Send command to the radio IC
  bApi_WaitforCTS();                                       // Wait for CTS

  // Configure the GPIOs, Select Tx state to GPIO2, Rx state to GPIO0
  abApi_Write[0] = 0x13;                                   // CMD_GPIO_PIN_CFG,Use GPIO pin configuration command
  abApi_Write[1] = 0x21;                                   // Configure GPIO0 as Rx state
  abApi_Write[2] = 20;                                     // Configure GPIO1 as Rx data
  abApi_Write[3] = 0x20;                                   // Configure GPIO2 as Tx state
  abApi_Write[4] = 17;                                     // Configure GPIO3 as Rx data CLK
  bApi_SendCommand(5,abApi_Write);                         // Send command to the radio IC
  bApi_WaitforCTS();

  // Adjust XTAL clock frequency
  abApi_Write[0] = 0x11;                                   // CMD_SET_PROPERTY,Use property command
  abApi_Write[1] = 0x00;                                   // PROP_GLOBAL_GROUP,Select property group
  abApi_Write[2] = 1;                                      // Number of properties to be written
  abApi_Write[3] = 0x00;                                   // PROP_GLOBAL_XO_TUNE,Specify property
  abApi_Write[4] = 0x5D;                                   // Set cap bank value to adjust XTAL clock frequency
  bApi_SendCommand(5,abApi_Write);                         // Send command to the radio IC
  bApi_WaitforCTS();                                       // Wait for CTS 
            
  // Reset Tx/Rx FIFO
  abApi_Write[0] = 0x15;                                   // CMD_FIFO_INFO,Use FIFO INFO command
  abApi_Write[1] = 0x03;                                   // Reset Tx/Rx FIFO
  bApi_SendCommand(2,abApi_Write);                         // Send API command to the radio IC
  bApi_WaitforCTS();                                       // Wait for CTS
  
  RFM26_ClrAllInterrupt();                                 // clear interrupt
}

/**********************************************************
**Name:     RFM26_EntryRx
**Function: Set RFM26 entry Rx_mode
**Input:    None
**Output:   None
**********************************************************/
void RFM26_EntryRx(void)
{
  RFM26_Config();                                         // config RFM26 base parameters
  RFM26_SetINT_CTL(0x01, 0x10, 0x00, 0x00);               // INT_CTL_PH: PACKET_RX  enabled
  RFM26_ClrAllInterrupt();                                // clear interrupt

  RFM26_ResetRxFifo();                                    // Reset Rx FIFO
  RFM26_Start_Rx(0, 0, 21, 0, 0x03, 0x03);                // Start Rx                             
}

/**********************************************************
**Name:     RFM26_EntryTx
**Function: Set RFM26 entry Tx_mode
**Input:    None
**Output:   None
**********************************************************/
void RFM26_EntryTx(void)
{
  RFM26_Config(); 
  RFM26_SetINT_CTL(0x01, 0x20, 0x00, 0x00);               // INT_CTL_PH:  PACKET_SENT ITs enable  
  RFM26_ClrAllInterrupt();
  RFM26_ResetTxFifo();                                    // Reset Tx FIFO
}
/**********************************************************
**Name:     RFM26_ClearFIFO
**Function: Change to RxMode from StandbyMode, can clear FIFO buffer
**Input:    None
**Output:   None
**********************************************************/
void RFM26_ClearFIFO(void)
{
  RFM26_ResetRxFifo();                                    // Reset Rx FIFO
}

/**********************************************************
**Name:     RFM26_Sleep
**Function: Set RFM26 to sleep mode 
**Input:    none
**Output:   none
**********************************************************/
void RFM26_Sleep(void)
{
  abApi_Write[0] = 0x34;                                  // CMD_CHANGE_STATE,Change state command
  abApi_Write[1] = 0x01;                                  // SLEEP state
  bApi_SendCommand(2,abApi_Write);                        // Send command to the radio IC
}

/**********************************************************
**Name:     RFM26_Standby
**Function: Set RFM26 to Standby mode
**Input:    none
**Output:   none
**********************************************************/
void RFM26_Standby(void)
{
  abApi_Write[0] = 0x34;                                  // CMD_CHANGE_STATE,Change state command
  abApi_Write[1] = 0x01;                                  // Ready state
  bApi_SendCommand(2,abApi_Write);                        // Send command to the radio IC
}

/**********************************************************
**Name:     RFM26_ReadRSSI
**Function: Read the RSSI value
**Input:    none
**Output:   temp, RSSI value
**********************************************************/
uint8_t RFM26_ReadRSSI(void)
{
  uint16_t temp=0xff;
  
//  abApi_Write[0] = 0x22;                                  // CMD_GET_MODEM_STATUS
//  abApi_Write[1] = 0x00;                                  // don't clear PEND
//  bApi_SendCommand(2,abApi_Write);                        // Send command to the radio IC
//  bApi_WaitforCTS();
//  bApi_GetResponse(8, abApi_Read );                       // Make sure that CTS is ready then get the response
//  temp=abApi_Read[3];
  
  return (uint8_t)temp;
}


/**********************************************************
**Parameter table define in test mode
**********************************************************/


/**********************************************************
**Name:     RFM26_EntryTestRx
**Function: Set RFM26 entry Rx test mode
**Input:    None
**Output:   None
**********************************************************/
void RFM26_EntryTestRx(void)
{
  RFM26_Config();                                          //Module parameter setting  
  RFM26_ClrAllInterrupt();                                 // clear interrupt
  
  // Start Rx    
  abApi_Write[0] = 0x32;                                   // CMD_START_RX,Use start Rx command  
  abApi_Write[1] = 0;                                      // Channel number to transmit the packet on 
  abApi_Write[2] = 0;                                      // Set conditions       
  abApi_Write[3] = 0;                                      // Upper byte of Rx length
  abApi_Write[4] = 21;                                     // Lower byte of Rx length 
  abApi_Write[5] = 0;                                      // Next state when Preamble Timeout occurs
  abApi_Write[6] = 0;                                      // Next state when a valid packet received
  abApi_Write[7] = 0;                                      // Next state when invalid packet received (e.g. CRC error).
  bApi_SendCommand(8,abApi_Write);                         // Send API command to the radio IC   
  bApi_WaitforCTS();                                       // Wait for CTS            
}

/**********************************************************
**Name:     RFM26_CarrierTest
**Function: Set RFM26 entry CW mode
**Input:    None
**Output:   None
**********************************************************/
void RFM26_CarrierTest(void)                    
{ 
  RFM26_ClrAllInterrupt();

  // Set CW mode
  abApi_Write[0] = 0x11;                                  // CMD_SET_PROPERTY,Use property command
  abApi_Write[1] = 0x20;                                  // PROP_MODEM_GROUP,Select property group
  abApi_Write[2] = 1;                                     // Number of properties to be written
  abApi_Write[3] = 0x00;                                  // PROP_MODEM_MOD_TYPE,Specify property
  abApi_Write[4] = 0x00;        
  bApi_SendCommand(5,abApi_Write);                        // Send command to the radio IC
  bApi_WaitforCTS();                                      // Wait for CTS
                  
  // Start Tx
  abApi_Write[0] = 0x31;                                  // CMD_START_TX,Use Tx Start command    
  abApi_Write[1] = 0x00;                                  //  
  abApi_Write[2] = 0x00;                                  // Sleep state after Tx, start Tx immediately             
  abApi_Write[3] = 0;                                     // Upper byte of Tx length
  abApi_Write[4] = 0;                                     // Lower byte of Tx length
  bApi_SendCommand(5,abApi_Write);                        // Send command to the radio IC
  bApi_WaitforCTS();                                      // Wait for CTS 
}

/**********************************************************
**Name:     RFM26_EntryTestTx
**Function: Set RFM26 entry Tx test mode
**Input:    None
**Output:   None
**********************************************************/
void RFM26_EntryTestTx(void)
{
  RFM26_Config();                                          //Module parameter setting
  RFM26_CarrierTest();                                     //Define to carrier mode
}

/**********************************************************
**Name:     RFM26_TestRx
**Function: RFM26 Rx test mode
**Input:    None
**Output:   "0" for Error Status
**********************************************************/
void RFM26_TestRx(void)
{

}

/**********************************************************
**Name:     RFM26_TestTx
**Function: RFM26 Tx test mode
**Input:    None
**Output:   "0" for Error Status
**********************************************************/
void RFM26_TestTx(void)
{ 

}

uint8_t receive_message(uint8_t* p_data)
{
  unsigned char  i,num;

  num = 21;
  if (!nIRQ0_READ()) {
  	
    for(i=0;i<num;i++) 
      gb_RxData[i] = 0x00;
    bApi_ReadRxDataBuffer(num,gb_RxData);
    RFM26_ClearFIFO();
    RFM26_ClrAllInterrupt();
    RFM26_Start_Rx(0, 0, num, 0, 0x03, 0x03);

    for (i = 0; i < num; i++) {
      p_data[i] = gb_RxData[i];
    }
	  return num;
  }
    return 0;
}

void send_message(uint8_t* p_data,uint8_t num)
{
	RFM26_Standby();
	bApi_WriteTxDataBuffer(num,(uint8_t*)p_data);				  // Write data to Tx FIFO
	bApi_WaitforCTS();	
	if(!nIRQ0_READ())											// RevB1A workaround;
	{
	  RFM26_ClrAllInterrupt();
	}
	RFM26_Start_Tx(0x00, 0x30, num);  
}
