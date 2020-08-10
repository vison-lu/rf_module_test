#ifndef HopeDuino_26_H_
#define HopeDuino_26_H_

#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

//Dirver hardware I/O define
#define DIO0			4
#define DIO2			5
#define RFData 		    6
#define nIRQ1			7
#define nIRQ0			8
#define RESET		    9

#define RESET_HIGH()	digitalWrite(RESET,HIGH)
#define RESET_LOW()		digitalWrite(RESET,LOW)

#define nIRQ0_READ()	digitalRead(nIRQ0)

#define delay_ms(x)		delay(x)
#define delay_us(x)		delayMicroseconds(x)

//Define module work mode
#define C_ModuleWorkMode_FSK     0
#define C_ModuleWorkMode_OOK     1
#define C_ModuleWorkMode_LoRa    2
#define C_ModuleWorkMode_Standby 3
#define C_ModuleWorkMode_Sleep   4

//define module frequency
#define C_315MHZ		0
#define C_434MHZ		1
#define C_868MHZ		2
#define C_915MHZ		3

//Define module rate and diavition
#define C_1_2KHZ_35KHZ		0
#define C_2_4KHZ_35KHZ		1
#define C_4_8KHZ_35KHZ		2
#define C_9_6KHZ_35KHZ		3

//Define module out power
#define C_20DBM		0
#define C_17DBM		1
#define C_14DBM		2
#define C_11DBM		3

/**********************************************************
**Name:     bSpi_SendDataNoResp
**Function: send data over SPI no response expected
**Input:    bDataInLength, length of data
            *pbDataIn, pointer to the data
**Output:   none
**********************************************************/
uint8_t bSpi_SendDataNoResp(uint16_t bDataInLength, uint8_t *pbDataIn);

/**********************************************************
**Name:     bSpi_SendDataGetResp
**Function: send dummy data over SPI and get the response
**Input:    bDataInLength, length of data to be read
            *pbDataOut, response
**Output:   none
**********************************************************/
uint8_t bSpi_SendDataGetResp(uint8_t bDataOutLength, uint8_t *pbDataOut);  

/**********************************************************
**Name:     bApi_SendCommand
**Function: send API command, no response expected
**Input:    bCmdLength , nmbr of u8s to be sent
            *pbCmdData , pointer to the commands
**Output:   none
**********************************************************/
uint8_t bApi_SendCommand(uint8_t bCmdLength, uint8_t *pbCmdData);   

/**********************************************************
**Name:     bApi_WaitforCTS
**Function: wait for CTS
**Input:    None
**Output:   0 , CTS arrived
            1 , CTS didn't arrive within MAX_CTS_RETRY
**********************************************************/
uint8_t bApi_WaitforCTS(void);

/**********************************************************
**Name:     bApi_GetResponse
**Function: wait for CTS and get the read u8s from the chip
**Input:    bRespLength , nmbr of u8s to be read
            *pbRespData , pointer to the read data
**Output:   0 , operation successful
            1 , no CTS within MAX_CTS_RETRY
**********************************************************/
uint8_t bApi_GetResponse(uint8_t bRespLength, uint8_t *pbRespData) ;

/**********************************************************
**Name:     bApi_ReadRxDataBuffer
**Function: Read RX FIFO content from the chip
**Input:    bRxFifoLength , one of the fast response registers
            *pbRxFifoData , pointer to the read data
**Output:   0 , operation successful
**********************************************************/
uint8_t bApi_ReadRxDataBuffer(uint8_t bRxFifoLength, uint8_t *pbRxFifoData);

/**********************************************************
**Name:     bApi_WriteTxDataBuffer
**Function: wait for CTS and get the read bytes from the chip
**Input:    bTxFifoLength , nmbr of u8s to be sent
            *pbTxFifoData , pointer to the transmit data
**Output:   0 , operation successful
            1 , no CTS within MAX_CTS_RETRY
**********************************************************/
uint8_t bApi_WriteTxDataBuffer(uint8_t bTxFifoLength, uint8_t *pbTxFifoData) ;

/**********************************************************
**Name:     RFM26_ClrAllInterrupt
**Function: Read ITs, clear pending ones
**Input:    None
**Output:   None
**********************************************************/
void RFM26_ClrAllInterrupt(void);

/**********************************************************
**Name:     RFM26_IntoSleep
**Function: Put the Radio into SLEEP state
**Input:    None
**Output:   None
**********************************************************/
void RFM26_IntoSleep(void) ;           

/**********************************************************
**Name:     RFM26_WakeUp
**Function: Wake up the radio from SLEEP mode
**Input:    None
**Output:   None
**********************************************************/
void RFM26_WakeUp(void);               

/**********************************************************
**Name:     RFM26_StartRadio
**Function: start radio
**Input:    None
**Output:   None
**********************************************************/
void RFM26_StartRadio(void);

/**********************************************************
**Name:     ParameterConfig
**Function: Parameter config
**Input:    * configurationTable,Parameter table
**Output:   None
**********************************************************/
void ParameterConfig(uint8_t *configurationTable);

/**********************************************************
**Name:     RFM26_SetParameter_Freq
**Function: Set frequency
**Input:    *FreqConfig,Frequency table
**Output:   None
**********************************************************/
void RFM26_SetParameter_Freq(uint8_t *FreqConfig);

/**********************************************************
**Name:     RFM26_SetParameter_Power
**Function: Set out power
**Input:    *PAConfig,Power table
**Output:   None
**********************************************************/
void RFM26_SetParameter_Power(uint8_t *PA);

/**********************************************************
**Name:     RFM26_Start_Tx
**Function: Start Tx
**Input:    Channel,Channel number
            Condition,Transmit condition
            Tx_Length, Data length
**Output:   None
**********************************************************/
void RFM26_Start_Tx(uint8_t Channel, uint8_t Condition, uint16_t Tx_Length);

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
void RFM26_Start_Rx(uint8_t Channel, uint8_t Condition, uint16_t Rx_Length, uint8_t State1, uint8_t State2, uint8_t State3);

/**********************************************************
**Name:     RFM26_SetINT_CTL
**Function: Set interrupt enable bit
**Input:    status,INT_CTL
            ctl_PH,INT_CTL_PH
            ctl_modem, INT_CTL_MODEM
            ctl_chip,INT_CTL_CHIP_EN
**Output:   None
**********************************************************/
void RFM26_SetINT_CTL(uint8_t status, uint8_t ctl_PH, uint8_t ctl_modem, uint8_t ctl_chip);

/**********************************************************
**Name:     RFM26_ChangeToRxMode
**Function: Change to rx mode
**Input:    length,data length
**Output:   None
**********************************************************/
void RFM26_ChangeToRxMode(uint8_t length);

/**********************************************************
**Name:     RFM26_ResetTxFifo
**Function: Reset Tx fifo
**Input:    None
**Output:   None
**********************************************************/
void RFM26_ResetTxFifo(void);

/**********************************************************
**Name:     RFM26_ResetRxFifo
**Function: Reset Rx fifo
**Input:    None
**Output:   None
**********************************************************/
void RFM26_ResetRxFifo(void);

/**********************************************************
**Name:     RFM26_Config
**Function: Initialize RFM26 & set it entry to standby mode
**Input:    none
**Output:   none
**********************************************************/
void RFM26_Config(void);

/**********************************************************
**Name:     RFM26_EntryRx
**Function: Set RFM26 entry Rx_mode
**Input:    None
**Output:   None
**********************************************************/
void RFM26_EntryRx(void);

/**********************************************************
**Name:     RFM26_EntryTx
**Function: Set RFM26 entry Tx_mode
**Input:    None
**Output:   None
**********************************************************/
void RFM26_EntryTx(void);

/**********************************************************
**Name:     RFM26_ClearFIFO
**Function: Change to RxMode from StandbyMode, can clear FIFO buffer
**Input:    None
**Output:   None
**********************************************************/
void RFM26_ClearFIFO(void);

/**********************************************************
**Name:     RFM26_Sleep
**Function: Set RFM26 to sleep mode 
**Input:    none
**Output:   none
**********************************************************/
void RFM26_Sleep(void);

/**********************************************************
**Name:     RFM26_Standby
**Function: Set RFM26 to Standby mode
**Input:    none
**Output:   none
**********************************************************/
void RFM26_Standby(void);

/**********************************************************
**Name:     RFM26_ReadRSSI
**Function: Read the RSSI value
**Input:    none
**Output:   temp, RSSI value
**********************************************************/
uint8_t RFM26_ReadRSSI(void);

/**********************************************************
**Parameter table define in test mode
**********************************************************/

/**********************************************************
**Name:     RFM26_EntryTestRx
**Function: Set RFM26 entry Rx test mode
**Input:    None
**Output:   None
**********************************************************/
void RFM26_EntryTestRx(void);

/**********************************************************
**Name:     RFM26_CarrierTest
**Function: Set RFM26 entry CW mode
**Input:    None
**Output:   None
**********************************************************/
void RFM26_CarrierTest(void) ;                   

/**********************************************************
**Name:     RFM26_EntryTestTx
**Function: Set RFM26 entry Tx test mode
**Input:    None
**Output:   None
**********************************************************/
void RFM26_EntryTestTx(void);

/**********************************************************
**Name:     RFM26_TestRx
**Function: RFM26 Rx test mode
**Input:    None
**Output:   "0" for Error Status
**********************************************************/
void RFM26_TestRx(void);

/**********************************************************
**Name:     RFM26_TestTx
**Function: RFM26 Tx test mode
**Input:    None
**Output:   "0" for Error Status
**********************************************************/
void RFM26_TestTx(void);

/**********************************************************/
uint8_t receive_message(uint8_t* p_data);

/**********************************************************/
void send_message(uint8_t* p_data,uint8_t num);

#ifdef __cplusplus
}
#endif

#endif
