/**
******************************************************************************
* File Name          : connectivity.c
* Description        : Functions related to connectivity
******************************************************************************
*
* COPYRIGHT(c) 2016 STMicroelectronics
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*   1. Redistributions of source code must retain the above copyright notice,
*      this list of conditions and the following disclaimer.
*   2. Redistributions in binary form must reproduce the above copyright notice,
*      this list of conditions and the following disclaimer in the documentation
*      and/or other materials provided with the distribution.
*   3. Neither the name of STMicroelectronics nor the names of its contributors
*      may be used to endorse or promote products derived from this software
*      without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*/
/* Includes ------------------------------------------------------------------*/
#include "connectivity.h"
#include "system.h"//this file to be handled better in future
#include "string.h"

//#include "system.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
DataQue RxQ;
DataQue TxQ;
uint8_t PacketRxData[PACKET_LENGTH];
uint8_t PacketTxData[PACKET_LENGTH];
uint8_t dupPacketTxData[PACKET_LENGTH];
extern UART_HandleTypeDef huart2;			//Added By Venky

// Added by venky to log the data to the PC
char packetTxBuffer[50];

bool Flag_ReadPacketData = false;
bool Flag_SendPacketData = false;
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/* This function will read the data from Spirit module through UART and store in the RxQ buffer */
void conn_ReadData(uint8_t data)
{
  RxQ.buff[RxQ.endNxtPntr] = data;
  RxQ.endNxtPntr++;
  if (RxQ.endNxtPntr >= DATABUFF_SIZE)
  {
    RxQ.endNxtPntr = 0;
  }
  
  if (RxQ.endNxtPntr >= RxQ.startPntr)
  {
    RxQ.length = RxQ.endNxtPntr - RxQ.startPntr;
  }
  else
  {
    RxQ.length = (RxQ.endNxtPntr + DATABUFF_SIZE) - RxQ.startPntr;
  }   
}

/* This function checks the packet and it will send to Spirit Module */
void conn_PacketHandler(void)
{
  conn_ExtractPacket();
  
  if (TxQ.length > 0)
  {
    if (system_TransmitUartData(TxQ.buff[TxQ.startPntr]))
    {
      TxQ.length -= 1;
      TxQ.startPntr = (TxQ.startPntr + 1) % DATABUFF_SIZE;
    }
  }
}

/* This function validates the packet */
void conn_ExtractPacket(void)
{
  if (RxQ.length >= PACKET_LENGTH)
  {
    if (RxQ.buff[RxQ.startPntr] == PACKET_START_IDENTIFIER)
    {
      if(RxQ.buff[(RxQ.startPntr+1) % DATABUFF_SIZE] == PACKET_START_IDENTIFIER)
      {
        if(RxQ.buff[((RxQ.startPntr + PACKET_LENGTH) - 1) % DATABUFF_SIZE] 
           == PACKET_STOP_IDENTIFIER)
        {
          if(RxQ.buff[((RxQ.startPntr + PACKET_LENGTH) - 2) % DATABUFF_SIZE] 
             == PACKET_STOP_IDENTIFIER)
          {
            for (uint16_t count=0; count<PACKET_LENGTH; count++)
            {
              PacketRxData[count] = RxQ.buff[(RxQ.startPntr+count)%DATABUFF_SIZE];
            }
            conn_SetReadPacketDataStatus(true);
            
            RxQ.startPntr = (RxQ.startPntr + PACKET_LENGTH) % DATABUFF_SIZE;
            if (RxQ.endNxtPntr >= RxQ.startPntr)
            {
              RxQ.length = RxQ.endNxtPntr - RxQ.startPntr;
            }
            else
            {
              RxQ.length = (RxQ.endNxtPntr + DATABUFF_SIZE) - RxQ.startPntr;
            }   
          }
        }
      }
      else
      {
        RxQ.startPntr = (RxQ.startPntr+2) % DATABUFF_SIZE;
        RxQ.length -= 2;
      }
    }
    else
    {
      RxQ.startPntr = (RxQ.startPntr+1)%DATABUFF_SIZE;
      RxQ.length -= 1;
    }
  }
}

/* This function will set the status for read */
void conn_SetReadPacketDataStatus(bool status)
{
  Flag_ReadPacketData = status;
}
/* This function gives the present status of read packet */
bool conn_GetReadPacketDataStatus(void)
{
  return Flag_ReadPacketData;
}

/* This function gets the packet data at partcular index */
uint8_t conn_GetPacketData(uint16_t index)
{
  return PacketRxData[index];
}
/* This function set the data at particular index */
void conn_SetPacketData(uint16_t index, uint8_t data)
{
  PacketTxData[index] = data;
}

void dupconn_SetPacketData(uint16_t index, uint8_t data)
{
  dupPacketTxData[index] = data;
}

/* This function validates the packet and stores*/
void conn_SetSendPacketStatus(bool status)
{
  if (status == true)
  {
    PacketTxData[PACKET_START_INDEX_1] =  PACKET_START_IDENTIFIER;
    PacketTxData[PACKET_START_INDEX_2] =  PACKET_START_IDENTIFIER;
    PacketTxData[COMMAND_CODE_INDEX]   =  COMMAND_SEND_STATUS;
    PacketTxData[PACKET_STOP_INDEX_1]  =  PACKET_STOP_IDENTIFIER;
    PacketTxData[PACKET_STOP_INDEX_2]  =  PACKET_STOP_IDENTIFIER;
    dupPacketTxData[PACKET_START_INDEX_1] =  PACKET_START_IDENTIFIER;
    dupPacketTxData[PACKET_START_INDEX_2] =  PACKET_START_IDENTIFIER;
    dupPacketTxData[COMMAND_CODE_INDEX]   =  COMMAND_SEND_STATUS;
    dupPacketTxData[PACKET_STOP_INDEX_1]  =  PACKET_STOP_IDENTIFIER;
    dupPacketTxData[PACKET_STOP_INDEX_2]  =  PACKET_STOP_IDENTIFIER;
  }
  
  for (uint16_t count=0; count<PACKET_LENGTH; count++)
  {
    TxQ.buff[TxQ.endNxtPntr] = PacketTxData[count];
    TxQ.endNxtPntr = (TxQ.endNxtPntr + 1) % DATABUFF_SIZE;
  }  
  
  if (TxQ.endNxtPntr >= TxQ.startPntr)
  {
    TxQ.length = TxQ.endNxtPntr - TxQ.startPntr;
  }
  else
  {
    TxQ.length = (TxQ.endNxtPntr + DATABUFF_SIZE) - TxQ.startPntr;
  }  
  
  Flag_SendPacketData = status;
}

void conn_ResetConn(void)
{
  RxQ.startPntr = 0;
  RxQ.endNxtPntr = 0;
  RxQ.length = 0;
  
  TxQ.startPntr = 0;
  TxQ.endNxtPntr = 0;
  TxQ.length = 0;
  
  Flag_ReadPacketData = false;
  Flag_SendPacketData = false;  
}


/*
void fun(int ch);
void fun(int ch){
HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1,0xFFFFFFFFU);
}  */
//
//static void getPacketStartByte1Data(){
//  //sprintf(packetTxBuffer, "\r\n Packet start Index_1:%0X", dupPacketTxData[0]);
//  HAL_UART_Transmit(&huart2, (uint8_t *)dupPacketTxData[0], strlen((char *)dupPacketTxData[0]),1000);
//  memset(packetTxBuffer, 0, sizeof(packetTxBuffer));
//}
//static void getPacketStartByte2Data(){
//  sprintf(packetTxBuffer, "\r\n Packet start Index_2:%0X", dupPacketTxData[1]);
//  HAL_UART_Transmit(&huart2, (uint8_t *)&packetTxBuffer, 26,1000);
//  memset(packetTxBuffer, 0, sizeof(packetTxBuffer));
//}
//static void getCommandCodeData(){
//  sprintf(packetTxBuffer, "\r\n Command Code :%d", dupPacketTxData[2]);
//  HAL_UART_Transmit(&huart2, (uint8_t *)&packetTxBuffer, 20,1000);
//  memset(packetTxBuffer, 0, sizeof(packetTxBuffer));
//}
//static void getOnOffStatusData(){
//  sprintf(packetTxBuffer, "\r\n On Off Status:%d", dupPacketTxData[3]);
//  HAL_UART_Transmit(&huart2, (uint8_t *)&packetTxBuffer, strlen(packetTxBuffer),3000);
//  memset(packetTxBuffer, 0, sizeof(packetTxBuffer));
//}
//static void getSubSystemData(){
//  sprintf(packetTxBuffer, "\r\n Sub System Index:%X", dupPacketTxData[4]);
//  HAL_UART_Transmit(&huart2, (uint8_t *)&packetTxBuffer, 21,1000);
//  memset(packetTxBuffer, 0, sizeof(packetTxBuffer));
//}
//static void getSensorStatus(){
//  sprintf(packetTxBuffer, "\r\n Sensor Status:%X", dupPacketTxData[5]);
//  HAL_UART_Transmit(&huart2,(uint8_t *) &packetTxBuffer, 19,1000);
//  memset(packetTxBuffer, 0, sizeof(packetTxBuffer));
//  
//}
//static void getBatteryStatus(){
//  sprintf(packetTxBuffer, "\r\n Battery Status:%X", dupPacketTxData[6]);
//  HAL_UART_Transmit(&huart2,(uint8_t *)& packetTxBuffer, 20,1000);
//  memset(packetTxBuffer, 0, sizeof(packetTxBuffer));
//  
//}
//static void getLedCurrent(){
//  sprintf(packetTxBuffer, "%0f", dupPacketTxData[12]/10);
//  //HAL_UART_Transmit(&huart2, (uint8_t *)& packetTxBuffer, 23,1000);
//  if(packetTxBuffer[0]>'2'){
//     int i =0;
//     sprintf(packetTxBuffer, "%0d", dupPacketTxData[12]/10);
//  }
//
//}
//static void getLedVoltage(){
//  sprintf(packetTxBuffer, "\r\n Led Voltage:%0f", dupPacketTxData[13]/10);
//  HAL_UART_Transmit(&huart2, (uint8_t *)&packetTxBuffer, 23,1000);
//  memset(packetTxBuffer, 0, sizeof(packetTxBuffer));
//}
//static void getBatteryCurrent(){
//  sprintf(packetTxBuffer, "\r\n Battery Current:%0f", dupPacketTxData[14]/10);
//  HAL_UART_Transmit(&huart2, (uint8_t *)&packetTxBuffer, 27,1000);
//  memset(packetTxBuffer, 0, sizeof(packetTxBuffer));
//}
//static void getBatteryVoltage(){
//  sprintf(packetTxBuffer, "\r\n Battery Voltage:%0f", dupPacketTxData[15]/10);
//  HAL_UART_Transmit(&huart2, (uint8_t *)&packetTxBuffer, 27,1000);
//  memset(packetTxBuffer, 0, sizeof(packetTxBuffer));
//}
//static void getPanelCurrent(){
//  sprintf(packetTxBuffer, "\r\n Panel Current:%0f", dupPacketTxData[16]/10);
//  HAL_UART_Transmit(&huart2,(uint8_t *) &packetTxBuffer, 25,1000);
//  memset(packetTxBuffer, 0, sizeof(packetTxBuffer));
//}
//static void getPanelVoltage(){
//  sprintf(packetTxBuffer, "\r\n Panel Voltage:%0f", dupPacketTxData[17]/10);
//  HAL_UART_Transmit(&huart2, (uint8_t *)&packetTxBuffer, 25,1000);
//  memset(packetTxBuffer, 0, sizeof(packetTxBuffer));
//}
//static void getPacketStopByte1(){
//  sprintf(packetTxBuffer, "\r\n Packet stop Index_1:%X", dupPacketTxData[18]);
//  HAL_UART_Transmit(&huart2, (uint8_t *)&packetTxBuffer, 25,1000);
//  memset(packetTxBuffer, 0, sizeof(packetTxBuffer));
//  
//}
//static void getPacketStopByte2(){
//  sprintf(packetTxBuffer, "\r\n Packet stop Index_2:%X", dupPacketTxData[19]);
//  HAL_UART_Transmit(&huart2, (uint8_t *)&packetTxBuffer, 25,1000); 
//  memset(packetTxBuffer, 0, sizeof(packetTxBuffer));
//}

//static void getPacketStartByte1Data(){
//  //sprintf(packetTxBuffer, "\r\n Packet start Index_1:%0X", dupPacketTxData[0]);
//  HAL_UART_Transmit(&huart2, (uint8_t *)dupPacketTxData[0], strlen((char *)dupPacketTxData[0]),1000);
//  memset(packetTxBuffer, 0, sizeof(packetTxBuffer));
//}
//static void getPacketStartByte2Data(){
//  sprintf(packetTxBuffer, "\r\n Packet start Index_2:%0X", dupPacketTxData[1]);
//  HAL_UART_Transmit(&huart2, (uint8_t *)&packetTxBuffer, 26,1000);
//  memset(packetTxBuffer, 0, sizeof(packetTxBuffer));
//}
//static void getCommandCodeData(){
//  sprintf(packetTxBuffer, "\r\n Command Code :%d", dupPacketTxData[2]);
//  HAL_UART_Transmit(&huart2, (uint8_t *)&packetTxBuffer, 20,1000);
//  memset(packetTxBuffer, 0, sizeof(packetTxBuffer));
//}
//static void getOnOffStatusData(){
//  sprintf(packetTxBuffer, "\r\n On Off Status:%d", dupPacketTxData[3]);
//  HAL_UART_Transmit(&huart2, (uint8_t *)&packetTxBuffer, strlen(packetTxBuffer),3000);
//  memset(packetTxBuffer, 0, sizeof(packetTxBuffer));
//}
//static void getSubSystemData(){
//  sprintf(packetTxBuffer, "\r\n Sub System Index:%X", dupPacketTxData[4]);
//  HAL_UART_Transmit(&huart2, (uint8_t *)&packetTxBuffer, 21,1000);
//  memset(packetTxBuffer, 0, sizeof(packetTxBuffer));
//}
//static void getSensorStatus(){
//  sprintf(packetTxBuffer, "\r\n Sensor Status:%X", dupPacketTxData[5]);
//  HAL_UART_Transmit(&huart2,(uint8_t *) &packetTxBuffer, 19,1000);
//  memset(packetTxBuffer, 0, sizeof(packetTxBuffer));
//  
//}
//static void getBatteryStatus(){
//  sprintf(packetTxBuffer, "\r\n Battery Status:%X", dupPacketTxData[6]);
//  HAL_UART_Transmit(&huart2,(uint8_t *)& packetTxBuffer, 20,1000);
//  memset(packetTxBuffer, 0, sizeof(packetTxBuffer));
//  
//}
//static void getLedCurrent(){
//  sprintf(packetTxBuffer, "\r\n Led Current:%0f", dupPacketTxData[12]/10);
//  HAL_UART_Transmit(&huart2, (uint8_t *)& packetTxBuffer, 23,1000);
//  memset(packetTxBuffer, 0, sizeof(packetTxBuffer));
//  
//}
//static void getLedVoltage(){
//  sprintf(packetTxBuffer, "\r\n Led Voltage:%0f", dupPacketTxData[13]/10);
//  HAL_UART_Transmit(&huart2, (uint8_t *)&packetTxBuffer, 23,1000);
//  memset(packetTxBuffer, 0, sizeof(packetTxBuffer));
//}
//static void getBatteryCurrent(){
//  sprintf(packetTxBuffer, "\r\n Battery Current:%0f", dupPacketTxData[14]/10);
//  HAL_UART_Transmit(&huart2, (uint8_t *)&packetTxBuffer, 27,1000);
//  memset(packetTxBuffer, 0, sizeof(packetTxBuffer));
//}
//static void getBatteryVoltage(){
//  sprintf(packetTxBuffer, "\r\n Battery Voltage:%0f", dupPacketTxData[15]/10);
//  HAL_UART_Transmit(&huart2, (uint8_t *)&packetTxBuffer, 27,1000);
//  memset(packetTxBuffer, 0, sizeof(packetTxBuffer));
//}
//static void getPanelCurrent(){
//  sprintf(packetTxBuffer, "\r\n Panel Current:%0f", dupPacketTxData[16]/10);
//  HAL_UART_Transmit(&huart2,(uint8_t *) &packetTxBuffer, 25,1000);
//  memset(packetTxBuffer, 0, sizeof(packetTxBuffer));
//}
//static void getPanelVoltage(){
//  sprintf(packetTxBuffer, "\r\n Panel Voltage:%0f", dupPacketTxData[17]/10);
//  HAL_UART_Transmit(&huart2, (uint8_t *)&packetTxBuffer, 25,1000);
//  memset(packetTxBuffer, 0, sizeof(packetTxBuffer));
//}
//static void getPacketStopByte1(){
//  sprintf(packetTxBuffer, "\r\n Packet stop Index_1:%X", dupPacketTxData[18]);
//  HAL_UART_Transmit(&huart2, (uint8_t *)&packetTxBuffer, 25,1000);
//  memset(packetTxBuffer, 0, sizeof(packetTxBuffer));
//  
//}
//static void getPacketStopByte2(){
//  sprintf(packetTxBuffer, "\r\n Packet stop Index_2:%X", dupPacketTxData[19]);
//  HAL_UART_Transmit(&huart2, (uint8_t *)&packetTxBuffer, 25,1000); 
//  memset(packetTxBuffer, 0, sizeof(packetTxBuffer));
//}

void logDatatoPC(){   
  if(strcmp((const char *)dupPacketTxData[0], "AB") && strcmp((const char *)dupPacketTxData[19], "BA")){
    HAL_UART_Transmit(&huart2, (uint8_t *)&dupPacketTxData, 20,1000);
    //getBatteryCurrent();
  }
}
    
//  getPacketStartByte1Data();
//  getPacketStartByte2Data();
//  getCommandCodeData();
//  getOnOffStatusData();
//  getSubSystemData();
//  getSensorStatus();
//  getBatteryStatus();
//  getLedCurrent();
//  getLedVoltage();
//  getBatteryCurrent();
//  getBatteryVoltage();
//  getPanelCurrent();
//  getPanelVoltage();
//  getPacketStopByte1();
//  getPacketStopByte2();    
  
  /*
  if(Rx_Buff[0]=='1'){
  getPacketStartByte1Data();
}
  if(Rx_Buff[1]=='1'){
  getPacketStartByte2Data();
}
  if(Rx_Buff[2]=='1'){
  getCommandCodeData();
}
  if(Rx_Buff[3]=='1'){
  getOnOffStatusData();
}
  if(Rx_Buff[4]=='1'){
  getSubSystemData();
}
  if(Rx_Buff[5]=='1'){
  getSensorStatus();
}
  if(Rx_Buff[6]=='1'){
  getBatteryStatus();
}  
  if(Rx_Buff[7]=='1'){
  getLedCurrent();
}
  if(Rx_Buff[8]=='1'){
  getLedVoltage();
}
  if(Rx_Buff[9]=='1'){
  getBatteryCurrent();
}
  if(Rx_Buff[10]=='1'){
  getBatteryVoltage();
}
  if(Rx_Buff[11]=='1'){
  getPanelCurrent();
}
  if(Rx_Buff[12]=='1'){
  getPanelVoltage();
}
  if(Rx_Buff[13]=='1'){
  getPacketStopByte1();
}
  if(Rx_Buff[14]=='1'){
  getPacketStopByte2();
}else{
  getPacketStartByte1Data();
  getPacketStartByte2Data();
  getCommandCodeData();
  getOnOffStatusData();
  getSubSystemData();
  getSensorStatus();
  getBatteryStatus();
  getLedCurrent();
  getLedVoltage();
  getBatteryCurrent();
  getBatteryVoltage();
  getPanelCurrent();
  getPanelVoltage();
  getPacketStopByte1();
  getPacketStopByte2(); 
}
  //memset(Rx_Buff,0,sizeof(Rx_Buff)); */

/**
* @}
*/

/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
