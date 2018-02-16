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

//#include "system.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
DataQue RxQ;
DataQue TxQ;
uint8_t PacketRxData[PACKET_LENGTH];
uint8_t PacketTxData[PACKET_LENGTH];
bool Flag_ReadPacketData = false;
bool Flag_SendPacketData = false;
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

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


void conn_SetReadPacketDataStatus(bool status)
{
  Flag_ReadPacketData = status;
}

bool conn_GetReadPacketDataStatus(void)
{
  return Flag_ReadPacketData;
}

uint8_t conn_GetPacketData(uint16_t index)
{
  return PacketRxData[index];
}

void conn_SetPacketData(uint16_t index, uint8_t data)
{
  PacketTxData[index] = data;
}

void conn_SetSendPacketStatus(bool status)
{
  if (status == true)
  {
    PacketTxData[PACKET_START_INDEX_1] =  PACKET_START_IDENTIFIER;
    PacketTxData[PACKET_START_INDEX_2] =  PACKET_START_IDENTIFIER;
    PacketTxData[COMMAND_CODE_INDEX]   =  COMMAND_SEND_STATUS;
    PacketTxData[PACKET_STOP_INDEX_1]  =  PACKET_STOP_IDENTIFIER;
    PacketTxData[PACKET_STOP_INDEX_2]  =  PACKET_STOP_IDENTIFIER;
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

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
