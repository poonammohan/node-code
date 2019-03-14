/**
  ******************************************************************************
  * @file    connectivity.h
  * @brief   Header file for connectivity.c
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
  
  /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CONNECTIVITY_H
#define __CONNECTIVITY_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "common.h"   
   
//#define UART_RX_DATABUFF_SIZE                   1000
//#define UART_TX_DATABUFF_SIZE                   1000
#define DATABUFF_SIZE                           1000//This value should be greater than PACKET_LENGTH   
   
#define PACKET_LENGTH                           20   
#define PACKET_STOP_IDENTIFIER                  0xBA
#define PACKET_START_IDENTIFIER                 0xAB    

#define COMMAND_ON_OFF                          0xA1   
#define COMMAND_SEND_STATUS                     0xB1   
   
#define PACKET_START_INDEX_1                    0   
#define PACKET_START_INDEX_2                    1   
#define PACKET_STOP_INDEX_1                    18   
#define PACKET_STOP_INDEX_2                    19   
   
   
#define COMMAND_CODE_INDEX                      2   
#define ON_OFF_INDEX                            3   
#define SUB_SYSTEM_INDEX                        4   
#define SENSOR_STATUS_INDEX                     5
#define BATTERY_STATUS_INDEX                    6
#define LED_CURRENT_INDEX                       12      // Added By Chinna
#define LED_VOLTAGE_INDEX                       13      // Added By Chinna
#define BATTERY_CURRENT_INDEX                   14
#define BATTERY_VOLTAGE_INDEX                   15
#define PANEL_CURRENT_INDEX                     16
#define PANEL_VOLTAGE_INDEX                     17   
   
/* Exported types ------------------------------------------------------------*/
typedef struct {
  uint8_t buff[DATABUFF_SIZE];
  uint16_t startPntr;
  uint16_t endNxtPntr;
  uint16_t length;
}DataQue; 
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void conn_ReadData(uint8_t);
void conn_PacketHandler(void);
void conn_ExtractPacket(void);
void conn_SetReadPacketDataStatus(bool);
bool conn_GetReadPacketDataStatus(void);
uint8_t conn_GetPacketData(uint16_t);
void conn_SetPacketData(uint16_t,uint8_t);
void conn_SetSendPacketStatus(bool);
void conn_ResetConn(void);
#ifdef __cplusplus
}
#endif

#endif /* __CONNECTIVITY_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
