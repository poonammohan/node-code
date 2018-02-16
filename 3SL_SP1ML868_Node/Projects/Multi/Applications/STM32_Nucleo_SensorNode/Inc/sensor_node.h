/**
  ******************************************************************************
  * @file    sensor_node.h 
  * @author  Central Labs
  * @version V1.0.0
  * @date    28-March-2016
  * @brief   Header for sensor node
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
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
#ifndef __SENSOR_NODE_H
#define __SENSOR_NODE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#ifdef USE_STM32L1XX_NUCLEO
#include "stm32l1xx.h"
#include "stm32l1xx_nucleo.h"
#endif

#ifdef USE_STM32F4XX_NUCLEO
#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#endif

#include "contiki.h"
#include "platform-conf.h"


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define MAX_NO_OF_SENSORS 12   
/************Sensor Ids**************/    
#define TEMPERATURE_SENSOR_ID   0x10
#define PRESSURE_SENSOR_ID      0x20
#define HUMIDITY_SENSOR_ID      0x30
#define ACCELEROMETER_ID        0x40
#define ACCELEROMETER_YAXIS_ID  0x41
#define ACCELEROMETER_ZAXIS_ID  0x42

#define MICROPHONE_ID           0x50
#define PHOTO_SENSOR_ID         0x60
#define BATTERY_ID              0x70

#define GYRX_ID                 0x80
#define GYRY_ID                 0x81
#define GYRZ_ID                 0x82

#define MAGX_ID                 0x90
#define MAGY_ID                 0x91
#define MAGZ_ID                 0x92

#define NO_SENSOR_CONNECTED     0x80
#define MAX_NO_OF_CHILD   20 

#define ACTUATOR_DATA                   0x51
#define ACTUATOR_ID_LED                 0x10
#define ACTUATOR_ID_BUTTON              0x20
#define ACTUATOR_ATTRIBUTE_ON_OFF       0x01 
#define ACTUATOR_ATTRIBUTE_DIMMING      0x02
#define ACTUATOR_DATA_UNICAST           0x00
#define ACTUATOR_DATA_BROADCAST         0x01
#define ACTUATOR_DATA_ON                0xFF
#define ACTUATOR_DATA_OFF               0xBB 
#define ACTUATOR_ACK                    0xAA
    


enum boolean {OFF,ON};    

typedef struct {
  uint8_t Id;
  uint8_t minThreshold;
  uint8_t maxThreshold;
  enum boolean alaramStatus;
  uint16_t data;
}sensorInfo;

 struct nodeInfo{
  /******related to IPv6*******/
  uint8_t No;
  uint8_t parentId[8];
  uint8_t Id[8];
  struct nodeInfo* child[MAX_NO_OF_CHILD];
  uint8_t childCount;
  //RSSI bits 4 to 7. bit 1 shows node is active or dead, bit 0 shows node is enabled or not
  uint8_t status;
  /************related to sensors*********/
  uint8_t sensorsCount;
  sensorInfo sensor[MAX_NO_OF_SENSORS];

};

typedef struct nodeInfo nodeInfo;

typedef struct {
  uint8_t actuator_data;
  uint8_t Id;
  uint8_t attribute;
  uint16_t data;
}actuatorInfo;
 
 
void SN_InitializeNode();    
void SN_UpdateNodeStatus(uint16_t nodeNo, uint8_t* nodeId, uint8_t* sensorData, uint8_t* parentId);
uint8_t IsNodePresent(uint8_t* nodeId);
uint8_t* UpdateNodeData(uint16_t nodeNo,uint8_t nodeIndex,uint8_t* sensorData, uint8_t* parentId);
void InitializeNode(uint16_t nodeNo,uint8_t* nodeId,uint8_t* sensorData, uint8_t* parentId);
void UpdateChild(uint8_t parentIndex, uint8_t nodeIndex);
void start_mesh_node_process(void);

#ifdef __cplusplus
}
#endif
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
