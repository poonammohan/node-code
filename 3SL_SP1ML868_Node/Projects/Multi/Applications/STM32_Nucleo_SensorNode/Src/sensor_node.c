/**
******************************************************************************
* @file    sensor_node.c 
* @author  Central Labs
* @version V1.0.0
* @date    15-March-2016
* @brief   Manage sensor node info and iterface with sensors in the node
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
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
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
#include "sensor_node.h"
#include "hw-config.h"
#include "contiki.h"
#include "math.h"


/**
 * @addtogroup USER
 * @{
 */


/** @addtogroup Sensor_Node_Applications
 *  @{
 */
 
/** @defgroup SENSOR_NODE
 * @{
 */

/** @defgroup SENSOR_NODE_TypesDefinitions
 * @{
 */

/**
  * @}
  */

/** @defgroup SENSOR_NODE_Private_Defines
  * @{
  */

/**
  * @}
  */


/** @defgroup SENSOR_NODE_Private_Macros
  * @{
  */
/* Private macros ------------------------------------------------------------*/
#define USART_PRINT_MSG
#ifndef NULL
#define NULL ((void *)0)
#endif
/**
  * @}
  */
/** @defgroup SENSOR_NODE_Private_Variables
  * @{
  */



extern UART_HandleTypeDef UartHandle;

nodeInfo node;
nodeInfo Node[10];
uint8_t NodePresent = 0;

/**
  * @}
  */



/** @defgroup SENSOR_NODE_Exported_Variables
* @{
*/
/**
* @}
*/ 

/** @defgroup SENSOR_NODE_Private_FunctionPrototypes
* @{
*/ 

uint8_t IsParentIdSame(uint8_t* prevParentId,uint8_t* parentId);
void RemoveChild(uint8_t nodeIndex, uint8_t childIndex);

/**
* @}
*/ 

/** @defgroup SENSOR_NODE_Private_Functions
  * @{
  */


/**
* @brief  SN_Initialize node with default values
* @param  None
* @retval None
*/ 
void SN_InitializeNode(){
  /*Initializes sensor Ids, default sensor thresholds, alarmstatus*/
  node.sensorsCount = MAX_NO_OF_SENSORS;
  
  node.sensor[0].Id = TEMPERATURE_SENSOR_ID;
  node.sensor[1].Id = PRESSURE_SENSOR_ID;
  node.sensor[8].Id = HUMIDITY_SENSOR_ID;
  node.sensor[2].Id = ACCELEROMETER_ID;
  node.sensor[3].Id = ACCELEROMETER_YAXIS_ID;
  node.sensor[4].Id = ACCELEROMETER_ZAXIS_ID;  
  node.sensor[5].Id = GYRX_ID;
  node.sensor[6].Id = GYRY_ID;
  node.sensor[7].Id = GYRZ_ID;

  node.sensor[9].Id = MAGX_ID;
  node.sensor[10].Id = MAGY_ID;
  node.sensor[11].Id = MAGZ_ID;
}



/**
* @brief  Update Node Status
* @param  nodeNo: Node number
* @param  nodeId: Node Ip ddress
* @param  sensorData: pointer to sensor write 
* @param  parentId: parent ip address
* @retval StatusBytes
*/
void SN_UpdateNodeStatus(uint16_t nodeNo, uint8_t* nodeId, uint8_t* sensorData, uint8_t* parentId){
  uint8_t nodeIndex = 100;//initialized to 100, genuine value should be between 0 and 9
  uint8_t parentIndex = 100;
  nodeIndex = IsNodePresent(nodeId);
  /*if the data is received from the node not already have its entry at the
  concentrator, create new entry corresponding to it*/
  if (nodeIndex >= 10){
    //create new node in the structure with the given data
    NodePresent++;
    InitializeNode(nodeNo,nodeId,sensorData,parentId);
    /*check if its parent is present*/
    parentIndex = IsNodePresent(parentId);
    if (parentIndex >= 10){
      //parent is not present
      NodePresent++;
      InitializeNode(65535,parentId,NULL,NULL);
      /*acutal index of parent is NodePresent-1 and of child is NodePresent-2*/
      UpdateChild(NodePresent-1,NodePresent-2);//index id of child is NodePresent-1
    }
    else{
      /*parent is present, update it with the new child info*/
      UpdateChild(parentIndex,NodePresent-1);
    }
  }
  
  else{
    uint8_t* prevParentId;
    
    //UpdateNodeData returns the id of the previous parent stored
    prevParentId = UpdateNodeData(nodeNo,nodeIndex,sensorData,parentId);
    //IsNodePresent will tell if the current parent is present
    parentIndex = IsNodePresent(parentId);
    if (parentIndex >= 10){
      //parent is not present
      NodePresent++;
      InitializeNode(65535,parentId,NULL,NULL);
      UpdateChild(NodePresent-1,nodeIndex);//index id NodePresent-1
      RemoveChild(IsNodePresent(prevParentId),nodeIndex);//Remove its association from the previous parent
    }
    else{
      /*check if associated to the same parent*/
      if(IsParentIdSame(prevParentId,parentId)){
        //do nothing
      }
      else{
        //associate it to the new parent
        UpdateChild(parentIndex,nodeIndex);
        RemoveChild(IsNodePresent(prevParentId),nodeIndex);//Remove its association from the previous parent
      }
    }
  }
}

/**
* @brief  check node is present or not
* @param  nodeId: Node ip address
* @retval node index in the array of nodes if node is present
*/
uint8_t IsNodePresent(uint8_t* nodeId)
{
  for (uint8_t count=0; count<NodePresent; count++)
  {
    if (Node[count].Id[0] == nodeId[0]&&
        Node[count].Id[1] == nodeId[1]&&
          Node[count].Id[2] == nodeId[2]&&
            Node[count].Id[3] == nodeId[3]&&
              Node[count].Id[4] == nodeId[4]&&
                Node[count].Id[5] == nodeId[5]&&
                  Node[count].Id[6] == nodeId[6]&&
                    Node[count].Id[7] == nodeId[7])
                    {
                      return count;
                    }
  }
  return 100;
}

/**
* @brief  Update Node Data structure
* @param  nodeNo: Node number
* @param  nodeIndex: Node offset
* @param  sensorData: pointer to sensor write 
* @param  parentId: parent ip address
* @retval StatusBytes
*/
uint8_t* UpdateNodeData(uint16_t nodeNo, uint8_t nodeIndex,uint8_t* sensorData, uint8_t* parentId){
  static uint8_t tempId[8];
  Node[nodeIndex].No = nodeNo;
  
  //update sensor data
  
  /*stores previous parent ID in tempId and update current parentID*/
  for (uint8_t count=0; count<7; count++){
    tempId[count] = Node[nodeIndex].parentId[count];
    Node[nodeIndex].parentId[count] = parentId[count];
  }
  return tempId;  
}

/**
* @brief  check node ID
* @param  prevParentId: previous parent node ID
* @param  parentId: current parent node ID
* @retval StatusBytes
*/
uint8_t IsParentIdSame(uint8_t* prevParentId,uint8_t* parentId)
{
  if (prevParentId[0] == parentId[0]&&
      prevParentId[1] == parentId[1]&&
        prevParentId[2] == parentId[2]&&
          prevParentId[3] == parentId[3]&&
            prevParentId[4] == parentId[4]&&
              prevParentId[5] == parentId[5]&&
                prevParentId[6] == parentId[6]&&
                  prevParentId[7] == parentId[7]){
                    return 1;
                  }
  return 0;
}

/**
* @brief  Initialize Node
* @param  nodeNo: Node number
* @param  nodeId: Node ID
* @param  sensorData: pointer to sensor write 
* @param  parentId: parent ip address
* @retval None
*/
void InitializeNode(uint16_t nodeNo,uint8_t* nodeId,uint8_t* sensorData, uint8_t* parentId){
  /*Index of the new node is NodePresent-1*/
  Node[NodePresent-1].No = NodePresent;
  for (uint8_t count=0; count<7; count++){
    Node[NodePresent-1].Id[count] = nodeId[count];
    Node[NodePresent-1].parentId[count] = parentId[count];
  }
}

/**
* @brief  
* @param  None
* @retval None
*/
void UpdateChild(uint8_t parentIndex,uint8_t nodeIndex){
  Node[parentIndex].childCount++;
  Node[parentIndex].child[Node[parentIndex].childCount-1] = &Node[nodeIndex];
}

/**
* @brief  
* @param  None
* @retval None
*/
void RemoveChild(uint8_t nodeIndex, uint8_t childIndex)
{
  
}



/**
* @}
*/ 

/**
 * @}
 */ 

/**
 * @}
 */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
