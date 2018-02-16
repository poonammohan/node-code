/**
******************************************************************************
* @file    root-node.c 
* @author  Central Labs
* @version V1.0.0
* @date    15-March-2016
* @brief   Root Node file which receive sensors data from connected nodes and 
*          transfer it to android phone or user application
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
#include "spirit1-config.h"
#include "contiki.h"
#include "sys/ctimer.h"
#include "sys/etimer.h"
#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ip/uip-debug.h"
#include "simple-udp.h"
#include "servreg-hack.h"
#include "net/rpl/rpl.h"
#include "sys/node-id.h"
#include "stm32l1xx_it.h"

/**
 * @addtogroup USER
 * @{
 */


/** @addtogroup Sensor_Node_Applications
 *  @{
 */
 
/** @defgroup Mesh_Node
 * @{
 */

/** @defgroup Mesh_Node_TypesDefinitions
 * @{
 */

/**
  * @}
  */

/** @defgroup Mesh_Node_Private_Defines
  * @{
  */

/**
  * @}
  */


/** @defgroup Mesh_Node_Private_Macros
  * @{
  */
/* Private macros ------------------------------------------------------------*/
#define UDP_PORT 1234 
#define SERVICE_ID 190


#define SEND_INTERVAL (CLOCK_SECOND * 5) // 5 SEC 

#define MAXSIZE 128 
#define MAX_SENSOR 8
/**
  * @}
  */



/** @defgroup Mesh_Node_Private_Variables
  * @{
  */
static struct simple_udp_connection unicast_connection;
uint8_t ExtData [32] ;
uint8_t counterDummy = 0;
uint32_t person_count;
uint16_t LocalRepairCounter=0;
uint16_t attemptedrepairs=0;
extern unsigned char node_mac[8];
uint8_t Send_Data_To_DCU;
/*---------------------------------------------------------------------------*/
/**
  * @}
  */

/** @defgroup Mesh_Node_Private_FunctionPrototypes
  * @{
  */

/**
  * @}
  */


/** @addtogroup Global_variables
*  @{
*/
extern nodeInfo node;
extern int8_t offsetNoise;
extern UART_HandleTypeDef UartHandle;
extern void floatToInt(float in, int32_t *out_int, int32_t *out_dec, int32_t dec_prec);

/**
* @}
*/

/** @defgroup Mesh_Node_Private_Functions
  * @{
  */
PROCESS(mesh_node_process, "Mesh node");

static struct simple_udp_connection unicast_connection;
PROCESS(unicast_receiver_process, "Unicast receiver example process");

AUTOSTART_PROCESSES(&mesh_node_process,&unicast_receiver_process);

/*---------------------------------------------------------------------------*/
#define MAXSIZE 128 
#define MAX_SENSOR 8



char rxBuf[MAXSIZE];
char uart_rxbuff[UART_DATA_SIZE];
char txBuf[UART_DATA_SIZE];


uint8_t BR_Connected =0;
uip_ipaddr_t *ipaddr;

/**
* @brief  start all processes
* @param  None
* @retval None
*/
 void start_mesh_node_process()
 {
  process_start(&mesh_node_process, NULL); 
  process_start(&unicast_receiver_process, NULL);
 }

/**
* @brief  Receive data from sender via simple UDP connection
* @param  c: A pointer to a struct simple_udp_connection
* @param  sender_addr: Sender IP address
* @param  sender_port: Sender UDP port
* @param  receiver_addr: Receiver IP address
* @param  receiver_port: Receiver UDP port
* @param  data: A pointer to the data
* @param  datalen: Length of the data  
* @retval None
*/

static void receiver(struct simple_udp_connection *c,
         const uip_ipaddr_t *sender_addr,
         uint16_t sender_port,
         const uip_ipaddr_t *receiver_addr,
         uint16_t receiver_port,
         const uint8_t *data,
         uint16_t datalen)
{

  // add UART START BYTE
   txBuf[0] = UART_START_BYTE;
   txBuf[1] = UART_START_BYTE;
   txBuf[2] = ACTUATOR_ACTION;
   
   memcpy(&txBuf[3], &data[MAX_PKT_FOR_DCU - MAX_PKT_LEN_FOR_EXT_STM32], MAX_PKT_LEN_FOR_EXT_STM32);   
   
   // add UART STOP BYTE
   txBuf[3 + MAX_PKT_LEN_FOR_EXT_STM32] = UART_STOP_BYTE;
   txBuf[4 + MAX_PKT_LEN_FOR_EXT_STM32] = UART_STOP_BYTE;
   
   if (HAL_OK != HAL_UART_Transmit(&UartHandle, (uint8_t*) &txBuf[0], UART_DATA_SIZE, 1000))       
  {  
    //Error_Handler();
  } 
     
}

/*---------------------------------------------------------------------------*/
static void
route_callback(int event, uip_ipaddr_t *route, uip_ipaddr_t *ipaddr,
               int numroutes)
{

  if(event == UIP_DS6_NOTIFICATION_DEFRT_ADD)    
  {
    //printf("Got a RPL route\n");
    BR_Connected = 1;
  }
  else if(event == UIP_DS6_NOTIFICATION_DEFRT_RM)
  {
    BR_Connected = 0;
  }
}

/**
* @brief  set global address
* @param  None
* @retval uip_ipaddr_t
*/
static uip_ipaddr_t * set_global_address(void)
{
  static uip_ipaddr_t ipaddr;
  int i;
  uint8_t state;

  uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);



  for(i = 0; i < UIP_DS6_ADDR_NB; i++) 
  {
    state = uip_ds6_if.addr_list[i].state;
    if(uip_ds6_if.addr_list[i].isused &&
       (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) 
       {
      uip_debug_ipaddr_print(&uip_ds6_if.addr_list[i].ipaddr);
    }
  }

  return &ipaddr;
}

/**
* @brief  Start Mesh node process
* @param  ev : event 
* @retval data
*/

PROCESS_THREAD(mesh_node_process, ev, data)
{
  static struct uip_ds6_notification n;
  static struct etimer periodic_timer;
  static char addrbuf[MAXSIZE];
  static uint8_t SP1ML_ResetCount = 0;
  uint8_t psz;
  
  PROCESS_BEGIN();
  
  uip_ds6_notification_add(&n, route_callback);
  
  
  etimer_set(&periodic_timer, SEND_INTERVAL);
  
  while(1) 
  {
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&periodic_timer));
    etimer_reset(&periodic_timer);
    rpl_dag_t * pDag;  
    pDag = rpl_get_any_dag();
    if(&(pDag->dag_id) != NULL)
      
    {
      const uip_ipaddr_t *parent;
      parent = (uip_ipaddr_t *)rpl_get_parent_ipaddr(pDag->preferred_parent);
#if 0 //indar   
      if(parent == NULL) 
#else
    if(!BR_Connected)
#endif      
      {
        //printf("Not sending, we have got no parent.\n");
        LocalRepairCounter+=1;
        
        if(LocalRepairCounter==10)//??
        {
          //  simple_rpl_local_repair();
          LocalRepairCounter=0;
          // attemptedrepairs +=1;
        }
	
      } 
      else 
      {
        // add UART START BYTE
        txBuf[0] = UART_START_BYTE;
        txBuf[1] = UART_START_BYTE;
        txBuf[2] = ACTUATOR_STATUS;
        
        // add UART STOP BYTE
        txBuf[3 + MAX_PKT_LEN_FOR_EXT_STM32] = UART_STOP_BYTE;
        txBuf[4 + MAX_PKT_LEN_FOR_EXT_STM32] = UART_STOP_BYTE;
        
      //  memset(&uart_rxbuff[0], 0, UART_DATA_SIZE); 
        if (HAL_OK != HAL_UART_Transmit(&UartHandle, (uint8_t*) &txBuf[0], UART_DATA_SIZE, 1000))       
        {  
          //Error_Handler();
        } 

      }
    }
    else 
    {
      //printf("Nt sending, we hav n RPL root.\n");
      LocalRepairCounter=0;
    }
  }
  
  PROCESS_END();
}

void SendPacket_To_BR()
{
  rpl_dag_t * pDag;  
  pDag = rpl_get_any_dag();
  if(&(pDag->dag_id) != NULL)    
  {
    const uip_ipaddr_t *parent;
    parent = (uip_ipaddr_t *)rpl_get_parent_ipaddr(pDag->preferred_parent);
    
    if(BR_Connected)
    {  
#if 0 //indar as per KD DCU        
      rxBuf[0] = 0x41;
      memcpy(&rxBuf[1],ipaddr,16);        
#else //indar DCU
      rxBuf[0] = 0x41;
      memcpy(&rxBuf[1],(uint8_t *) &parent->u8[0],16);
#endif  
      
      
      if((uart_rxbuff[0] == UART_START_BYTE) 
         && (uart_rxbuff[1] == UART_START_BYTE) 
           && (uart_rxbuff[2] == ACTUATOR_STATUS)  
             && (uart_rxbuff[UART_DATA_SIZE-1] == UART_STOP_BYTE)
               && (uart_rxbuff[UART_DATA_SIZE-2] == UART_STOP_BYTE))
      {
        // remove start byte( 2B) from uart_rxbuff and copy 15 B of lighting data in rxBuf
        memcpy(&rxBuf[MAX_PKT_FOR_DCU - MAX_PKT_LEN_FOR_EXT_STM32],(uint8_t *) &uart_rxbuff[3],MAX_PKT_LEN_FOR_EXT_STM32); 
        simple_udp_sendto(&unicast_connection, rxBuf, MAX_PKT_FOR_DCU, &(pDag->dag_id));
      } 
      
      else
      {
        
        //nothing to do , packet is corrupted
        
      }
    }
  }
}


/**
* @brief  start unicast receiver process
* @param  ev : event 
* @retval data
*/
PROCESS_THREAD(unicast_receiver_process, ev, data)
{
//  uip_ipaddr_t *ipaddr;

  PROCESS_BEGIN();

  servreg_hack_init();
 
  ipaddr = set_global_address();

 // create_rpl_dag(ipaddr);

  servreg_hack_register(SERVICE_ID, ipaddr);

  simple_udp_register(&unicast_connection, UDP_PORT,
                      NULL, UDP_PORT, receiver);
  while(1) {
    PROCESS_WAIT_EVENT();
  }
  PROCESS_END();
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

/**
 * @}
 */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
