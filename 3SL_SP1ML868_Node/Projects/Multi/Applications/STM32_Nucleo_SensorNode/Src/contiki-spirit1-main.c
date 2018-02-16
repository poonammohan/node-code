/**
******************************************************************************
* @file    contiki-spirit1-main.c
* @author  System LAB
* @version V1.0.0
* @date    17-June-2015
* @brief   Contiki main file for stm32nucleo-spirit1 platform
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
/**
 * \addtogroup stm32nucleo-spirit1
 * @{
 *
 * \file
 * main file for stm32nucleo-spirit1 platform
 */
/*---------------------------------------------------------------------------*/

#include "contiki.h"
#include "contiki-net.h"
#include "dev/leds.h"
#include "dev/watchdog.h"

#ifdef USE_STM32L1XX_NUCLEO
#include "stm32l1xx.h"
#endif

#ifdef USE_STM32F4XX_NUCLEO
#include "stm32f4xx.h"
#endif

#include "node-id.h"
#include "hw-config.h" 
#include "sensor_node.h"
#include "st-lib.h"

#if NETSTACK_CONF_WITH_IPV6
#include "net/ipv6/uip-ds6.h"
#endif /*NETSTACK_CONF_WITH_IPV6*/

/*---------------------------------------------------------------------------*/
extern unsigned char node_mac[8];
PROCINIT(&etimer_process, &tcpip_process);
extern nodeInfo Node[];

static void set_rime_addr(void);

/*---------------------------------------------------------------------------*/
void Contiki_Init ()
{
  /* init LEDs */
  leds_init();

  /* Initialize Contiki and our processes. */
 
  SN_InitializeNode(); 
    
  ctimer_init();
  rtimer_init();
  watchdog_init();
  process_init();
  process_start(&etimer_process, NULL);
  
  /* Restore node id if such has been stored in external mem */
  node_id_restore(); /* also configures node_mac[] */

  set_rime_addr();
  random_init(node_id);

  netstack_init();


  spirit_radio_driver.on();

  energest_init();
  memcpy(&uip_lladdr.addr, node_mac, sizeof(uip_lladdr.addr));
  /* Setup nullmac-like MAC for 802.15.4 */
  
  queuebuf_init();
  process_start(&tcpip_process, NULL);
 
  start_mesh_node_process();

  autostart_start(autostart_processes);

  watchdog_start();
}


/*---------------------------------------------------------------------------*/
static void set_rime_addr(void)
{
  linkaddr_t addr;
  
  memset(&addr, 0, sizeof(linkaddr_t));
  memcpy(addr.u8, node_mac, sizeof(addr.u8));

  linkaddr_set_node_addr(&addr);
}
/*---------------------------------------------------------------------------*/
/** @} */
