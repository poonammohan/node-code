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
 * @mainpage
 * @section Introduction
 *	OsxContiki6LP is a software package that expands the functionality provided by STM32Cube. The library is implemented as a middleware ready to be integrated in projects based on STM32Cube and the X-CUBE-SUBG1 expansion software. OsxContiki6LP requires, in fact, the X-CUBE-SUBG1 (version 1.1.1 or higher) package to work.
 *	The key features of osxContiki6LP are:
 *		- Middleware library with Contiki OS and Contiki 6LoWPAN protocol stack 3.x
 *		- Support for mesh networking technology by the means of the standard RPL protocol
 *		- Built-in support for STM32 L1 and F4 platforms.
 *		- Easy portability across different STM32 MCU families thanks to STM32Cube.
 *		- Sample applications enabling communication via UDP over 6LoWPAN.
 *		- Free user-friendly license terms.
 *		- Examples available for NUCLEO-F401RE and NUCLEO-L152RE boards.
 *
 *	This software provides a porting of the Contiki OS for the STM32 platform, in the STM32Cube software environment, enabling the support for the SPIRIT1 sub-1GHz radio transceiver.
 *	The package also includes three sample applications that the developer can use to start experimenting with the code. Two of these applications are supposed to be used together since they implement the communication between two nodes. A sender node (@ref Udp_sender) is going to periodically send UDP packets to a receiver node (@ref Udp_receiver) looking up for a specific service number in the same 6LoWPAN network. A third application implements a 6LoWPAN border router (@ref Border_router) functionality.
 *
 */

/**
 * @defgroup STM32_Contiki_Library
 * @{
 */

/**
 * @addtogroup STM32_Contiki_Library
 * @{
 */
/*---------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "contiki.h"
#include "contiki-net.h"
#include "sys/autostart.h"
#include "dev/leds.h"
#include "dev/slip.h"
#include "dev/watchdog.h"
#include "lib/random.h"
#include "net/netstack.h"
#include "net/ip/uip.h"
#include "net/mac/frame802154.h"
#include "SPIRIT_Config.h"
#include "SPIRIT_Management.h"
#include "spirit1.h"
#include "spirit1-arch.h"
#include "node-id.h"
#include "stdbool.h"
#include "dev/button-sensor.h"
#include "dev/radio-sensor.h"
#ifdef USE_X_CUBE_IDW01M1
#include "wifi_const.h"
#include "wifi_module.h"
#endif

#ifdef WITH_IP64
#include "ip64.h"
#include "ip64-addr.h"
#endif

#ifdef USE_STM32L1XX_NUCLEO
#include "stm32l1xx.h"
#include "hw-config.h" 
#endif

#ifdef USE_STM32F4XX_NUCLEO
#include "stm32f4xx.h"
#endif
/*---------------------------------------------------------------------------*/
#if NETSTACK_CONF_WITH_IPV6
#include "net/ipv6/uip-ds6.h"
#endif /*NETSTACK_CONF_WITH_IPV6*/
/*---------------------------------------------------------------------------*/
#if COMPILE_SENSORS
extern const struct sensors_sensor temperature_sensor;
extern const struct sensors_sensor humidity_sensor;
extern const struct sensors_sensor pressure_sensor;
extern const struct sensors_sensor magneto_sensor;
extern const struct sensors_sensor acceleration_sensor;
extern const struct sensors_sensor gyroscope_sensor;
SENSORS(&button_sensor,
        &radio_sensor,
        &temperature_sensor,
        &humidity_sensor,
	&pressure_sensor,
	&magneto_sensor,
        &acceleration_sensor,
	&gyroscope_sensor);
#else /*COMPILE_SENSORS*/
//SENSORS(&button_sensor,
//        &radio_sensor);
#endif /*COMPILE_SENSORS*/
/*---------------------------------------------------------------------------*/
extern unsigned char node_mac[8];
/*---------------------------------------------------------------------------*/
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */  
/*---------------------------------------------------------------------------*/
#if NETSTACK_CONF_WITH_IPV6
PROCINIT(&etimer_process, &tcpip_process);
#else /*NETSTACK_CONF_WITH_IPV6*/
PROCINIT(&etimer_process);
#warning "No TCP/IP process!"
#endif /*NETSTACK_CONF_WITH_IPV6*/
/*---------------------------------------------------------------------------*/
#if 0
#define BUSYWAIT_UNTIL(cond, max_time)                                  \
do {                                                                    \
  rtimer_clock_t t0;                                                    \
    t0 = RTIMER_NOW();                                                  \
      while(!(cond) && RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + (max_time))); \
} while(0)
#endif
/*---------------------------------------------------------------------------*/
static void set_rime_addr(void);
void Stack_6LoWPAN_Init(void);
/*---------------------------------------------------------------------------*/
#if 0
static void panic_main(void)
{
  volatile uint16_t k;
  while(1) {
    leds_toggle(LEDS_ALL);
    for(k = 0; k < 0xffff/8; k += 1) { }
  }
}
#endif
/*---------------------------------------------------------------------------*/

/**
 * @brief  Stack_6LoWPAN_Init program
 * 		initialises Contiki structures, the SPIRIT1 and runs all the PROCESSes
 * @param  None
 * @retval None
 */
void Stack_6LoWPAN_Init(void)
{

  /* Initialize Contiki and our processes. */
//  clock_init(); //it should be in main.c file
  ctimer_init();
  rtimer_init();
  watchdog_init();

  process_init();
  process_start(&etimer_process, NULL);

  /* Restore node id if such has been stored in external mem */
  node_id_restore(); /* also configures node_mac[] */

  random_init(node_id);

  set_rime_addr();

  netstack_init();

  spirit_radio_driver.on();

  energest_init();

    
#if NETSTACK_CONF_WITH_IPV6
  memcpy(&uip_lladdr.addr, node_mac, sizeof(uip_lladdr.addr));
  
  queuebuf_init();
  process_start(&tcpip_process, NULL);

  uip_ipaddr_t ipaddr;
  uip_ip6addr(&ipaddr, 0xfe80, 0, 0, 0, 0, 0, 0, 0);
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);  
#endif /* NETSTACK_CONF_WITH_IPV6*/

 // printf("Contiki and SPIRIT1 correctly configured... Starting all processes\n");

 // process_start(&sensors_process, NULL);

  autostart_start(autostart_processes);

#if WITH_IP64
  uip_ip4addr_t ipv4addr, netmask, ip4addr;
  uip_ip6addr_t ip6addr;

  uip_ipaddr(&ipv4addr, 192, 168, 0, 1);
  uip_ipaddr(&netmask, 255, 255, 255, 0);
  ip64_set_ipv4_address(&ipv4addr, &netmask);
  printf("IPv4 address %d.%d.%d.%d\n",
  	   ipv4addr.u8[0], ipv4addr.u8[1],
  	   ipv4addr.u8[2], ipv4addr.u8[3]);

  uip_ipaddr(&ip4addr, 8,8,8,8);
  ip64_addr_4to6(&ip4addr, &ip6addr);
  uip_nameserver_update((uip_ipaddr_t *)&ip6addr, UIP_NAMESERVER_INFINITE_LIFETIME);
#endif

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

/** @} */
