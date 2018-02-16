/**
******************************************************************************
* @file    main.c 
* @author  Central Labs
* @version V1.0.0
* @date    15-March-2016
* @brief   Entry point of Sub-GHz and BLE vertical application. 
*          User can read the node sensors data on android application.
*          The communication is done using a Nucleo board and a Smartphone  with BTLE.
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
#include "cube_hal.h"

#include "stdbool.h"
#include "MCU_Interface.h"
#include "radio_spi.h" 
#include "clock.h"
#include "process.h"
#include "main.h"
#include "dev/watchdog.h"    
#include "sensor_node.h"
#include "leds.h"
/**
 * @addtogroup USER
 *  @{
 */


/** @addtogroup Sensor_Node_Applications
 * @{
 */

/** @defgroup MAIN 
 * @{
 */

/** @defgroup MAIN_TypesDefinitions
 * @{
 */

/**
  * @}
  */

/** @defgroup MAIN_Private_Defines
  * @{
  */

/**
 * @}
 */
 

/** @defgroup MAIN_Private_Macros
  * @{
  */
/* Private macros ------------------------------------------------------------*/

/**
 * @}
 */
 


/** @defgroup MAIN_Private_Variables
 * @{
 */

/**
 * @}
 */

/** @defgroup MAIN_Private_FunctionPrototypes
 * @{
 */

/**
 * @}
 */

/** @addtogroup Global_variables
*  @{
*/
extern uint8_t received_uart_packet ;
/**
* @}
*/

/** @defgroup MAIN_Private_Functions
  * @{
  */


/**
 * @brief  Entry point of Sub-GHz and BLE vertical application. 
*          User can read the node sensors data on android application.
*          The communication is done using a Nucleo board and a Smartphone with BTLE.
 * @param  None
 * @retval None
 */

int main(void)
{
  /* STM32Cube HAL library initialization:
  *  - Configure the Flash prefetch, Flash preread and Buffer caches
  *  - Systick timer is configured by default as source of time base, but user 
  *    can eventually implement his proper time base source (a general purpose 
  *    timer for example or other time source), keeping in mind that Time base 
  *    duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
  *    handled in milliseconds basis.
  *  - Low Level Initialization
  */
  
  HAL_Init();
  
  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize Contiki and our processes. */
  
  clock_init(); 
  
  HAL_EnableDBGStopMode();
  
  MX_GPIO_Init(); 
  USARTConfig();
  
  /* Initialize RTC */
  RTC_Config();
  RTC_TimeStampConfig();
  
  /* init LEDs */
  leds_init();
  
  /* Initialize Node and sensor structure */
  SN_InitializeNode();  
  
  /* Initialize Contiki */
  Stack_6LoWPAN_Init();
  
  while(1)
  {
    watchdog_periodic();
    process_run(); 
    
    if(received_uart_packet)
    {
      SendPacket_To_BR();
      received_uart_packet = 0;
    } 
    
  } 
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

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
