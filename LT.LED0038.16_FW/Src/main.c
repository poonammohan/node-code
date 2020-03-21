/**
  ******************************************************************************
  * File Name          : main.cq3
  * Description        : Main program body
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
#include "stm32f3xx_hal.h"
#include "state.h"
#include "system.h"
#include "led.h"
#include "state.h"
#include "battery.h"
#include "panel.h"
#include "mppt.h"
#include "dc_dc_ld.h"
#include "ac_dc_ld.h"
#include "connectivity.h"

/* Private variables ---------------------------------------------------------*/
extern ThreeSL_StateTypeDef ThreeSL_State;
extern uint16_t BattMaxVoltage;
extern uint16_t BattMinVoltage;
extern uint16_t BattMinDischrgVoltage;
/* Private function prototypes -----------------------------------------------*/
int dc_dc_Led_count = 1;

int main(void)
{
  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock
     Do not change clock frequency settings, it might result in board damage */
  SystemClock_Config();

  /* Check if the system has resumed from WWDG reset */
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) != RESET)
  {
    /* Insert code here */
  }

  /* Clear reset flags in any case */
  __HAL_RCC_CLEAR_RESET_FLAGS();  
  
  /* Check if panel is connected in reverse */
  system_CheckPanelReverse();
  
  /* Initialize required peripherals */
  system_Init();        // All MCC peripherals initialized in system.c
  HAL_Delay(1);         // Delay of 1 ms just it mak sure the intializsations
  /* Set status of LED driver, charge controller and connectivity */    // Initialize the default states for LED, Solar Charge Controller, Connection
  ThreeSL_State.state_LED = LED_STATE_OFF;
  ThreeSL_State.state_SCC = SCC_STATE_START_UP;
  ThreeSL_State.state_Conn = Conn_STATE_ON;
  
  /* Setting the DIM flag so that the defaultly the light will be in dimm condition depending on PIR output the light will turn on Fully */
    //led_SetFlag(LED_MASK_DIM_ENABLE);   //Added By Chinna

  /* Take several values of analog signals to stabilize system at startup */
  for (uint16_t count=0; count<60000; count++)
  {
    system_UpdateSysParams();
  }
    
  /* Calculate constant multiplier for computing battery current from 
     output LED current */
        system_CalcBattCurrK();  

  /* Infinite loop */
  while (1)
  {

    system_UpdateSysParams();
    SM_ThreeSl();    /* Runs the main State Machine */
  }
}


int32_t cmpfunc (const void * a, const void * b)
{
   return ( *(uint32_t*)a - *(uint32_t*)b );
}


#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

}



#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
