/**
 ******************************************************************************
 * @file    led6001.h
 * @author  CL
 * @version V1.0.0
 * @date    30-Sep-2015
 * @brief   This file contains definitions for the led6001.c
 *          firmware driver.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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
#ifndef __LED6001_H
#define __LED6001_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "led_driver.h"

/** @addtogroup BSP
 * @{
 */

/** @addtogroup Components
 * @{
 */

/** @addtogroup LED6001
 * @{
 */

/** @defgroup LED6001_Exported_Defines LED6001_Exported_Defines
 * @{
 */
#ifndef NULL
#define NULL      (void *) 0
#endif


/**
 * @}
 */

/** @defgroup LED6001_Imported_Functions LED6001_Imported_Functions
 * @{
 */
/* LED driver IO functions */
extern LED_DRIVER_StatusTypeDef LED6001_IO_Init(void);
extern void LED6001_IO_DeInit(void);
extern void LED6001_IO_ITConfig( void );
extern void LED6001_SetPdimTimer(uint8_t PwmVal);
extern void LED6001_SetAdimTimer(uint8_t PwmVal);
/**
 * @}
 */

   
#define LED6001_PDIM_FULL       100
#define LED6001_ADIM_FULL       100
   
/** @addtogroup LED6001_Exported_Variables LED6001_Exported_Variables
 * @{
 */
/* LED driver structure */
extern LED_DRIVER_DrvTypeDef LED6001Drv;

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

#ifdef __cplusplus
}
#endif

#endif /* __LED6001_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
