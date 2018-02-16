/**
 ******************************************************************************
 * @file    led_driver.h
 * @author  CL
 * @version V1.0.0
 * @date    30-Sep-2015
 * @brief   This header file contains the functions prototypes for the
 *          led driver.
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
#ifndef __LED_DRIVER_H
#define __LED_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/** @addtogroup BSP
  * @{
  */

/** @addtogroup Components
  * @{
  */

/** @addtogroup LED_DRIVER
  * @{
  */

/** @defgroup LED_DRIVER_Exported_Types
  * @{
  */

/**
 * @brief  LED_DRIVER init structure definition
 */
typedef struct
{
  uint8_t PdimVal;
  uint8_t AdimVal;  
} LED_DRIVER_InitTypeDef;

/**
 * @brief  LED_DRIVER status enumerator definition
 */
typedef enum
{
  LED_DRIVER_OK = 0,
  LED_DRIVER_ERROR = 1,
  LED_DRIVER_TIMEOUT = 2,
  LED_DRIVER_NOT_IMPLEMENTED = 3
} LED_DRIVER_StatusTypeDef;

/**
 * @brief  LED_DRIVER component id enumerator definition
 */
typedef enum
{
  LED_DRIVER_NONE_COMPONENT = 0,
  LED_DRIVER_LED6001_COMPONENT = 1,
} LED_DRIVER_ComponentTypeDef;

/**
 * @brief  LED_DRIVER driver extended structure definition
 */
typedef struct
{
  LED_DRIVER_ComponentTypeDef
  id; /* This id must be unique for each component belonging to this class that wants to extend common class */
  void *pData; /* This pointer is specific for each component */
} LED6001_DrvExtTypeDef;

/**
 * @brief  LED_DRIVER driver structure definition
 */
typedef struct
{
  LED_DRIVER_StatusTypeDef (*Init)(LED_DRIVER_InitTypeDef *);
  void (*DeInit)(void);
  void (*SetPwmDim)(uint8_t);
  void (*SetAnaDim)(uint8_t);
  LED_DRIVER_StatusTypeDef (*FaultStatus)(void);
  LED6001_DrvExtTypeDef   *extData;
} LED_DRIVER_DrvTypeDef;

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

#endif /* __LED_DRIVER_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
