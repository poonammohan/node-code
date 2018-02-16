/**
  ******************************************************************************
  * File Name          : panel.c
  * Description        : Functions related to solar panel
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
#include "panel.h"

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
  PANEL_ENABLE,
  PANEL_DISABLE
} PanelEnabeTypeDef;

typedef enum
{
  PANEL_PRESENT,
  PANEL_NOT_PRESENT
} PanelPresentTypeDef;

typedef enum
{
  PANEL_OK,
  NATTERY_NOT_OK
} PanelStatusTypeDef;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t StateFlags_Panel = 0x01;
uint16_t PanelMinVoltage = PANEL_MINIMUM_VOLTAGE;
uint16_t PanelMaxVoltage = PANEL_MAXIMUM_VOLTAGE;
uint16_t PanelMpptVoltage = PANEL_MPPT_VOLTAGE;
uint16_t PanelVoltage;
/* Private function prototypes -----------------------------------------------*/
void Panel_Control(PanelEnabeTypeDef);
PanelPresentTypeDef Panel_Is_Present(void);
PanelStatusTypeDef panel_Is_Ok(void);
/* Exported functions --------------------------------------------------------*/

/**
* @brief This function update panel voltage
*/
void panel_UpdateVoltage(uint32_t* volt)
{
  PanelVoltage = (uint16_t)*volt;
}

/**
* @brief This function return panel minimum voltage threshold
*/
uint16_t panel_GetMinVoltage(void)
{
  return PanelMinVoltage;
}

/**
* @brief This function return existing panel voltage
*/
uint16_t panel_GetVoltage(void)
{
  return PanelVoltage;
}

/**
* @brief This function return maximum panel voltage threshold
*/
uint16_t panel_GetMaxVoltage(void)
{
  return PanelMaxVoltage;
}

/**
* @brief This function return panel mppt voltage threshold
*/
uint16_t panel_GetMpptVoltage(void)
{
  return PanelMpptVoltage;
}

/**
* @brief This function set various panel flags
*/
void panel_SetFlag(uint16_t flag)
{
  StateFlags_Panel = StateFlags_Panel | flag;
}

/**
* @brief This function reset various panel flags
*/
void panel_ResetFlag(uint16_t flag)
{
  StateFlags_Panel = StateFlags_Panel & (~flag);
}

/**
* @brief This function return various panel flags
*/
bool panel_GetFlag(uint16_t flag)
{
  if (StateFlags_Panel & flag)
    return true;
  else
    return false;
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
