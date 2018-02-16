/**
******************************************************************************
* @file    pressure-sensor.c
* @author  System LAB
* @version V1.0.0
* @date    17-June-2015
* @brief   Enable pressure sensor functionality 
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
 * @defgroup pressure_sensor
 * @ingroup dev
 * @{
 */

/**
 * @addtogroup pressure_sensor
 * @ingroup dev
 * @{
 * @file Driver for the Pressure sensor (on expansion board)
 */
/*---------------------------------------------------------------------------*/
#if COMPILE_SENSORS
/*---------------------------------------------------------------------------*/
#include "lib/sensors.h"
#include "pressure-sensor.h"
#include "st-lib.h"
/*---------------------------------------------------------------------------*/
static int _active = 1;
/*---------------------------------------------------------------------------*/
static void init(void)
{
  st_lib_bsp_pressure_init();
  _active =1;
}
/*---------------------------------------------------------------------------*/
static void activate(void)
{
  _active = 1;
}
/*---------------------------------------------------------------------------*/
static void deactivate(void)
{
  _active = 0;
}
/*---------------------------------------------------------------------------*/
static int active(void)
{
  return _active;
}
/*---------------------------------------------------------------------------*/
static int value(int type)
{
  uint16_t pressure;
  volatile float pressure_value;
  
  st_lib_bsp_pressure_get_pressure((float *)&pressure_value);
  pressure = pressure_value * 10;

  return pressure;
}
/*---------------------------------------------------------------------------*/
static int configure(int type, int value)
{
  switch(type) {
    case SENSORS_HW_INIT:
      init();
      return 1;
    case SENSORS_ACTIVE:
      if(value) {      
        activate();
      } else {
        deactivate();
      }
      return 1;
  }
 
  return 0;
}
/*---------------------------------------------------------------------------*/
static int status(int type)
{
  switch(type) {
    case SENSORS_READY:
      return active();
  }
  
  return 0;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(pressure_sensor, PRESSURE_SENSOR, value, configure, status);
/*---------------------------------------------------------------------------*/
#endif /*COMPILE_SENSORS*/
/** @} */
/** @} */
