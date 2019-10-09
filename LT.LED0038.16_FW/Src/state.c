/**
  ******************************************************************************
  * File Name          : state.c
  * Description        : Functions related to state of the board
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
#include "state.h"
#include "system.h"
#include "battery.h"
#include "panel.h"
#include "mppt.h"
#include "dc_dc_ld.h"
#include "ac_dc_ld.h"
#include "led.h"
#include "connectivity.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ThreeSL_StateTypeDef ThreeSL_State;
LED_StateTypeDef CurrentState_LED = LED_STATE_OFF;
LED_StateTypeDef NextState_LED = LED_STATE_OFF;
SCC_StateTypeDef CurrentState_SCC = SCC_STATE_OFF;
SCC_StateTypeDef NextState_SCC = SCC_STATE_OFF;
Conn_StateTypeDef CurrentState_Conn;
Conn_StateTypeDef NextState_Conn;

bool LedStateChange = false;
uint32_t LedStateChangeTick;

uint8_t State_SubSystem;
uint8_t State_SensorStatus;
uint8_t State_BatteryStatus;
uint8_t ledIntensityDefault=1;//Added By Chinna for setting Initial LED Intensity

BatteryChargingModeTypeDef currentBattState = BATTERY_CHARGING_MODE_OFF;
uint16_t countBattLow = 0;

extern UART_HandleTypeDef Sp1mlUart;
extern TIM_HandleTypeDef htm16; //Added By Chinna
extern uint8_t UartRxData[UART_RX_DATABUFF_SIZE];
extern uint8_t UartTxData[UART_TX_DATABUFF_SIZE];
extern uint16_t UartRxDataStartPntr;
extern uint16_t UartRxDataLength;
extern uint32_t LedCurrent;
extern uint16_t DcDcLdOutVoltage;
extern uint32_t LedI_Th ;
extern uint16_t BattMinVoltage;
extern uint16_t BattMaxVoltage;
extern uint16_t BattMinDischrgVoltage;
extern uint8_t PacketRxData[PACKET_LENGTH];
/* Private function prototypes -----------------------------------------------*/
/* Different state functions */
static void state_StartUp_Led(void);
static void state_FacSetting_Led(void);
static void state_On_Led(void);
static void state_Off_Led(void);
static void state_Critical_Led(void);

static void state_StartUp_SCC(void);
static void state_FacSetting_SCC(void);
static void state_BattChg_SCC(void);
static void state_Critical_SCC(void);
static void state_Off_SCC(void);

static void state_StartUp_Conn(void);
static void state_Conn_On(void);
/* This table contains a pointer to the function to call in each
state */
static void (*StateTable_LED[5])(void) =
{
  state_FacSetting_Led,
  state_Off_Led,
  state_StartUp_Led,
  state_On_Led,
  state_Critical_Led
};

static void (*StateTable_SCC[5])(void) =
{
  state_FacSetting_SCC,
  state_Off_SCC,
  state_StartUp_SCC,
  state_BattChg_SCC,  
  state_Critical_SCC,
};

static void (*StateTable_Conn[2])(void) = 
{
  state_StartUp_Conn,
  state_Conn_On
};
/* Exported functions --------------------------------------------------------*/
uint32_t DIMM_ZERO = 0;
uint32_t DIMM_ONE = 0.4*LED_I_MAX;      // In the 25% Dimm level the LED current is same as Offset current.
uint32_t DIMM_TWO = 0.5*LED_I_MAX;
uint32_t DIMM_THREE = 0.75*LED_I_MAX;
uint32_t DIMM_FOUR = LED_I_MAX;

/**
* @brief This function initialized LEDs configuration to factory settings
*/
void state_FacSetting_Led(void)
{
  /* To be implemented */
}

/**
* @brief This function maintains LED off state
*/
void state_Off_Led(void)
{  
  /* Reset battery low check counter */
  countBattLow = 0;
  LedStateChange = false;
  
//  system_Monitor();
  system_DcDcLdOutDisable();
  system_AcDcLdOutDisable();
  //if(PacketRxData[3]>100 | PacketRxData[3]<=150)
 // {
   // ThreeSL_State.state_LED = LED_STATE_START_UP;
  //}
}

/**
* @brief This function corresponds to LED startup phase
*/
void state_StartUp_Led(void)
{
//  system_Monitor();
  
  /* Preference is to DC-DC over AC-DC */
  if ((battery_GetFlag(BATTERY_MASK_OK)) && 
      (!(battery_GetFlag(BATTERY_MASK_DSCHRG))))
  {
    system_DcDcLdOutEnable();
    dc_dc_ld_SetFlag(DC_DC_LD_MASK_ENABLE);
    ac_dc_ld_ResetFlag(AC_DC_LD_MASK_ENABLE);
    battery_ResetFlag(BATTERY_MASK_CHRGD);
  }
  else
  {
    system_DcDcLdOutDisable();
     system_AcDcLdEnable();
    system_AcDcLdOutEnable();
    dc_dc_ld_ResetFlag(DC_DC_LD_MASK_ENABLE);
    ac_dc_ld_SetFlag(AC_DC_LD_MASK_ENABLE);
  }
  
  LedStateChange = true;
  /* LEDs state has been changed and current tick is saved */
  LedStateChangeTick = HAL_GetTick();
  /* Ignore LED current monitoring in startup, to avoid false triggering of
     output short circuit protection */
  system_SetIgnoreLedCurrent(true);  
  GPIOB->BSRR = (uint32_t)GPIO_PIN_5;//Added by Sunil
  
  ThreeSL_State.state_LED = LED_STATE_ON;
}

/**
* @brief This function maintains LED on state
*/
void state_On_Led(void)
{ 
  static bool prevLedDimFlag = false;
  static bool currLedDimFlag = false;
  static bool changeDim = false;
  static uint32_t changeDimTick;  
  
  currLedDimFlag = led_GetFlag(LED_MASK_DIM_ENABLE);
  if (currLedDimFlag != prevLedDimFlag)
  {
    changeDim = true;
    changeDimTick = HAL_GetTick();
  }
  prevLedDimFlag = currLedDimFlag;
  
  if ((changeDim == true) && (HAL_GetTick() == (changeDimTick + 500)))
  {
    changeDim = false;
    
    if (led_GetFlag(LED_MASK_DIM_ENABLE))
    {
 //////////////Added by Sunil to check brightness////////////////////
#if 0
          if (conn_GetPacketData(ON_OFF_INDEX) >= 0x01 && \
            conn_GetPacketData(ON_OFF_INDEX) <= 0x19) //25% bright
          { 
            led_SetThreshI(85+((LED_I_MAX-85)/4));
          }   
          else if (conn_GetPacketData(ON_OFF_INDEX) >= 0x1A && \
            conn_GetPacketData(ON_OFF_INDEX) <= 0x32)//50% bright
          { 
            led_SetThreshI(85+((LED_I_MAX-85)/2));
          }
          else                                       //75% bright
          { 
            led_SetThreshI(85+(((LED_I_MAX-85)*3)/4));
          }   
          GPIOB->BSRR = (uint32_t)GPIO_PIN_5;
    //      led_ResetFlag(LED_MASK_DIM_ENABLE);
 //         state_StartUp_Led();
#endif
//////////////Added by Sunil to check brightness////////////////////
      //led_SetThreshI((uint32_t)(0.5*LED_I_MAX));//DC DC originally was there
     // led_ResetFlag(LED_MASK_DIM_ENABLE);
          //led_SetThreshI((uint32_t)(0.5*LED_I_MAX));// Added By Chinna
      if (ac_dc_ld_GetFlag(AC_DC_LD_MASK_ENABLE))//AC DC
      {
        system_AcDcSetDim(20);
      }
      else
      {
        system_AcDcSetDim(2);//Idle value
      }
    }
    else
    {
      //led_SetThreshI(DIMM_FOUR);//DC DC
      if (ac_dc_ld_GetFlag(AC_DC_LD_MASK_ENABLE))//AC DC
      {
        system_AcDcSetDim(2);//Idle value
      }
    }  
  }
  
  /* Wait for 100 ms before starting switching and current regulation loop */
  if (LedStateChange == true)
  {
    if ((HAL_GetTick() - LedStateChangeTick) == 100)
    {
      if (!(dc_dc_ld_GetFlag(DC_DC_LD_MASK_OVER_CURRENT)))
      {
        /* Start LED current regulation */
        system_DcDcStartRegulate();
        /* Ignore LED current monitoring in startup, to avoid false triggering
           of output short circuit protection */ 
        system_SetIgnoreLedCurrent(true);
      }
    }
    else if ((HAL_GetTick() - LedStateChangeTick) > 500)
    {
      LedStateChange = false;
      /* Start monitoring LED current for protection */
      system_SetIgnoreLedCurrent(false);      
    }
  }
  
/*
  if(dc_dc_ld_GetFlag(DC_DC_LD_MASK_ENABLE))
  {
    if(dc_dc_Led_count == 1)
    {
        led_SetThreshI((uint32_t)(0.25*LED_I_MAX));
        conn_SetPacketData(ON_OFF_INDEX,25);
        dc_dc_Led_count++;
    }
      system_CalcBattDischgCurrent();
  } */
  /* While LED is on, check for battery voltage (compensated with cable drop) */
  if ((battery_GetVoltage() + (((system_CalcBattDischgCurrent())*
                                BATTERY_DISCHRG_VOLT_OFFSET_PA)/10)) < 
                                battery_GetMinDischrgVoltage())
  {
    /* If battery voltage is low, check 100 times at least to decide */
    if (system_GetBattTick_10() == true)
    {
      system_SetBattTick_10(false);
      countBattLow++;
      if (countBattLow == 500)
      {
        battery_SetFlag(BATTERY_MASK_DSCHRG);
        system_DcDcLdOutDisable();
         system_AcDcLdEnable();
        system_AcDcLdOutEnable();         
        battery_ResetFlag(BATTERY_MASK_OK);
        dc_dc_ld_ResetFlag(DC_DC_LD_MASK_ENABLE);
        ac_dc_ld_SetFlag(AC_DC_LD_MASK_ENABLE);
        led_SetThreshI((uint32_t)(LED_I_MAX));
        State_SubSystem = State_SubSystem | 0x02;
        State_SubSystem = State_SubSystem & 0xFE;
         if(conn_GetPacketData(ON_OFF_INDEX) > 100 && conn_GetPacketData(ON_OFF_INDEX)<=150)
       {
         conn_SetPacketData(ON_OFF_INDEX,150 | 0x80);
       }
         else {
        conn_SetPacketData(ON_OFF_INDEX,100);
         }
      }
    }
  }
  
  /* Monitor for output over voltage */
  if (dc_dc_ld_GetFlag(DC_DC_LD_MASK_OVER_VOLTAGE))
  {
    system_DcDcLdOutDisable();
    system_AcDcLdOutEnable();    
    dc_dc_ld_ResetFlag(DC_DC_LD_MASK_ENABLE);
    ac_dc_ld_SetFlag(AC_DC_LD_MASK_ENABLE);
  }
  
  /* Monitor for output over current */
  if (dc_dc_ld_GetFlag(DC_DC_LD_MASK_OVER_CURRENT))
  {
    system_DcDcLdOutDisable();
    system_AcDcLdOutDisable();
    dc_dc_ld_ResetFlag(DC_DC_LD_MASK_ENABLE);
    ac_dc_ld_ResetFlag(AC_DC_LD_MASK_ENABLE);
    
    ThreeSL_State.state_LED = LED_STATE_CRITICAL;
  }
}
    
/**
* @brief This function handles LED critical state
*/
void state_Critical_Led(void)
{
  system_DcDcLdOutDisable();
  system_AcDcLdOutDisable();
  dc_dc_ld_ResetFlag(DC_DC_LD_MASK_ENABLE);
  ac_dc_ld_ResetFlag(AC_DC_LD_MASK_ENABLE);
  
  dc_dc_ld_ResetFlag(DC_DC_LD_MASK_OVER_VOLTAGE);
  dc_dc_ld_ResetFlag(DC_DC_LD_MASK_OVER_CURRENT);
    
  led_ResetFlag(LED_MASK_ENABLE);
  ThreeSL_State.state_LED = LED_STATE_OFF;
}


/**
* @brief  State for updating the factory settings
*/
static void state_FacSetting_SCC()
{  
  ThreeSL_State.state_SCC = SCC_STATE_BATT_CHG;
}

/**
* @brief This function maintains SCC off state
*/
void state_Off_SCC(void)
{
  system_SetSccPwm(0);
  system_DisableCharging();
}

/**
* @brief  SCC state for the start up
*/    
static void state_StartUp_SCC()
{
  /* Clear all battery flags */
  
  system_PanelEnable();

  /* Check panel status */
  currentBattState = BATTERY_CHARGING_MODE_BULK; 
  mppt_Reinit();
  system_SetSccPwm(0);
    
  ThreeSL_State.state_SCC = SCC_STATE_BATT_CHG;
}

/**
* @brief  This function charge battery from panel depending on battery and panel
          state
*/
static void state_BattChg_SCC()
{
  static uint16_t pwmDuty;

  /* Enters below routine every 10ms */
  if (system_GetBattTick_10() == true)
  {
    system_SetBattTick_10(false);

    if ((panel_GetFlag(PANEL_MASK_OK)))
    {
      system_EnableCharging();
      
      
       if(conn_GetPacketData(ON_OFF_INDEX) > 160 && conn_GetPacketData(ON_OFF_INDEX) <=170)
       {
                 pwmDuty = mppt_Routine((battery_GetMaxVoltage()),
                               battery_GetMaxCurrent(),
                               battery_GetVoltage(),battery_GetChargngCurrent(),//battery_GetChargngCurrent()
                               system_GetSccPwm());
                   //currentBattState = BATTERY_CHARGING_MODE_BULK;
       }
       else if(conn_GetPacketData(ON_OFF_INDEX) > 170 && conn_GetPacketData(ON_OFF_INDEX) <=180)
       {
                  pwmDuty = mppt_Routine(battery_GetFltVoltage(),battery_GetFltCurrent(),
                               battery_GetVoltage(),battery_GetChargngCurrent(),
                               system_GetSccPwm());
                  //currentBattState = BATTERY_CHARGING_MODE_FLOAT;
         
       }
      else if (currentBattState == BATTERY_CHARGING_MODE_FLOAT)
      {
        battery_ResetFlag(BATTERY_MASK_DEEP_DSCHRG);
        battery_ResetFlag(BATTERY_MASK_DSCHRG); 
        
        /* If battery voltage is near to battery float voltage */
          /* In float mode, if current is more than float threshold current */
          if (battery_GetChargngCurrent() > (BATTERY_FLOAT_CURRENT_TH + 
                                             BATTERY_FLOAT_CURRENT_OFFSET))
          {
            battery_ResetFlag(BATTERY_MASK_DEEP_DSCHRG);
            battery_ResetFlag(BATTERY_MASK_CHRGD);
            /* Set bulk mode*/
            currentBattState = BATTERY_CHARGING_MODE_BULK;              
          }

        pwmDuty = mppt_Routine(battery_GetFltVoltage(),battery_GetFltCurrent(),
                               battery_GetVoltage(),battery_GetChargngCurrent(),
                               system_GetSccPwm());
      }
      
      else if (currentBattState == BATTERY_CHARGING_MODE_BULK)
      {
        if (battery_GetChargngCurrent() < battery_GetFltCurrent())
        {
          /* Set float mode*/
          currentBattState = BATTERY_CHARGING_MODE_FLOAT;
          battery_SetFlag(BATTERY_MASK_CHRGD);
        } 
        
        pwmDuty = mppt_Routine((battery_GetMaxVoltage()),
                               battery_GetMaxCurrent(),
                               battery_GetVoltage(),battery_GetChargngCurrent(),//battery_GetChargngCurrent()
                               system_GetSccPwm());
      }
      
      /* Check if battery is not charged and Vpanel is less than
      Vmppt by >1 V or mppt duty cycle is less than 65% of maximum duty cycle 
      for some time, reset MPPT (it implies MPPT is stuck, need 
      to reinitialize) */
      if (system_GetBattTick_1000s() == true)
      {
        system_SetBattTick_1000s(false);
        
        if (((panel_GetVoltage() + 180) < panel_GetMpptVoltage()) || 
              (pwmDuty <= MPPT_PWM_DUTY_RESET))          
        {
          mppt_Reinit();
        }
      }
      
      system_SetSccPwm(pwmDuty);
    }
    else
    {
      currentBattState = BATTERY_CHARGING_MODE_OFF;
      ThreeSL_State.state_SCC = SCC_STATE_CRITICAL;
      system_SetSccPwm(0);
      system_DisableCharging();
      mppt_Reinit();
    }
  }
}


/**
* @brief  State for Critical Condition(Fault Conditions)
*/
static void state_Critical_SCC()
{
  system_PanelDisable();
  
  system_DisableCharging();
  
  /* Check for Change in state from Critical state */  
  ThreeSL_State.state_SCC = SCC_STATE_START_UP;
}

/**
* @brief This function maintains startup phase of connectivity
*/
void state_StartUp_Conn(void)
{
  system_ResetConn();
  ThreeSL_State.state_Conn = Conn_STATE_ON;
}

/**
* @brief This function decipher received commands from DCU and transmit board
        status
*/
void state_Conn_On(void)
{
  if (system_GetUartWaitTick() >= 300000)//5 min
  {
    ThreeSL_State.state_Conn = Conn_STATE_START_UP;
  }
  
  conn_PacketHandler();
  
  if (conn_GetReadPacketDataStatus() == true)
  {
    system_Monitor();
    system_ResetUartTick();
    
    conn_SetReadPacketDataStatus(false);
    
    if (conn_GetPacketData(COMMAND_CODE_INDEX) == COMMAND_ON_OFF)
    {
      
       // system_Monitor();
      
      if (conn_GetPacketData(ON_OFF_INDEX) == 0)
      {
        /* LED enable/disable is also dependent on panel voltage */
        led_SetThreshI(0);
        system_SetServerLightCommand(false);
        led_ResetFlag(LED_MASK_DIM_ENABLE);//--->originally present but commented by Sunil
        led_ResetFlag(LED_MASK_ENABLE);
        system_DcDcLdOutDisable();//------------------>Added by Sunil
        system_AcDcLdOutDisable();//------------------>Added by Sunil
 //       system_AcDcLdDisable();//------------------>Added by Sunil
        ac_dc_ld_ResetFlag(AC_DC_LD_MASK_ENABLE);//------------------>Added by Sunil
        dc_dc_ld_ResetFlag(DC_DC_LD_MASK_ENABLE);//------------------>Added by Sunil
        /* On board LED places at test button */
         GPIOB->BRR = (uint32_t)GPIO_PIN_5;//BRR means switching off test LED
 //    }   
     //   HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);//Added by Sunil
         
         conn_SetPacketData(ON_OFF_INDEX, 0x00 | 0x80);      //Added By Chinna for User Action
         
         State_SubSystem = State_SubSystem & 0xFC;      //Added By Chinna 
         //State_SubSystem = State_SubSystem | 0x01;       //Added By Chinna 
    if (panel_GetFlag(PANEL_MASK_OK))
      { 
      /* Set status of LED driver, charge controller and connectivity */
        ThreeSL_State.state_LED = LED_STATE_OFF;
        ThreeSL_State.state_SCC = SCC_STATE_START_UP;
      }
      
      }
      else
      {
        /* LED enable/disable is also dependent on panel voltage */      
       // system_SetServerLightCommand(true);
        //system_SetServerLightDimCommand(true);
        if (conn_GetPacketData(ON_OFF_INDEX) < 100)
        {
          system_SetServerLightDimCommand(true);
        //  system_DcDcLdOutEnable();//----->Added by Sunil
          led_SetFlag(LED_MASK_ENABLE);
          led_SetFlag(LED_MASK_DIM_ENABLE);
 //////////////Added by Sunil to check brightness////////////////////
//#if 1
          if (conn_GetPacketData(ON_OFF_INDEX) > 0x01 && conn_GetPacketData(ON_OFF_INDEX) <= 0x19) //25% bright
          { 
            if(dc_dc_ld_GetFlag(DC_DC_LD_MASK_ENABLE))
            {
            led_SetThreshI(DIMM_ONE);
            //led_SetThreshI(85+((LED_I_MAX-85)/4));
              conn_SetPacketData(ON_OFF_INDEX, 25 | 0x80);     //Added By Chinna for User Action
            }
              if(ac_dc_ld_GetFlag(AC_DC_LD_MASK_ENABLE))
              {
                system_AcDcSetDim(50);
              }
          }   
          else if (conn_GetPacketData(ON_OFF_INDEX) >= 0x1A && conn_GetPacketData(ON_OFF_INDEX) <= 0x32)//50% bright
          { 
            led_SetThreshI(DIMM_TWO);
            system_AcDcSetDim(20);
            //led_SetThreshI(85+((LED_I_MAX-85)/2));
              conn_SetPacketData(ON_OFF_INDEX, 50 | 0x80);     //Added By Chinna for User Action            
          }
          else if (conn_GetPacketData(ON_OFF_INDEX) >= 0x33 && conn_GetPacketData(ON_OFF_INDEX) <= 0x4B)                                    //75% bright
          { 
            led_SetThreshI(DIMM_THREE);
            //led_SetThreshI(85+(((LED_I_MAX-85)*3)/4));
              conn_SetPacketData(ON_OFF_INDEX, 75 | 0x80);     //Added By Chinna for User Action
          }   
          //GPIOB->BSRR = (uint32_t)GPIO_PIN_5; //Commented By Chinna
          //led_ResetFlag(LED_MASK_DIM_ENABLE);//Commented By Chinna
 //         state_StartUp_Led();
//#endif
//////////////Added by Sunil to check brightness////////////////////
          
        }
        else if(conn_GetPacketData(ON_OFF_INDEX) ==100)
        {
          system_SetServerLightCommand(true);
           system_SetServerLightDimCommand(false);
          led_SetThreshI(DIMM_FOUR);
          //system_Monitor();
          
            //ThreeSL_State.state_LED == LED_STATE_OFF;
             //ThreeSL_State.state_SCC = SCC_STATE_BATT_CHG;          
         conn_SetPacketData(ON_OFF_INDEX, 100 | 0x80);  //Added By Chinna for User Action
         led_SetFlag(LED_MASK_ENABLE);
        led_ResetFlag(LED_MASK_DIM_ENABLE);//--->originally was there but commented by sunil

        GPIOB->BSRR = (uint32_t)GPIO_PIN_5;     //Added By Chinns

        state_StartUp_Led();//---------->    // Added by Sunil
 /////////////////////Added by Sunil////////////////////////////////////////////////// 
                  }
          //**************** Switching from Battery to AC ************//
       if(conn_GetPacketData(ON_OFF_INDEX) > 100 && conn_GetPacketData(ON_OFF_INDEX)<=110)
       {
        BattMinVoltage = BATTERY_AC_DC_SWITCH_HIGH_CUT_OFF;
        BattMaxVoltage = BATTERY_AC_DC_SWITCH_LOW_CUT_OFF;
        BattMinDischrgVoltage = BATTERY_AC_DC_SWITCH_MIN_DISCHRG_VOLT;
        //ThreeSL_State.state_LED = LED_STATE_START_UP;
        conn_SetPacketData(ON_OFF_INDEX, 150 | 0x80);
        //battery_ResetFlag(BATTERY_MASK_OK);
       // battery_SetFlag(BATTERY_MASK_DSCHRG);
       // system_Battery_Disable();
       // HAL_Delay(500);
        //system_DcDcLdOutDisable();
        //system_AcDcLdOutEnable();//Added By Chinna For Switching Battery to AC
        //ac_dc_ld_SetFlag(AC_DC_LD_MASK_ENABLE);//Added By Chinna For Switching Battery to AC
        //dc_dc_ld_ResetFlag(DC_DC_LD_MASK_ENABLE);//Added By Chinna For Switching Battery to AC
       //State_SubSystem = State_SubSystem | 0x02;//Added By Chinna For Switching Battery to AC
      // State_SubSystem = State_SubSystem & 0xFE;//Added By Chinna For Switching Battery to AC
        
       }
        //**************** Switching from Battery to AC ************//

       
          //**************** Switching from Battery to AC ************//
       if(conn_GetPacketData(ON_OFF_INDEX) > 110 && conn_GetPacketData(ON_OFF_INDEX) <=120)
       {
         BattMinVoltage = BATTERY_MINIMUM_VOLTAGE;
         BattMaxVoltage = BATTERY_MAXIMUM_VOLTAGE;
         BattMinDischrgVoltage = BATTERY_MIN_DISCHRG_VOLT;
         ThreeSL_State.state_LED = LED_STATE_START_UP;
         conn_SetPacketData(ON_OFF_INDEX, 0 | 0x80);
         HAL_Delay(10000);
        //system_DcDcLdOutEnable();
        //system_AcDcLdOutDisable();//Added By Chinna For Switching Battery to AC
        //ac_dc_ld_ResetFlag(AC_DC_LD_MASK_ENABLE);//Added By Chinna For Switching Battery to AC
       // dc_dc_ld_SetFlag(DC_DC_LD_MASK_ENABLE);//Added By Chinna For Switching Battery to AC
       //State_SubSystem = State_SubSystem | 0x01;//Added By Chinna For Switching Battery to AC
       //State_SubSystem = State_SubSystem & 0xFE;//Added By Chinna For Switching Battery to AC
        
       }

        
        /* On board LED places at test button */
        //GPIOB->BSRR = (uint32_t)GPIO_PIN_5;//
      }
      
       //**************** Buzzer Integration with Cloud************//       
        if(conn_GetPacketData(ON_OFF_INDEX) > 120 && conn_GetPacketData(ON_OFF_INDEX) <=130)        //added By Chinna
       {
         //system_SetServerLightCommand(false);
         HAL_TIM_PWM_Start(&htm16,TIM_CHANNEL_1);
         //HAL_Delay(1000);
         //HAL_TIM_PWM_Stop(&htm16,TIM_CHANNEL_1);
       }
       else if(conn_GetPacketData(ON_OFF_INDEX) > 140 && conn_GetPacketData(ON_OFF_INDEX) <=150)        //added By Chinna
       {
         //system_SetServerLightCommand(false);
       HAL_TIM_PWM_Stop(&htm16,TIM_CHANNEL_1);
       }
        else if(conn_GetPacketData(ON_OFF_INDEX) > 150 && conn_GetPacketData(ON_OFF_INDEX) <=160)        //added By Chinna
       {
          
         ThreeSL_State.state_SCC = SCC_STATE_OFF;
       }
    }    
    
    /********************* Set board status ****************************/
    else // command is send status
    {
      conn_SetPacketData(COMMAND_CODE_INDEX, COMMAND_SEND_STATUS);
      
        system_Monitor();//Added By Chinna
        /*
       if(!((conn_GetPacketData(ON_OFF_INDEX))&0x80))
       {
        if (panel_GetFlag(PANEL_MASK_OK))
        {
          if(ThreeSL_State.state_LED = LED_STATE_OFF)
          {
          conn_SetPacketData(ON_OFF_INDEX,0);
          }
        }
        else if (led_GetFlag(LED_MASK_ENABLE))
        {
        if(LedI_Th==DIMM_ZERO)
          {
            conn_SetPacketData(ON_OFF_INDEX,0);
          }
        if(LedI_Th==DIMM_ONE)
        {
          conn_SetPacketData(ON_OFF_INDEX,25);
        }
        else if(LedI_Th==DIMM_TWO)
          
        {
          conn_SetPacketData(ON_OFF_INDEX,50);
        }
        else if(LedI_Th==DIMM_THREE)
        {
          conn_SetPacketData(ON_OFF_INDEX,75);
        }
        else if(LedI_Th==DIMM_FOUR)
        {
          conn_SetPacketData(ON_OFF_INDEX,100);
        }
         }
        }
/*
      /* Set status for DC DC LED driver */
      if (dc_dc_ld_GetFlag(DC_DC_LD_MASK_OVER_VOLTAGE) || 
          dc_dc_ld_GetFlag(DC_DC_LD_MASK_OVER_CURRENT))
      {
        State_SubSystem = State_SubSystem | 0x80;
      }
      else
      {
        State_SubSystem = State_SubSystem & 0x7F;
      }
      
      /* Set status for panel fault (panel reverse) */
      if (panel_GetFlag(PANEL_MASK_REVERSE))
      {
        State_SubSystem = State_SubSystem | 0x20;
      }
      else
      {
        State_SubSystem = State_SubSystem & 0xDF;
      }  
#if 0
      /* Set status for AC-DC driver fault */
      if (ac_dc_ld_GetFlag(AC_DC_LD_MASK_ENABLE))
      {
        State_SubSystem = State_SubSystem & 0xEF;
      } 
      else
      {
        State_SubSystem = State_SubSystem  | 0x10;
      } 
#endif      
      /* Set status for solar availability */
      if (panel_GetFlag(PANEL_MASK_OK))
      {
        State_SubSystem = State_SubSystem | 0x08;
      }
      else
      {
        State_SubSystem = State_SubSystem & 0xF7;
      }      
      /* Set status for current source for LED */
      if (led_GetFlag(LED_MASK_ENABLE))
      {
        if(battery_GetFlag(BATTERY_MASK_OK))
        {
        
        if (dc_dc_ld_GetFlag(DC_DC_LD_MASK_ENABLE))
        {
          State_SubSystem = State_SubSystem & 0xFD;
          State_SubSystem = State_SubSystem | 0x01;
        }
        }
        else if(ac_dc_ld_GetFlag(AC_DC_LD_MASK_ENABLE))
          {
          State_SubSystem = State_SubSystem | 0x02;
          State_SubSystem = State_SubSystem & 0xFE;
          }
      }
      else
      {
        State_SubSystem = State_SubSystem & 0xFC;
      }
      
      conn_SetPacketData(SUB_SYSTEM_INDEX, State_SubSystem);
      
      /* Set status for over temperature */
      if (system_GetTemperature() > OVER_TEMPERATURE_LIMIT)
      {
        State_SensorStatus = State_SensorStatus | 0x80;
      }
      else
      {
        State_SensorStatus = State_SensorStatus & 0x7F;
      }
      
      conn_SetPacketData(SENSOR_STATUS_INDEX, State_SensorStatus);
      
      /* Set status for battery */
      if (battery_GetFlag(BATTERY_MASK_OK))
      {
        State_BatteryStatus = State_BatteryStatus | 0x01;
      }
      else
      {
        State_BatteryStatus = State_BatteryStatus & 0xFE;
      }
      
      /* Set status for battery charging */
      if ((CurrentState_SCC == SCC_STATE_BATT_CHG)&&(panel_GetFlag(PANEL_MASK_OK))&&CurrentState_LED == LED_STATE_OFF)
      {
        State_BatteryStatus = State_BatteryStatus & 0xDF;
        State_BatteryStatus = State_BatteryStatus | 0x10;
      }
      else
      {
        if ((CurrentState_LED == LED_STATE_ON) && dc_dc_ld_GetFlag(DC_DC_LD_MASK_ENABLE))
        {
          State_BatteryStatus = State_BatteryStatus | 0x20;
          State_BatteryStatus = State_BatteryStatus & 0xEF;
        }
        else
        {
          State_BatteryStatus = State_BatteryStatus & 0xCF;
        }
      }
      
      conn_SetPacketData(BATTERY_STATUS_INDEX, State_BatteryStatus);       
     
      /************Added By Chinna *******************/
      /*********** Sending the LED Current & LED Voltage ***************/
      if (led_GetFlag(LED_MASK_ENABLE))
      {
        if (panel_GetFlag(PANEL_MASK_OK))
      {
        conn_SetPacketData(LED_CURRENT_INDEX,0);
        conn_SetPacketData(LED_VOLTAGE_INDEX,0);
      }
        if(dc_dc_ld_GetFlag(DC_DC_LD_MASK_ENABLE)|ac_dc_ld_GetFlag(AC_DC_LD_MASK_ENABLE))
        {
        conn_SetPacketData(LED_CURRENT_INDEX,0.1*LedCurrent);
      conn_SetPacketData(LED_VOLTAGE_INDEX,0.0129*DcDcLdOutVoltage);
        }
      }
      else if((panel_GetFlag(PANEL_MASK_OK))&&CurrentState_LED == LED_STATE_OFF)
      {
        conn_SetPacketData(LED_CURRENT_INDEX,0);
        conn_SetPacketData(LED_VOLTAGE_INDEX,0);
      }
      /*********** Added By Chinna ******************/
      
      
      /* Set battery current */
      if (ThreeSL_State.state_LED == LED_STATE_OFF)
      {
        /* -ve current while charging, calculation is based on MCU ADC voltage,
        battery current sense resistor, amplifier gain, ADC resolution.
        Value calculated is 10 times of actual current */
        conn_SetPacketData(BATTERY_CURRENT_INDEX, (uint8_t)((((battery_GetChargngCurrent())* 467)/10000) | 0x80));
      }
      else
      {
        if(led_GetFlag(LED_MASK_ENABLE))
        {
          if(dc_dc_ld_GetFlag(DC_DC_LD_MASK_ENABLE))
          {
        /* 10 times of aprroximate current */
        conn_SetPacketData(BATTERY_CURRENT_INDEX, system_CalcBattDischgCurrent());
          }
        }
      }
      
      /* Set battery voltage 
      10 times of actual value */
      conn_SetPacketData(BATTERY_VOLTAGE_INDEX, (uint8_t)(battery_GetVoltage()*0.052));   
      
      /* Set panel current 
      10 times of actual value. It depends on battery charging current,
      battery voltage, panel voltage along with resistor dividers ratios and 
      other parameters */
      if (ThreeSL_State.state_LED == LED_STATE_OFF)
      {
        conn_SetPacketData(PANEL_CURRENT_INDEX, (uint8_t)(((battery_GetVoltage()*
                                         battery_GetChargngCurrent())/
                                        panel_GetVoltage())*0.0475));
        conn_SetPacketData(PANEL_VOLTAGE_INDEX, (uint8_t)(panel_GetVoltage()*0.057));
         //HAL_TIM_PWM_Start(&htm16,TIM_CHANNEL_1);
         //HAL_Delay(20000);
         //HAL_TIM_PWM_Stop(&htm16,TIM_CHANNEL_1);
      }
      else
      {
        /* zero panel current while LED is on */
        conn_SetPacketData(PANEL_CURRENT_INDEX, 0);
        conn_SetPacketData(PANEL_VOLTAGE_INDEX, 0);

      }
      
      /* Set for panel voltage */      
      //conn_SetPacketData(PANEL_VOLTAGE_INDEX, (uint8_t)(panel_GetVoltage()*0.057));
      
      conn_SetSendPacketStatus(true);
    }
  }
}

uint32_t DimVar=0;
/**
* @brief This function run main state machine for required board functions
*/
void SM_ThreeSl(void)
{ 
  static bool prevLedOnFlag = false;
  static bool currLedOnFlag = false;
  static bool changeState = false;
  static uint32_t changeStateTick;
  
  StateTable_LED[CurrentState_LED]();
  StateTable_SCC[CurrentState_SCC]();
  StateTable_Conn[CurrentState_Conn]();
  /* LED enable/disable by detection On from DCU or panel detection  */
  if (system_GetServerLightCommand() == true)
  {
    led_SetFlag(LED_MASK_ENABLE);
  }
  else
  {
    if (!(panel_GetFlag(PANEL_MASK_OK)))
    {
      led_SetFlag(LED_MASK_ENABLE);
    }
    else if(panel_GetFlag(PANEL_MASK_OK))
    {
      led_ResetFlag(LED_MASK_ENABLE);
    }
  }
  /* Toggle between battery charging and LED enable */
  currLedOnFlag = led_GetFlag(LED_MASK_ENABLE);
  if (currLedOnFlag != prevLedOnFlag)
  {
    ThreeSL_State.state_SCC = SCC_STATE_OFF;
    ThreeSL_State.state_LED = LED_STATE_OFF;  
    
    changeState = true;
    changeStateTick = HAL_GetTick();
  }
  prevLedOnFlag = currLedOnFlag;
  
  /* Change state when change state is true and time is away by 100ms */
  if ((changeState == true) && (HAL_GetTick() == (changeStateTick + 100)))
  {
    changeState = false;
    
    if (currLedOnFlag)
    {
      ThreeSL_State.state_LED = LED_STATE_START_UP;
    }
    else
    {
      ThreeSL_State.state_SCC = SCC_STATE_START_UP;
    }
  }
  
  if (CurrentState_LED == LED_STATE_ON)
  {
    /* LED dimming enable/disable by detection from MCU and PIR
    If diiming on from DCU -> dimming enable
     else dimming is dependent on PIR input */
    if (system_GetServerLightDimCommand() == true)
    {
      led_SetFlag(LED_MASK_DIM_ENABLE);      
    }
#if 0
    else
    {
      if((GPIOA->IDR & GPIO_PIN_10) != (uint32_t)GPIO_PIN_RESET)
      {
        led_ResetFlag(LED_MASK_DIM_ENABLE);
      }
      else
      {
        led_SetFlag(LED_MASK_DIM_ENABLE);
      }
    }    
#endif
#if 0
    /*** Added By Chinna ***/
             if( HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) != SET)
        {
          //led_ResetFlag(LED_MASK_ENABLE);
          led_SetFlag(LED_MASK_DIM_ENABLE);
        }
    /*** Added By Chinna ***/
#endif
  }
  
  system_Monitor();
  
  /* If battery voltage is very low, enable AC-DC LED driver to provide aux
     supply voltage */
  if(panel_GetFlag(PANEL_MASK_OK))
  {
        system_AcDcLdDisable();
        system_AcDcLdOutDisable();
  }
  if(CurrentState_LED == LED_STATE_OFF)
  {
      conn_SetPacketData(ON_OFF_INDEX,0);
  }
  /*
  if(led_GetFlag(LED_MASK_ENABLE))
  {
  if (!battery_GetFlag(BATTERY_MASK_OK))  
  {
    system_AcDcLdEnable();
  }
  // If battery voltage is ok and AC DC output to LED is not enabled, disable
     AC-DC LED driver //
  else if (!ac_dc_ld_GetFlag(AC_DC_LD_MASK_ENABLE))
  {
    system_AcDcLdDisable();
  }
  }*/
    
  CurrentState_LED = ThreeSL_State.state_LED;
  CurrentState_SCC = ThreeSL_State.state_SCC;
  CurrentState_Conn = ThreeSL_State.state_Conn;
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
