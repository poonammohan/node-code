/**
******************************************************************************
* File Name          : system.c
* Description        : System configuration, monitoring functions
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
#include "configuration.h"
#include <stdlib.h>
#include <string.h>
#include "led.h"


extern uint8_t Rx_Buff[20];
char string[3];
char var;
extern bool configFlag;
extern uint32_t LedI_Th;
int num;

//Led parameters
extern uint16_t BattMinVoltage;
extern uint16_t BattMinDischrgVoltage;
extern uint16_t BattFltVoltage; //this will always be 13.8V
extern uint16_t BattMaxVoltage;
extern uint16_t BattMaxChargingCurrent; // not using in the firmware
extern uint16_t BattDeepDischrgTh;
uint16_t PanelMaxVolt;
uint16_t PanelMinVolt;
uint16_t PanelMPPTVoltage;

#ifdef TEST_AUTOMATION
void setConfigParams(void){
  if(configFlag==1){
    sprintf(string,"%d%d", Rx_Buff[0]-'0',Rx_Buff[1]-'0'); 
    num= atoi(string); 
    if(num>0 && num <60){
      LedI_Th=(num/30)*100;
    }
    sprintf(string,"%d%d", Rx_Buff[3]-'0',Rx_Buff[4]-'0');
    num= atoi(string);
    //sprintf(string,"%d", Rx_Buff[2]-'0');
    if(Rx_Buff[2]-'0'=='1'){// lead acid battery
      BattMaxVoltage=2999; //max voltage is 15.6V
      BattMinVoltage=2208; //min voltage is 11.4V
      BattMinDischrgVoltage=2208; //11.4V 
    }
    if(Rx_Buff[2]-'0'==2){ //LoFePo4 battery
      BattMaxVoltage=2806; //max voltage is 14.6V
      BattMinVoltage=1650; //min voltage is 8.4V
      BattMinDischrgVoltage=1650; //11.4V
    }    
    sprintf(string,"%d%d", Rx_Buff[5]-'0',Rx_Buff[6]-'0'); 
    num= atoi(string);
    PanelMaxVolt=4096;  //21.29
    PanelMinVolt=2300;  //11.9
    PanelMPPTVoltage=3075;      //15.9
  }
  configFlag=0;
  //memset(Rx_Buff,0,sizeof(Rx_Buff));
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
