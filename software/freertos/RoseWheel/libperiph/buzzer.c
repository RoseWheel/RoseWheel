/*
  buzzer.c is part of the RoseWheel project.
  Copyright (C) 2011 RoseWheel Team <rosewheel@googlegroups.com>
  
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"

#include "buzzer.h"

# define BUZZER_CNT_MAX 1000
# define BUZZER_MIN_PSC 0
# define BUZZER_MAX_PSC (BUZZER_CNT_MAX / 4)

void buzzer_init(buzzer_t* buzzer)
{
  GPIO_InitTypeDef Buzzer_InitStructure =
    {
      .GPIO_Pin   = buzzer->GPIO_Pin_x,
      .GPIO_Speed = GPIO_Speed_2MHz,
      .GPIO_Mode  = GPIO_Mode_AF_PP,
    };

  GPIO_Init(buzzer->GPIOx, &Buzzer_InitStructure);

  TIM_TimeBaseInitTypeDef Timer_InitStructure;
  Timer_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  Timer_InitStructure.TIM_Period = BUZZER_CNT_MAX;
  Timer_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_DeInit(buzzer->TIMx);
  TIM_TimeBaseInit(buzzer->TIMx, &Timer_InitStructure);

  TIM_OCInitTypeDef OC_InitStructure;
  TIM_OCStructInit(&OC_InitStructure);
  OC_InitStructure.TIM_OCMode = TIM_OCMode_PWM1;

  switch (buzzer->TIM_Channel)
  {
    case 1:
      TIM_OC1Init(buzzer->TIMx, &OC_InitStructure);
      TIM_OC1PreloadConfig(buzzer->TIMx, TIM_OCPreload_Enable);
      break;
    case 2:
      TIM_OC2Init(buzzer->TIMx, &OC_InitStructure);
      TIM_OC2PreloadConfig(buzzer->TIMx, TIM_OCPreload_Enable);
      break;
    case 3:
      TIM_OC3Init(buzzer->TIMx, &OC_InitStructure);
      TIM_OC3PreloadConfig(buzzer->TIMx, TIM_OCPreload_Enable);
      break;
    case 4:
      TIM_OC4Init(buzzer->TIMx, &OC_InitStructure);
      TIM_OC4PreloadConfig(buzzer->TIMx, TIM_OCPreload_Enable);
      break;
  }

  if (buzzer->TIMx == TIM1 || buzzer->TIMx == TIM8)
    TIM_CtrlPWMOutputs(buzzer->TIMx, ENABLE);

  TIM_ARRPreloadConfig(buzzer->TIMx, ENABLE);
  TIM_Cmd(buzzer->TIMx, ENABLE);
}

void buzzer_on(buzzer_t* buzzer)
{
  TIM_CCxCmd(buzzer->TIMx, buzzer->TIM_Channel, TIM_CCx_Enable);
}

void buzzer_off(buzzer_t* buzzer)
{
  TIM_CCxCmd(buzzer->TIMx, buzzer->TIM_Channel, TIM_CCx_Disable);
}

void buzzer_volume_set(buzzer_t* buzzer, int v)
{
  uint16_t value = v * (BUZZER_MAX_PSC - BUZZER_MIN_PSC)
    / BUZZER_VOLUME_STEPS + BUZZER_MIN_PSC;

  switch (buzzer->TIM_Channel)
  {
    case 1:
      TIM_SetCompare1(buzzer->TIMx, value);
      break;
    case 2:
      TIM_SetCompare2(buzzer->TIMx, value);
      break;
    case 3:
      TIM_SetCompare3(buzzer->TIMx, value);
      break;
    case 4:
      TIM_SetCompare4(buzzer->TIMx, value);
      break;
  }
}

void buzzer_frequency_set(buzzer_t* buzzer, int f)
{
  TIM_PrescalerConfig(
    buzzer->TIMx,
    ((72000000L / BUZZER_CNT_MAX) / f - 1),
    TIM_PSCReloadMode_Immediate);
}
