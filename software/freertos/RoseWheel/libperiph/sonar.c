/*
  sonar.c is part of the RoseWheel project.
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

#include "sonar.h"

#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include "misc.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "strutils.h"

#include "libperiph/uart.h"
#include "libperiph/hardware.h"

#define SONARS_BAD_VALUE (-1)
#define SONARS_TIMEOUT_MS 30
#define SOUND_VELOCITY 341

static xSemaphoreHandle xResponseSemphr;
static bool enabled = FALSE;
static int value;

void sonar_init()
{
  // Enable Clocks
  gpio_clock_init(GPIOB);
  timer_clock_init(TIM2);

  GPIO_InitTypeDef GPIO_InitStructure =
    {
      .GPIO_Pin   = GPIO_Pin_10,
      .GPIO_Mode  = GPIO_Mode_IPD,
      .GPIO_Speed = GPIO_Speed_2MHz
    };
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource10);

  EXTI_InitTypeDef EXTI_InitStructure =
    {
      .EXTI_Line = EXTI_Line10,
      .EXTI_Mode = EXTI_Mode_Interrupt,
      .EXTI_Trigger = EXTI_Trigger_Rising_Falling,
      .EXTI_LineCmd = ENABLE,
    };
  EXTI_Init(&EXTI_InitStructure);

  NVIC_InitTypeDef NVIC_InitStructure =
    {
      .NVIC_IRQChannel = EXTI15_10_IRQn,
      .NVIC_IRQChannelPreemptionPriority = 7,
      .NVIC_IRQChannelSubPriority = 0,
      .NVIC_IRQChannelCmd = ENABLE,
    };
  NVIC_Init(&NVIC_InitStructure);

  TIM_TimeBaseInitTypeDef Timer_InitStructure =
    {
      .TIM_ClockDivision      = TIM_CKD_DIV1,
      .TIM_Prescaler          = 44,
      .TIM_Period             = 0xffff,
      .TIM_CounterMode        = TIM_CounterMode_Up
    };
  TIM_TimeBaseInit(TIM2, &Timer_InitStructure);

  // Enables TIM2 peripheral Preload register on ARR
  TIM_ARRPreloadConfig(TIM2, ENABLE);

  GPIO_WriteBit(GPIOB, GPIO_Pin_11, Bit_RESET);

  TIM_Cmd(TIM2, DISABLE);

  vSemaphoreCreateBinary(xResponseSemphr);
}

void EXTI15_10_IRQHandler()
{
  portBASE_TYPE reschedNeeded = pdFALSE;
  if (EXTI_GetFlagStatus(EXTI_Line10))
  {
    if (enabled)
    {
      value = TIM_GetCounter(TIM2);
      TIM_Cmd(TIM2, DISABLE);
      xSemaphoreGiveFromISR(xResponseSemphr, &reschedNeeded);
      enabled = FALSE;
    }
    else
    {
      TIM_Cmd(TIM2, ENABLE);
      enabled = TRUE;
    }
    EXTI_ClearFlag(EXTI_Line10);
  }
  portEND_SWITCHING_ISR(reschedNeeded);
}

int sonar_measure_dist_cm()
{
  int us;
  TIM_SetCounter(TIM2, 0);
  enabled = FALSE;

  GPIO_WriteBit(GPIOB, GPIO_Pin_11, Bit_SET);
  vTaskDelay(1 / portTICK_RATE_MS);
  GPIO_WriteBit(GPIOB, GPIO_Pin_11, Bit_RESET);

  if (!xSemaphoreTake(xResponseSemphr,
                      (SONARS_TIMEOUT_MS + 2) / portTICK_RATE_MS))
    return -1;

  us = value * 40000 / 0xffff;

  if (us >= (SONARS_TIMEOUT_MS - 2) * 1000)
    return -1;

  return (us * SOUND_VELOCITY / 2) / 10000;
}

