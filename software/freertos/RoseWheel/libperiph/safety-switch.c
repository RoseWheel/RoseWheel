/*
  safety-switch.c is part of the RoseWheel project.
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

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "misc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_exti.h"

#include "libglobal/strutils.h"
#include "uart.h"
#include "hardware.h"
#include "safety-switch.h"

#define BUTTON_MAX_PERIOD_MS 1000

static xSemaphoreHandle xSafetySemphr;
static bool safety_switch_status = FALSE;

static void safety_switch_daemon(void* pvParameters);

void default_handler() {}
static safety_handler_t press_handler = default_handler;
static safety_handler_t release_handler = default_handler;
static safety_handler_t long_release_handler = default_handler;
static int long_release_time_ms = 1000;

void safety_switch_init(unsigned portBASE_TYPE daemon_priority)
{
  gpio_clock_init(GPIOA);

  GPIO_InitTypeDef init =
    {
      .GPIO_Pin   = GPIO_Pin_6,
      .GPIO_Speed = GPIO_Speed_2MHz,
      .GPIO_Mode  = GPIO_Mode_IPU,
    };
  GPIO_Init(GPIOA, &init);

  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource6);

  EXTI_InitTypeDef EXTI_InitStructure =
    {
      .EXTI_Line = EXTI_Line6,
      .EXTI_Mode = EXTI_Mode_Interrupt,
      .EXTI_Trigger = EXTI_Trigger_Rising,
      .EXTI_LineCmd = ENABLE,
    };
  EXTI_Init(&EXTI_InitStructure);

  vSemaphoreCreateBinary(xSafetySemphr);

  EXTI_ClearFlag(EXTI_Line6);    // Clear pending interrupt.

  NVIC_InitTypeDef NVIC_InitStructure =
    {
      .NVIC_IRQChannel = EXTI9_5_IRQn,
      .NVIC_IRQChannelPreemptionPriority = 7,
      .NVIC_IRQChannelSubPriority = 0,
      .NVIC_IRQChannelCmd = ENABLE,
    };
  NVIC_Init(&NVIC_InitStructure);

  // Create the daemon
  xTaskCreate(safety_switch_daemon, (const signed char * const)"safetyswd",
	      configMINIMAL_STACK_SIZE, NULL, daemon_priority, NULL);
}

void EXTI9_5_IRQHandler()
{
  portBASE_TYPE reschedNeeded = pdFALSE;
  if (EXTI_GetFlagStatus(EXTI_Line6)) // EXTI6 interrupt pending ?
  {
    xSemaphoreGiveFromISR(xSafetySemphr, &reschedNeeded);
    EXTI_ClearFlag(EXTI_Line6);    // Clear pending interrupt.
  }
  portEND_SWITCHING_ISR(reschedNeeded);
}

void safety_switch_set_handler(event_t event, safety_handler_t handler)
{
  handler = handler ? handler : default_handler;
  switch (event)
  {
    case PRESS:
      press_handler = handler;
      break;
    case RELEASE:
      release_handler = handler;
      break;
    case LONG_RELEASE:
      long_release_handler = handler;
      break;
  }
}

bool safety_switch_set_long_release_time(int ms)
{
  if (ms > 0)
  {
    long_release_time_ms = ms;
    return FALSE;
  }
  return TRUE;
}

void safety_switch_daemon(void* pvParameters)
{
  portTickType last_press = 0, last_release = 0, current;
  int value;

  vTaskDelay(1000 / portMAX_DELAY);
  xSemaphoreTake(xSafetySemphr, 0);

  safety_switch_status = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6);
  for (;;)
  {
    current = xTaskGetTickCount() * portTICK_RATE_MS;

    if (xSemaphoreTake(xSafetySemphr, 100 / portTICK_RATE_MS))
    {
      vTaskDelay(200 / portTICK_RATE_MS);
      xSemaphoreTake(xSafetySemphr, 0);

      value = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6);
      
      if (!safety_switch_status && value)
      {
        last_press = current;
        safety_switch_status = TRUE;
        (*press_handler)();
        
        EXTI_InitTypeDef EXTI_InitStructure = {
          .EXTI_Line = EXTI_Line6,
          .EXTI_Mode = EXTI_Mode_Interrupt,
          .EXTI_Trigger = EXTI_Trigger_Falling,
          .EXTI_LineCmd = ENABLE,
        };
        EXTI_Init(&EXTI_InitStructure);

      }
      else if (safety_switch_status && !value)
      {
        last_release = current;
        safety_switch_status = FALSE;
        (*release_handler)();

        EXTI_InitTypeDef EXTI_InitStructure = {
          .EXTI_Line = EXTI_Line6,
          .EXTI_Mode = EXTI_Mode_Interrupt,
          .EXTI_Trigger = EXTI_Trigger_Rising,
          .EXTI_LineCmd = ENABLE,
        };
        EXTI_Init(&EXTI_InitStructure);

      }
    }
    else
    {
      if (!safety_switch_status
          && last_release
          && (current - last_release > long_release_time_ms))
      {
        last_release = 0;
        (*long_release_handler)();
      }
    }
  }
}
