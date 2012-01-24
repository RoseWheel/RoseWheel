/*
  adc.c is part of the RoseWheel project.
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

#include "stm32f10x_adc.h"
#include "misc.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "adc.h"
#include "hardware.h"

static xQueueHandle xADCQueue;
static xSemaphoreHandle xADCMutex;

void adc_init(adc_t* adc)
{
  static int already_initialized = 0;

  gpio_clock_init(adc->GPIOx);

  GPIO_InitTypeDef GPIO_InitStructure =
    {
      .GPIO_Pin = adc->GPIO_Pin_x,
      .GPIO_Speed = GPIO_Speed_2MHz,
      .GPIO_Mode = GPIO_Mode_IN_FLOATING,
    };

  GPIO_Init(adc->GPIOx, &GPIO_InitStructure);

  // avoid configuring ADC2 again:
  if (already_initialized)
    return;

  already_initialized = 1;

  adc_clock_init(ADC2);
  xADCMutex = xSemaphoreCreateMutex();
  xADCQueue = xQueueCreate(10, sizeof(unsigned short));

  ADC_Cmd(ADC2, ENABLE);
  // Wait until it stabilizes
  wait_us(1000);
  ADC_InitTypeDef ADC_InitStructure;
  ADC_StructInit(&ADC_InitStructure);
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC2, &ADC_InitStructure);
  ADC_StartCalibration(ADC2);
  while (ADC_GetCalibrationStatus(ADC2));
  ADC_Cmd(ADC2, DISABLE);
  ADC_ITConfig(ADC2, ADC_IT_EOC, ENABLE);

  // Enable interrupt UART:
  NVIC_InitTypeDef NVIC_InitStructure =
  {
    .NVIC_IRQChannel = ADC1_2_IRQn,
    .NVIC_IRQChannelPreemptionPriority = 7,
    .NVIC_IRQChannelSubPriority = 0,
    .NVIC_IRQChannelCmd = ENABLE,
  };
  NVIC_Init(&NVIC_InitStructure);
  ADC_Cmd(ADC2, ENABLE);
}

void ADC1_2_IRQHandler(void)
{
  signed portBASE_TYPE xHigherPriorityTaskWoken;
  unsigned short in_buffer = ADC2->DR & 0xfff;
  xQueueSendToBackFromISR(xADCQueue, &in_buffer,
                          &xHigherPriorityTaskWoken);
  portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

unsigned short adc_read(adc_t* adc)
{
  unsigned short in_buffer;
  xSemaphoreTake(xADCMutex, portMAX_DELAY);
  // we use the same ADC for all peripherals
  ADC_RegularChannelConfig(ADC2, adc->ADC_Channel, 1,
                           ADC_SampleTime_239Cycles5);
  ADC_Cmd(ADC2, ENABLE);
  xQueueReceive(xADCQueue, &in_buffer, portMAX_DELAY);
  xSemaphoreGive(xADCMutex);
  return in_buffer;
}
