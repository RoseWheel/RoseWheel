/*
  sharps.c is part of the RoseWheel project.
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
#include "misc.h"
#include "semphr.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#include "hardware.h"
#include "sharps.h"

#define AVERAGE_NUMBER 5

static xSemaphoreHandle xSharpsSemaphore;
static volatile uint16_t ADC_DMA_Buffer[SHARPS_MAX_NUMBER];
static volatile uint16_t ADC_Smoothed[SHARPS_MAX_NUMBER];

void sharps_init(sharps_t* sharps)
{
  ADC_TypeDef* ADCx = ADC1;
  sharp_t* ADCs = sharps->ADCs;
  DMA_Channel_TypeDef* DMA = DMA1_Channel1;

  dma_clock_init(DMA1);
  adc_clock_init(sharps->ADCx);

  GPIO_InitTypeDef GPIO_InitStructure =
    {
      .GPIO_Pin = 0,
      .GPIO_Speed = GPIO_Mode_IN_FLOATING,
      .GPIO_Mode = GPIO_Speed_2MHz,
    };

  for (int i = 0; i < sharps->number; i++)
  {
    gpio_clock_init(ADCs[i].GPIOx);
    GPIO_InitStructure.GPIO_Pin = ADCs[i].GPIO_Pin_x;
    GPIO_Init(ADCs[i].GPIOx, &GPIO_InitStructure);
  }

  ADC_DeInit(ADCx);
  ADC_Cmd(ADCx, ENABLE);
  // Wait until it stabilizes
  wait_us(1000);
  ADC_InitTypeDef ADC_InitStructure;
  ADC_StructInit(&ADC_InitStructure);
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_NbrOfChannel = sharps->number;
  ADC_Init(ADCx, &ADC_InitStructure);
  ADC_ResetCalibration(ADCx);
  while (ADC_GetResetCalibrationStatus(ADCx));
  ADC_StartCalibration(ADCx);
  while (ADC_GetCalibrationStatus(ADCx));

  for (int i = 0; i < sharps->number; i++)
    ADC_RegularChannelConfig(ADCx, ADCs[i].ADC_Channel,
                             i + 1, ADC_SampleTime_239Cycles5);

  ADC_Cmd(ADCx, ENABLE);

  DMA_DeInit(DMA);
  DMA_InitTypeDef DMA_InitStructure;
  DMA_StructInit(&DMA_InitStructure);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADCx->DR;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC_DMA_Buffer;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = sharps->number;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 16 ?
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA, &DMA_InitStructure);

  DMA_ITConfig(DMA, DMA_IT_TC, ENABLE);

  NVIC_InitTypeDef NVIC_InitStructure =
    {
      .NVIC_IRQChannel = DMA1_Channel1_IRQn,
      .NVIC_IRQChannelPreemptionPriority = 3, // ?
      .NVIC_IRQChannelSubPriority = 3,        // ?
      .NVIC_IRQChannelCmd = ENABLE,
    };

  NVIC_Init(&NVIC_InitStructure);

  DMA_Cmd(DMA, ENABLE);

  vSemaphoreCreateBinary(xSharpsSemaphore);
}

void DMA1_Channel1_IRQHandler()
{
  portBASE_TYPE resched = pdFALSE;
  if (DMA_GetFlagStatus(DMA1_FLAG_TC1))
  {
    xSemaphoreGiveFromISR(xSharpsSemaphore, &resched);
    DMA_ClearITPendingBit(DMA1_IT_TC1);
  }
  portEND_SWITCHING_ISR(resched);
}

void sharps_smooth_task(void* pvParameters)
{
  static int i = 0;
  static uint32_t average[SHARPS_MAX_NUMBER] = {0};

  for (;;)
  {
    xSemaphoreTake(xSharpsSemaphore, portMAX_DELAY);

    for (int j = 0; j < SHARPS_MAX_NUMBER; j++)
      average[j] += ADC_DMA_Buffer[j];

    i++;

    if (i == AVERAGE_NUMBER)
    {
      for (int j = 0; j < SHARPS_MAX_NUMBER; j++)
      {
        ADC_Smoothed[j] = average[j] / AVERAGE_NUMBER;
        average[j] = 0;
      }
      i = 0;
    }
  }
}

uint16_t sharps_get_value(int i)
{
  return ADC_Smoothed[i];
}
