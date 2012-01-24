/*
  hardware.c is part of the RoseWheel project.
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

#include "misc.h"
#include "stm32f10x.h"
#include "stm32f10x_flash.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#include "hardware.h"

void hardware_init()
{
  // Enable HSE:
  RCC_HSEConfig(RCC_HSE_ON);

  // Wait for HSE to be ready:
  while (RCC_WaitForHSEStartUp() != SUCCESS);

  // Set PLL to be 9 * HSE = 72 MHz:
  RCC_PLLCmd(DISABLE);
  RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
  RCC_PLLCmd(ENABLE);

  // Wait for PLL to be ready:
  while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) != SET);

  // Two wait states, if 48 MHz < SYSCLK <= 72 MHz:
  FLASH_SetLatency(FLASH_Latency_2);

  // Set PLL as system clock:
  RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

  // Disable HSI:
  RCC_HSICmd(DISABLE);

  // Set APB low-speed clock (PCLK1), divide by 2:
  RCC_PCLK1Config(RCC_HCLK_Div2);

  // Set APB high-speed clock (PCLK2), do not divide:
  RCC_PCLK2Config(RCC_HCLK_Div1);

  // Set AHB clock (HCLK), do not divide:
  RCC_HCLKConfig(RCC_SYSCLK_Div1);

  // 3 bits for pre-emption priority 1 bits for subpriority:
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);

  // Set core clock as SYSTICK source:
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);

#ifdef RAM_BOOT
  // Put vector interrupt table in RAM:
  NVIC_SetVectorTable(NVIC_VectTab_RAM, SCB_VTOR_TBLBASE);
#endif
}

#define GPIO_CASE(GPIO)                                          \
  case (uint32_t)GPIO:                                           \
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_##GPIO, ENABLE);         \
  break                                                          \

void gpio_clock_init(GPIO_TypeDef* GPIOx)
{
  switch((uint32_t)GPIOx)
  {
    GPIO_CASE(GPIOA);
    GPIO_CASE(GPIOB);
    GPIO_CASE(GPIOC);
    GPIO_CASE(GPIOD);
    GPIO_CASE(GPIOE);
    GPIO_CASE(GPIOF);
    GPIO_CASE(GPIOG);
  }
}

#undef GPIO_CASE

#define TIMER_CASE81(TIM)                                  \
  case (uint32_t)TIM:                                      \
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_##TIM,  ENABLE);   \
  break                                                    \

#define TIMER_CASE27(TIM)                                  \
  case (uint32_t)TIM:                                      \
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_##TIM,  ENABLE);   \
  break                                                    \

void timer_clock_init(TIM_TypeDef* TIMz)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,  ENABLE);
  switch((uint32_t)TIMz)
  {
    TIMER_CASE81(TIM1);
    TIMER_CASE27(TIM2);
    TIMER_CASE27(TIM3);
    TIMER_CASE27(TIM4);
    TIMER_CASE27(TIM5);
    TIMER_CASE27(TIM6);
    TIMER_CASE27(TIM7);
    TIMER_CASE81(TIM8);
  }
}

#undef TIMER_CASE27
#undef TIMER_CASE81

void dma_clock_init(DMA_TypeDef* DMAx)
{
  if (DMAx == DMA1)
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  else if (DMAx == DMA2)
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
}

void adc_clock_init(ADC_TypeDef* ADCx)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,  ENABLE);
  if (ADCx == ADC1)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,  ENABLE);
  else if (ADCx == ADC2)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2,  ENABLE);
  else if (ADCx == ADC3)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3,  ENABLE);
}

void spi_clock_init(SPI_TypeDef* SPIx)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  if (SPIx == SPI1)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
  else if (SPIx == SPI2)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
}

void can_clock_init(CAN_TypeDef* CANx)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
}

void wait_us(int x)
{
  int i, j;
  for(i = 0; i < x; i++)
    for(j = 0; j < 50; j++);
}

uint8_t gpio_pin_to_source(uint16_t GPIO_Pin_x)
{
  uint8_t GPIO_PinSource_x;
  switch (GPIO_Pin_x)
  {
    default: // Shut the compiler warning.
    case GPIO_Pin_0:
      GPIO_PinSource_x = GPIO_PinSource0;
      break;
    case GPIO_Pin_1:
      GPIO_PinSource_x = GPIO_PinSource1;
      break;
    case GPIO_Pin_2:
      GPIO_PinSource_x = GPIO_PinSource3;
      break;
    case GPIO_Pin_4:
      GPIO_PinSource_x = GPIO_PinSource4;
      break;
    case GPIO_Pin_5:
      GPIO_PinSource_x = GPIO_PinSource5;
      break;
    case GPIO_Pin_6:
      GPIO_PinSource_x = GPIO_PinSource6;
      break;
    case GPIO_Pin_7:
      GPIO_PinSource_x = GPIO_PinSource7;
      break;
    case GPIO_Pin_8:
      GPIO_PinSource_x = GPIO_PinSource8;
      break;
    case GPIO_Pin_9:
      GPIO_PinSource_x = GPIO_PinSource9;
      break;
    case GPIO_Pin_10:
      GPIO_PinSource_x = GPIO_PinSource10;
      break;
    case GPIO_Pin_11:
      GPIO_PinSource_x = GPIO_PinSource11;
      break;
    case GPIO_Pin_12:
      GPIO_PinSource_x = GPIO_PinSource12;
      break;
    case GPIO_Pin_13:
      GPIO_PinSource_x = GPIO_PinSource13;
      break;
    case GPIO_Pin_14:
      GPIO_PinSource_x = GPIO_PinSource14;
      break;
    case GPIO_Pin_15:
      GPIO_PinSource_x = GPIO_PinSource15;
      break;
  }

  return GPIO_PinSource_x;
}
