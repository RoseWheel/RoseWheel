/*
  can.c is part of the RoseWheel project.
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

#include "assert_param.h"
#include "FreeRTOS.h"
#include "misc.h"
#include "semphr.h"
#include "queue.h"
#include "task.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_can.h"

#include "can.h"
#include "hardware.h"
#include "leds.h"

#define CAN_RX_PIN GPIO_Pin_11
#define CAN_TX_PIN GPIO_Pin_12

static xQueueHandle xCAN_receive_queue0;
static xQueueHandle xCAN_receive_queue1;

void can_init(can_t* can_filters)
{

  gpio_clock_init(GPIOA);
  can_clock_init(CAN1);

  GPIO_InitTypeDef GPIO_InitStructure =
    {
      .GPIO_Pin   = CAN_RX_PIN,
      .GPIO_Speed = GPIO_Speed_50MHz,
      .GPIO_Mode  = GPIO_Mode_IN_FLOATING,
    };
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = CAN_TX_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  CAN_InitTypeDef CAN_InitStructure;
  CAN_StructInit(&CAN_InitStructure);
  CAN_InitStructure.CAN_Prescaler = 1;
  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
  CAN_InitStructure.CAN_SJW = CAN_SJW_4tq;
  CAN_InitStructure.CAN_BS1 = CAN_BS1_11tq;
  CAN_InitStructure.CAN_BS2 = CAN_BS2_4tq;
  CAN_InitStructure.CAN_ABOM = ENABLE;
  CAN_InitStructure.CAN_RFLM = ENABLE;
  CAN_Init(CAN1, &CAN_InitStructure);

  CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
  CAN_ITConfig(CAN1, CAN_IT_FMP1, ENABLE);

  NVIC_InitTypeDef NVIC_InitStructure =
    {
      .NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn,
      .NVIC_IRQChannelPreemptionPriority = 7,
      .NVIC_IRQChannelSubPriority = 0,
      .NVIC_IRQChannelCmd = ENABLE,
    };
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX1_IRQn;
  NVIC_Init(&NVIC_InitStructure);

  for (int i = 0; i < can_filters->n_filters; i++)
    CAN_FilterInit(can_filters->CAN_filterInit + i);

  xCAN_receive_queue0 = xQueueCreate(10, sizeof(CanRxMsg));
  xCAN_receive_queue1 = xQueueCreate(10, sizeof(CanRxMsg));

}

void USB_LP_CAN1_RX0_IRQHandler()
{
  CanRxMsg RxMessage;
  portBASE_TYPE resch = pdFALSE;
  CAN_Receive(CAN1, CAN_FIFO0,&RxMessage);
  xQueueSendFromISR(xCAN_receive_queue0, &RxMessage, &resch);
  portEND_SWITCHING_ISR(resch);
}

void CAN1_RX1_IRQHandler()
{
  CanRxMsg RxMessage;
  portBASE_TYPE resch = pdFALSE;
  CAN_Receive(CAN1, CAN_FIFO1, &RxMessage);
  xQueueSendFromISR(xCAN_receive_queue1, &RxMessage, &resch);
  portEND_SWITCHING_ISR(resch);
}

void can_send_msg(uint8_t data[8], uint8_t TxID)
{
  static CanTxMsg TxMessage;

  // Values between 0 to 0x7FF
  TxMessage.StdId = TxID;
  // Specifies the type of identifier for the message that will be transmitted
  TxMessage.IDE = CAN_ID_STD;
  // Specifies the type of frame for the message that will be transmitted
  TxMessage.RTR = CAN_RTR_DATA;
  // Specifies the length of the frame that will be transmitted
  TxMessage.DLC = 8;

  for (int i = 0; i < 8; i++)
    TxMessage.Data[i] = data[i];

  CAN_Transmit(CAN1,&TxMessage);
}

portBASE_TYPE can_receive_msgFIFO0(CanRxMsg* RxMessage)
{
  return xQueueReceive(xCAN_receive_queue0, RxMessage, portMAX_DELAY);
}

portBASE_TYPE can_receive_msgFIFO1(CanRxMsg* RxMessage)
{
  return xQueueReceive(xCAN_receive_queue1, RxMessage, portMAX_DELAY);
}


