/*
  can.h is part of the RoseWheel project.
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

#ifndef LIBPERIPH_CAN_H
# define LIBPERIPH_CAN_H

#include "FreeRTOS.h"
#include "stm32f10x_can.h"

typedef struct
{
  CAN_FilterInitTypeDef* CAN_filterInit;
  int n_filters;
} can_t;

void can_init(can_t* can_filters);
void can_send_msg(uint8_t data[8], uint8_t TxID);
portBASE_TYPE can_receive_msgFIFO0(CanRxMsg* RxMessage);
portBASE_TYPE can_receive_msgFIFO1(CanRxMsg* RxMessage);

#endif
