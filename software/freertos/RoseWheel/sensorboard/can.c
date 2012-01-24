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

#include "libperiph/can.h"
#include "can.h"

void vCANInit()
{
  CAN_FilterInitTypeDef CAN_filterInit[] = {
    {
      // Vital message filter
      .CAN_FilterActivation     = ENABLE,
      .CAN_FilterFIFOAssignment = CAN_FilterFIFO0,
      .CAN_FilterIdHigh         = 0xffff,
      .CAN_FilterIdLow          = 0x0100,
      .CAN_FilterMaskIdHigh     = 0xffff,
      .CAN_FilterMaskIdLow      = 0x0100,
      .CAN_FilterMode           = CAN_FilterMode_IdMask,
      .CAN_FilterNumber         = 0,
      .CAN_FilterScale          = CAN_FilterScale_16bit,
    },

    {
      // Non-vital message filter
      .CAN_FilterActivation     = ENABLE,
      .CAN_FilterFIFOAssignment = CAN_FilterFIFO1,
      .CAN_FilterIdHigh         = 0xffff,
      .CAN_FilterIdLow          = 0x0200,
      .CAN_FilterMaskIdHigh     = 0xffff,
      .CAN_FilterMaskIdLow      = 0x0200,
      .CAN_FilterMode           = CAN_FilterMode_IdMask,
      .CAN_FilterNumber         = 2,
      .CAN_FilterScale          = CAN_FilterScale_16bit,
    }
  };

  can_t can_filters = {
    .CAN_filterInit = CAN_filterInit,
    .n_filters = 2,
  };

  can_init(&can_filters);
}

void vCANSendMsg(xCANMsg* pxMsg)
{
  can_send_msg(pxMsg->xData.octets, pxMsg->eID);
}

void vCANReceiveVitalMsg(xCANMsg* pxMsgBuffer)
{
  CanRxMsg RxMessage;

  can_receive_msgFIFO0(&RxMessage);

  pxMsgBuffer->eID = RxMessage.StdId;

  for (int i = 0; i < 8; i++)
    pxMsgBuffer->xData.octets[i] = RxMessage.Data[i];
}

void vCANReceiveNonVitalMsg(xCANMsg* pxMsgBuffer)
{
  CanRxMsg RxMessage;

  can_receive_msgFIFO1(&RxMessage);

  pxMsgBuffer->eID = RxMessage.StdId;

  for (int i = 0; i < 8; i++)
    pxMsgBuffer->xData.octets[i] = RxMessage.Data[i];
}

