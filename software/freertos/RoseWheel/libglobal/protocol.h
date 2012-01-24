/*
  protocol.h is part of the RoseWheel project.
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

#ifndef LIBGLOBAL_PROTOCOL_H
# define LIBGLOBAL_PROTOCOL_H

enum eCANMsgID {
  MSG_STOP         = 0x08,
  MSG_RESET        = 0x09,
  MSG_ANGLE_ANGVEL = 0x0a,
  MSG_ENCINFO      = 0x0b,
  MSG_MOTORCMD     = 0x0c,
  MSG_DANGER       = 0x0d,
  MSG_REMOTECMD    = 0x10,
  MSG_USERCMD      = 0x11,
  MSG_SPEED        = 0x12,
  MSG_BATTERYLVL   = 0x13,
  MSG_CTRLSTATE    = 0x14,
};

typedef struct
{
  enum eCANMsgID eID;
  union {
    struct {
      float first;
      float second;
    } values;
    struct {
      uint32_t high;
      uint32_t low;
    } words;
    uint8_t octets[8];
  } xData;
} xCANMsg;

#define BT_MSG_START '#'
#define BT_MSG_END   '\r'

enum eControlState {REMOTE, HUMAN, SLEEPING};

#endif
