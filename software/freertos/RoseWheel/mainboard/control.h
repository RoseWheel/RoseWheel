/*
  control.h is part of the RoseWheel project.
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

#ifndef CONTROL_H
# define CONTROL_H

# include "FreeRTOS.h"
# include "libglobal/linalg.h"

# define HUMAN_DRIVEN_EQ_ANGLE 0
# define SELF_DRIVEN_EQ_ANGLE DEG2RAD(15) /* equilibrium position for
                                            self-driven mode */

extern const double EQ_ANGLE[];

enum eControlMode {
  SELF_DRIVEN = 0,
  HUMAN_DRIVEN = 1,
};

enum eStateVariable {
  THETA     = 0,
  THETA_DOT = 1,
  PHI       = 2,
  PHI_DOT   = 3,
};

enum eCommandVariable {
  LEFT = 0,
  RIGHT = 0,
};

void vControlInit(unsigned portBASE_TYPE xDaemonPriority);

void vControlUpdate(vector_t* pxCurrentState, double dTimeElapsed);

vector_t* xControlGetCommand();

double vControlGetEqAngle();

void vControlSetEqAngle(double dEqAngle);

void vControlSetEqAngleOffset(double dEqAngleOffset);

double vControlGetCurrentAngleOffset();

#endif
