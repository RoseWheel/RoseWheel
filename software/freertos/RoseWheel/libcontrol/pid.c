/*
  pid.c is part of the RoseWheel project.
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

#include "pid.h"
#include "strutils.h"
#include "libperiph/uart.h"
#include "strutils.h"

#define PID_INT_PRECISION 1
#define PID_MIN_CMD -24
#define PID_MAX_CMD  24

static double pid_kp = 0;
static double pid_ki = 0;
static double pid_kd = 0;
static double pid_int = 0;

void pid_init(double kp, double ki, double kd)
{
  pid_reset();
  pid_set(kp, ki, kd);
}

void pid_reset()
{
  pid_int = 0;
}

void pid_set(double kp, double ki, double kd)
{
  pid_set_kp(kp);
  pid_set_ki(ki);
  pid_set_kd(kd);
}

void pid_set_kp(double kp)
{
  pid_kp = kp;
}

void pid_set_ki(double ki)
{
  pid_ki = ki;
}

void pid_set_kd(double kd)
{
  pid_kd = kd;
}

double pid_compute_command(double error, double error_dot, double dt_s)
{
  double iterm, dterm;
  double cmd;

  if (pid_ki != 0) {
    pid_int += error * dt_s;

    iterm = pid_ki * pid_int;
    if (iterm > PID_MAX_CMD)
      iterm = PID_MAX_CMD;
    else if (iterm < PID_MIN_CMD)
      iterm = PID_MIN_CMD;
  } else {
    pid_int = 0;
    iterm = 0;
  }

  dterm = pid_kd * error_dot;

  cmd = pid_kp * error + dterm + iterm;
  if (cmd > PID_MAX_CMD)
    cmd = PID_MAX_CMD;
  else if (cmd < PID_MIN_CMD)
    cmd = PID_MIN_CMD;

  return cmd;
}
