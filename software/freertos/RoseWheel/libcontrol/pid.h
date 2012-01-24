/*
  pid.h is part of the RoseWheel project.
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

#ifndef PID_H
# define PID_H

void pid_init(double kp, double ki, double kd);
void pid_reset();
void pid_set(double kp, double ki, double kd);
void pid_set_kp(double kp);
void pid_set_ki(double ki);
void pid_set_kd(double kd);
double pid_compute_command(double error, double error_dot, double dt_s);

#endif
