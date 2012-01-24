/*
  kalman.h is part of the RoseWheel project.
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

#ifndef KALMAN_H
#define KALMAN_H

double kalman_get_angle();
double kalman_get_rate();
double kalman_get_bias();

void kalman_reset(const double angle_i, const double rate_i, const double bias_i);
void kalman_state_update(const double Vleft, const double Vright);
void kalman_cov_update(const double acc_mx, double acc_mz, const double gyro_m);

#endif
