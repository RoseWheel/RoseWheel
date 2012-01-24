/*
  kalman.c is part of the RoseWheel project.
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

#include <math.h>
#include "kalman.h"
#include "constants.h"

/*
 * A matrix coefficients
 */

static const double A00 = 1.02631;
static const double A01 = 0.05042;
static const double A10 = 1.05646;
static const double A11 = 1.02568;

/*
 * B matrix coefficients
 */

static const double B0 = -0.0001128;
static const double B1 = -0.0045274;

/*
 * Covariance matrix. Updated at each time step to determine how well
 * the sensors are tracking the actual state.
 */

static double P[3][3] = {
  { 1, 0, 0},
  { 0, 1, 0},
  { 0, 0, 1},
};

/*
 * 3 states system:
 * Angle.
 * Angular rate.
 * gyro bias.
 */

static double angle = 0.0;
static double rate  = 0.0;
static double bias  = 0.0;

/*
 * Functions to access the data.
 */

double kalman_get_angle() { return angle; }
double kalman_get_rate()  { return rate;  }
double kalman_get_bias()  { return bias;  }

/*
 * R represents the measurement covariance noise.
 */

static const double R_angle = 0.0000001;
static const double R_rate  = 0.0000001;

/*
 * Q is a 3x3 matrix that represents the process covariance noise.
 * In this case, it indicates how much we trust the modelisation of
 * The system.
 */

static const double Q_angle = 10000;
static const double Q_rate  = 10000;
static const double Q_bias  = 10000;

/*
 * State_update is called every dt with a new set of measurements.
 * It updates the state vector estimate.
 *
 * the state vector is :
 *
 * X = [ angle, rate, bias ]'
 *
 * It runs the state estimation forward via the functions:
 *
 * X = A * X + B * U
 * P = A * P * A' + Q
 *
 */

void kalman_reset(const double angle_i, const double rate_i, const double bias_i)
{
  angle = angle_i;
  rate = rate_i;
  bias = bias_i;
}

void kalman_state_update(const double Vleft, const double Vright)
{

  /* Update state vector */
  /* Update only the values that need to be updated. */
  angle = A00 * angle + A01 * rate + B0 * Vleft + B0 * Vright;
  rate  = A10 * angle + A11 * rate + B1 * Vleft + B1 * Vright;

  /* Update the covariance matrix */
  /* Uptade only the values that need to be updated. */

  const double PAt[3][3] = {
    { P[0][0] * A00 + P[0][1] * A01, P[0][0] * A10 + P[0][1] * A11, P[0][2] },
    { P[1][0] * A00 + P[1][1] * A01, P[1][0] * A10 + P[1][1] * A11, P[1][2] },
    { P[2][0] * A00 + P[2][1] * A01, P[2][0] * A10 + P[2][1] * A11, P[2][2] },
  };

  P[0][0] = A00 * PAt[0][0] + A01 * PAt[1][0] + Q_angle;
  P[0][1] = A00 * PAt[0][1] + A01 * PAt[1][1];
  P[0][2] = A00 * PAt[0][2] + A01 * PAt[1][2];

  P[1][0] = A10 * PAt[0][0] + A11 * PAt[1][0];
  P[1][1] = A10 * PAt[0][1] + A11 * PAt[1][1] + Q_rate;
  P[1][2] = A10 * PAt[0][2] + A11 * PAt[1][2];

  P[2][0] = PAt[2][0];
  P[2][1] = PAt[2][1];
  P[2][2] = PAt[2][2] + Q_bias;

}

/*
 * ax_m and az_m do not need to be scaled into actual units, but
 * must be zeroed and have the same scale.
 *
 * We work in degrees, atan2 must be scale to fit degrees and not radians.
 *
 * This does not need to be called every time step, but can be if
 * the accelerometer data are available at the same rate as the
 * rate gyro measurement.
 *
 * For a two-axis accelerometer mounted perpendicular to the rotation
 * axis, we can compute the angle for the full 360 degree rotation
 * with no linearization errors by using the arctangent of the two
 * readings.
 *
 * As commented in state_update, the math here is simplified to
 * make it possible to execute on a small microcontroller with no
 * floating point unit.  It will be hard to read the actual code and
 * see what is happening, which is why there is this extensive
 * comment block.
 *
 * The C matrix is a 2x3 (measurements x states). It shows the relation
 * between the measurements and the state vector.
 *
 *      C = [ 1  0  0
 *            0  1  1]
 *
 */

void kalman_cov_update(const double acc_mx, const double acc_mz, const double gyro_m)
{

  const double angle_err = atan2(acc_mx, -acc_mz) * 180 / PI - angle;
  const double rate_err  = gyro_m - (rate + bias);

/*
 * Matrix product PxC'
 */

  const double PCt[3][2] = {
    { P[0][0], P[0][1] + P[0][2] },
    { P[1][0], P[1][1] + P[1][2] },
    { P[2][0], P[2][1] + P[2][2] },
  };

/*
 * Compute the error estimate.  From the Kalman filter paper:
 *
 *      E = C * P * C' + R
 *
 */

  const double E[2][2] = {
    { PCt[0][0] + R_angle, PCt[0][1] },
    { PCt[1][0] + PCt[2][0], PCt[1][1] + PCt[2][1] + R_rate },
  };


  /*
   * Compute the inverse of E
   */

  double iE[2][2] = {
    {0, 0},
    {0, 0},
  };

  const double detE = E[0][0] * E[1][1] - E[1][0] * E[0][1];

  if (detE != 0)
  {
    iE[0][0] =  E[1][1] / detE;
    iE[0][1] = -E[0][1] / detE;
    iE[1][0] = -E[1][0] / detE;
    iE[1][1] =  E[0][0] / detE;
  }

  /*
   * Compute the Kalman filter gains.  From the Kalman paper:
   *
   *      K = P * C' * inv(E)
   *
   */

  const double K[3][2] = {
    { PCt[0][0]*iE[0][0] + PCt[0][1]*iE[1][0], PCt[0][0]*iE[0][1] + PCt[0][1]*iE[1][1] },
    { PCt[1][0]*iE[0][0] + PCt[1][1]*iE[1][0], PCt[1][0]*iE[0][1] + PCt[1][1]*iE[1][1] },
    { PCt[2][0]*iE[0][0] + PCt[2][1]*iE[1][0], PCt[2][0]*iE[0][1] + PCt[2][1]*iE[1][1] },
  };

  /*
   * Update covariance matrix.  Again, from the Kalman filter paper:
   *
   *      P = P - K * C * P
   *
   */

  const double CP[2][3] = {
    { P[0][0]          , P[0][1]          , P[0][2]         },
    { P[1][0] + P[2][0], P[1][1] + P[2][1], P[1][2] + P[2][2] },
  };

  P[0][0] -= K[0][0] * CP[0][0] + K[0][1] * CP[1][0];
  P[0][1] -= K[0][0] * CP[0][1] + K[0][1] * CP[1][1];
  P[0][2] -= K[0][0] * CP[0][2] + K[0][1] * CP[1][2];

  P[1][0] -= K[1][0] * CP[0][0] + K[1][1] * CP[1][0];
  P[1][1] -= K[1][0] * CP[0][1] + K[1][1] * CP[1][1];
  P[1][2] -= K[1][0] * CP[0][2] + K[1][1] * CP[1][2];

  P[2][0] -= K[2][0] * CP[0][0] + K[2][1] * CP[1][0];
  P[2][1] -= K[2][0] * CP[0][1] + K[2][1] * CP[1][1];
  P[2][2] -= K[2][0] * CP[0][2] + K[2][1] * CP[1][2];

  /*
   * Update our state estimate.  Again, from the Kalman paper:
   *
   *      X += K * (Y - C*X)
   *
   */

  angle += (K[0][0] * angle_err + K[0][1] * rate_err);
  rate  += (K[1][0] * angle_err + K[1][1] * rate_err);
  bias  += (K[2][0] * angle_err + K[2][1] * rate_err);

}

