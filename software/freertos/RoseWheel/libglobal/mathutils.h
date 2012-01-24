/*
  mathutils.h is part of the RoseWheel project.
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

#ifndef MATHUTILS_H
# define MATHUTILS_H

# define PI 3.1415926535897932384
# define RAD2DEG(x) ((x) * 180.0 / PI)
# define DEG2RAD(x) ((x) * PI / 180.0)
# define RAD2DEG(x) ((x) * 180.0 / PI)
# define ABS(x) (((x) > 0) ? (x) : (-(x)))
# define SIGN(x) (((x) > 0) ? (1) : (-1))

#endif
