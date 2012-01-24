/*
  linalg.h is part of the RoseWheel project.
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

#ifndef LINALG_H
# define LINALG_H

#include <stdlib.h>

typedef struct
{
  double* data;
  int size_x;
  int size_y;
} matrix_t;

typedef struct
{
  double* data;
  int size;
} vector_t;

void matrix_init(matrix_t* matrix, double* data,
                 size_t size_x, size_t size_y);
void vector_init(vector_t* vector, double* data, size_t size);
void mv_mult(vector_t* result, matrix_t* m, vector_t* v);

#endif
