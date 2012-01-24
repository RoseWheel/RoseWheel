/*
  linalg.c is part of the RoseWheel project.
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

#include <string.h>

#include "assert_param.h"
#include "linalg.h"

void matrix_init(matrix_t* matrix, double* data,
                 size_t size_x, size_t size_y)
{
  matrix->data = data;
  matrix->size_x = size_x;
  matrix->size_y = size_y;
  matrix->data = data;
}

void vector_init(vector_t* vector, double* data, size_t size)
{
  vector->size = size;
  vector->data = data;
}

void mv_mult(vector_t* result, matrix_t* m, vector_t* v)
{
  assert_param(m->size_x == v->size);
  assert_param(m->size_y == result->size);
  for (int i = 0; i < m->size_y; i++)
  {
    result->data[i] = 0;
    for (int j = 0; j < m->size_x; j++)
      result->data[i] += m->data[i * m->size_x + j] * v->data[j];
  }
}
