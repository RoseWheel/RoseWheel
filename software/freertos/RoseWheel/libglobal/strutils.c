/*
  strutils.c is part of the RoseWheel project.
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
#include "strutils.h"

/* Source: Wikipédia */
/* reverse:  reverse string s in place */
void reverse(char* s)
{
  int i, j;
  char c;
  for (i = 0, j = strlen(s) - 1; i < j; i++, j--)
  {
    c = s[i];
    s[i] = s[j];
    s[j] = c;
  }
}

/* Adapted from Wikipédia */
/* itoa:  convert n to characters in s */
char* itoa(int n, char* s)
{
  int i, sign;

  if ((sign = n) < 0)       /* record sign */
    n = -n;                 /* make n positive */
  i = 0;
  do                        /* generate digits in reverse order */
  {
    s[i++] = n % 10 + '0';  /* get next digit */
  } while ((n /= 10) > 0);  /* delete it */
  if (sign < 0)
    s[i++] = '-';
  s[i] = '\0';
  reverse(s);

  return s;
}

#define FLTOA_PRECISION 0.000001

char* fltoa(float f, char *s)
{
  int f_int;
  int f_frac;
  char* const str = s;

  if (f < 0) {
    *s++ = '-';
    f = -f;
  }

  f_int  = f;
  f_frac = (f - f_int) / FLTOA_PRECISION;

  itoa(f_int, s);

  while (*s)
    s++;

  *s++ = '.';

  itoa(f_frac, s);

  while (*s)
    s++;

  while (*(--s) == '0')
    *s = '\0';

  if (*s == '.')
    *s = '\0';

  return str;
}

float atofl(const char* str)
{
  float value = 0, f = 0;
  int i = 0;

  while (!(('0' <= str[i]) && (str[i] <= '9')) && (str[i] != 0))
    i++;

  while (str[i] != 0)
  {
    if (str[i] == '.')
    {
      f = 10;
      i++;
      continue;
    }

    if (f)
    {
      value += (float)(str[i] - '0') / f;
      f *= 10;
    }
    else
      value = 10 * value + str[i] - '0';

    i++;
  }

  if (str[0] == '-')
    value = -value;

  return value;
}

int atoi_eol(const char* str, char eol)
{
  const char* s = str;
  int value = 0;

  if (*s == '-')
    s++;

  while (*s != eol)
    value = 10 * value + *s++ - '0';

  if (*str == '-')
    value = -value;

  return value;
}

int atoi(const char* str)
{
  return atoi_eol(str, 0);
}

int htoi_eol(const char* str, char eol)
{
  int value = 0, i = 0, tmp;
  while (str[i] != eol)
  {
    tmp = str[i];
    if (str[i] <= '9' && str[i] >= '0')
      tmp -= '0';
    else if (str[i] <= 'F' && str[i] >= 'A')
      tmp = tmp - 'A' + 10;
    else if (str[i] <= 'f' && str[i] >= 'a')
      tmp = tmp - 'a' + 10;

    value = 16 * value + tmp;
    i++;
  }
  return value;
}

int htoi(const char* str)
{
  return htoi_eol(str, 0);
}

int xtoi_eol(const char* str, char eol)
{
  if (str[0] == '0' && str[1] == 'x')
    return htoi_eol(str + 2, eol);
  return atoi_eol(str, eol);
}

int xtoi(const char* str)
{
  return xtoi_eol(str, 0);
}

char* trim_in_place(char* str)
{
  // Trim spaces at start
  int offset = 0, size;

  while (str[offset] == ' ')
    offset++;
  size = offset;
  while (str[size])
    size++;

  // Empty string
  if (size == 0)
    return str;

  // Full of spaces string
  if (size == offset)
  {
    str[0] = 0;
    return str;
  }

  // Trim spaces at end
  for (int i = size - 1; i >= offset; --i)
  {
    if (str[i] != ' ')
    {
      str[i + 1] = 0;
      break;
    }
  }

  return str + offset;
}

int is_letter(char c)
{
  if (c >= 'A' && c <= 'Z')
    return 1;
  if (c >= 'a' && c <= 'z')
    return 1;
  return 0;
}

int is_number(char c)
{
  return c >= '0' && c <= '9';
}

int is_space(char c)
{
  return c == ' ' || c == '\t';
}
