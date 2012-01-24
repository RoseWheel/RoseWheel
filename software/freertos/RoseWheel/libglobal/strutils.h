/*
  strutils.h is part of the RoseWheel project.
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

#ifndef STRUTILS_H
# define STRUTILS_H

void reverse(char* s);
char* itoa(int n, char* s);
float atofl(const char* str);
char* fltoa(float f, char *s);
int atoi_eol(const char* str, char eol);
int atoi(const char* str);
int htoi_eol(const char* str, char eol);
int htoi(const char* str);
int xtoi_eol(const char* str, char eol);
int xtoi(const char* str);
char* trim_in_place(char* str);
int is_letter(char c);
int is_number(char c);
int is_space(char c);

#endif
