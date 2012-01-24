# This makefile is part of the RoseWheel project.
# Copyright (C) 2011 RoseWheel Team <rosewheel@googlegroups.com>
# 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.


CHECK = if test -z "$(BOARD)"; then 						\
		echo "error: set a board using BOARD make variable" >&2; 	\
		return 1; 							\
	fi

all::
	$(MAKE) BOARD=Capteurs extract
	$(MAKE) BOARD=Main_Board extract

compress::
	@$(CHECK)
	@echo "Compressing..."
	@7z a $(BOARD).7z $(BOARD)
	@rm -rf $(BOARD)
	@echo "Don't forget to commit !"

extract::
	@$(CHECK)
	@echo "Extracting..."
	@while [ -d $(BOARD) -a "$$answer" != "y" ]; do 			\
		echo -n "warning: changes in $(BOARD)/ will be lost, ";		\
		echo    "are you sure ? (y/N)";					\
		read answer;							\
	done
	@rm -rf $(BOARD)
	@7z x $(BOARD).7z
