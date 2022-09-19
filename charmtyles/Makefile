#  Copyright (C) 2022 Nikunj Gupta
#
#  This program is free software: you can redistribute it and/or modify it
#  under the terms of the GNU General Public License as published by the Free
#   Software Foundation, version 3.
#
#  This program is distributed in the hope that it will be useful, but WITHOUT
#  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
#  FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
#  more details.
#
#  You should have received a copy of the GNU General Public License along
#  with this program. If not, see <https://www.gnu.org/licenses/>.

include ../Makefile.common

all: libs

libs: libcharmtyles.a

.PHONY: distclean clean

libcharmtyles.a: charmtyles.o
	$(CHARMC) $(LD_OPTS) -o $@ $< -I$(BASE_DIR); rm charmtyles.o

charmtyles.o: frontend/charmtyles.cpp charmtyles.hpp
	cd backend; $(CHARMC) charmtyles.ci;
	$(CHARMC) -c $< -I$(BASE_DIR) $(OPTS)

distclean: clean
	rm libcharmtyles.a

clean:
	cd backend; rm *.decl.h *.def.h
	rm libcharmtyles.a