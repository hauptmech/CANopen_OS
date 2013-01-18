#! gmake

#
# Copyright (C) 2006 Laurent Bessard
# 
# This file is part of canfestival, a library implementing the canopen
# stack
# 
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.
# 
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
# 
# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
# 

CC = gcc
CXX = g++
LD = g++
OPT_CFLAGS = -O2
CFLAGS = $(OPT_CFLAGS)
PROG_CFLAGS =  -fPIC
EXE_CFLAGS =  -lpthread -lrt -ldl
OS_NAME = Linux
ARCH_NAME = x86_64
PREFIX = /home/teh/advr/Robot-PKGBUILDs/canfestival/pkg/usr
TARGET = unix
CAN_DRIVER = can_peak_linux
TIMERS_DRIVER = timers_unix
CANOPENSHELL = CANOpenShell

INCLUDES = -I/usr/include/canfestival

MASTER_OBJS = CANOpenShellMasterOD.o CANOpenShellSlaveOD.o CANOpenShell.o

OBJS = $(MASTER_OBJS) -lcanfestival -lcanfestival_can_socket -lcanfestival_unix -lreadline

ifeq ($(TIMERS_DRIVER),timers_xeno)
	PROGDEFINES = -DUSE_XENO
endif

all: $(CANOPENSHELL)


$(CANOPENSHELL): $(OBJS)
	$(LD) $(CFLAGS) $(PROG_CFLAGS) ${PROGDEFINES} $(INCLUDES) -o $@ $(OBJS) $(EXE_CFLAGS)
	
CANOpenShellMasterOD.c: CANOpenShellMasterOD.od
	$(MAKE) -C objdictgen gnosis
	python2 objdictgen/objdictgen.py CANOpenShellMasterOD.od CANOpenShellMasterOD.c

CANOpenShellSlaveOD.c: CANOpenShellSlaveOD.od
	$(MAKE) -C objdictgen gnosis
	python2 objdictgen/objdictgen.py CANOpenShellSlaveOD.od CANOpenShellSlaveOD.c

%.o: %.c
	$(CC) $(CFLAGS) $(PROG_CFLAGS) ${PROGDEFINES} $(INCLUDES) -o $@ -c $<

clean:
	rm -f $(MASTER_OBJS)
	rm -f $(CANOPENSHELL)
		
mrproper: clean
	rm -f CANOpenShellMasterOD.c
	rm -f CANOpenShellSlaveOD.c

install: $(CANOPENSHELL)
	mkdir -p $(PREFIX)/bin/
	cp $< $(PREFIX)/bin/
	
uninstall:
	rm -f $(PREFIX)/bin/$(CANOPENSHELL)