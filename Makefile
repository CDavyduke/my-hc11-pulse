# Pulse Generator Makefile
#   Copyright (C) 2003 Free Software Foundation, Inc.
#   Written by Stephane Carrez (stcarrez@nerim.fr)	
#
# This file is free software; you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the
# Free Software Foundation; either version 2, or (at your option) any
# later version.
# 
# In addition to the permissions in the GNU General Public License, the
# Free Software Foundation gives you unlimited permission to link the
# compiled version of this file with other programs, and to distribute
# those programs without any restriction coming from the use of this
# file.  (The General Public License restrictions do apply in other
# respects; for example, they cover modification of the file, and
# distribution when not linked into another program.)
# 
# This file is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program; see the file COPYING.  If not, write to
# the Free Software Foundation, 59 Temple Place - Suite 330,
# Boston, MA 02111-1307, USA.
#
SRCDIR=c:/gel

GEL_BASEDIR=$(SRCDIR)

# Use the cme11 board configuration files (compilation flags
# and specific includes).
override TARGET_BOARD=m68hc11-corey

include $(GEL_BASEDIR)/config/make.defs

CSRCS=pulse.c

OBJS=$(CSRCS:.c=.o)

PROGS= pulse.elf

all::	$(PROGS) pulse.s19

pulse.elf:	$(OBJS)
	$(CC) $(LDFLAGS) -o $@ $(OBJS) $(GEL_LIBS)

install::	$(PROGS)
	$(INSTALL) $(PROGS) $(GEL_INSTALL_BIN)
