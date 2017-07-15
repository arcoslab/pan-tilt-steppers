##
## Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
## Copyright (C) 2017 Ariel Mora <ariel.moraj@gmail.com>
##
## This library is free software: you can redistribute it and/or modify
## it under the terms of the GNU Lesser General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This library is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU Lesser General Public License for more details.
##
## You should have received a copy of the GNU Lesser General Public License
## along with this library.  If not, see <http://www.gnu.org/licenses/>.
##

## put your main source file here
BINARY = pantilt2

## put your project objects
SRC :=

## put the correct interface and board
OOCD_INTERFACE = stlink-v2
OOCD_BOARD = stm32f4discovery

## put the linker script of your board
LDSCRIPT = ../libopencm3-plus/lib/libopencm3_plus_$(OOCD_BOARD).ld

include ../Makefile.include
