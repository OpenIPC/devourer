#---------------------------------------------------------------------------
# Author: Cedric Adjih, Hana Baccouch
#---------------------------------------------------------------------------
# Copyright 2015-2017 Inria
#
# Permission is hereby granted, free of charge, to any person obtaining
# a copy of this software and associated documentation files (the
# "Software"), to deal in the Software without restriction, including
# without limitation the rights to use, copy, modify, merge, publish,
# distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so, subject to
# the following conditions:
#
# The above copyright notice and this permission notice shall be
# included in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
# NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
# LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
# OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
# WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#---------------------------------------------------------------------------

-include Makefile.local

#--------------------------------------------------

ifeq ($(WITH_SANITIZE),1)
SANITIZE=-fsanitize=leak -fsanitize=address -fsanitize=undefined  -static-libasan -static-libubsan -fno-omit-frame-pointer
else
SANTIZE=
endif


CFLAGS += -Wall -g3 -fPIC -std=c99 -O3 $(SANITIZE)

#------------------------------

# TODO : swif_linear-code.c 
SRCS += swif_api.c swif_prng.c swif_coding_coefficients.c swif_symbol.c swif_rlc_api.c
SRCS += swif_full_symbol.c
SRCS += $(EXTRA_SRCS)

HEADERS += $(SRCS:.c=.h) swif_general.h swif_includes.h swif_rlc_cb.h
HEADERS += $(EXTRA_HEADERS)

OBJS = $(SRCS:.c=.o)

#---------------------------------------------------------------------------
# Main targets
#---------------------------------------------------------------------------

all: libswif.a

#---------------------------------------------------------------------------
# Dependencies
# http://stackoverflow.com/questions/97338/gcc-dependency-generation-for-a-different-output-directory
#---------------------------------------------------------------------------

DEPS = $(SRCS:.c=.d)

-include $(DEPS)

%.d : %.c %.h
	$(CC) $(CFLAGS) -MF"$@" -MG -MM -MP -MT"$@" -MT"$(<:.c=.o)" "$<"

#---------------------------------------------------------------------------
# Library
#---------------------------------------------------------------------------

libswif.a: ${OBJS}
	   ${AR} cru $@ ${OBJS}

#---------------------------------------------------------------------------
# Testing
#---------------------------------------------------------------------------

test-coded-packet: test-coded-packet.o libswif.a
	${CC} ${CFLAGS} -o $@ $< -llc -L.

#---------------------------------------------------------------------------
# Documentation
#---------------------------------------------------------------------------

doc:
	doxygen Doxyfile
	sensible-browser html/index.html

#---------------------------------------------------------------------------
# Cleaning
#---------------------------------------------------------------------------

clean:
	rm -f *.a *.so *.o *.d *~
	rm -f test-coded-packet

really-clean: clean

#---------------------------------------------------------------------------
