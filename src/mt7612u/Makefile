# Standalone build of the subtree. The project's CMakeLists.txt also compiles
# the library sources (DEVOURER_MT7612U=ON); this Makefile stays because it
# builds `bringup` and the offline tests, which CMake does not.
#
# The library is C++ (see internal.h): MSVC has no <pthread.h>, and devourer
# builds Windows first-class, so the sync and timing primitives are std:: types.
# Both builds therefore compile the SAME sources in the SAME language — there is
# no second compilation mode that can rot unnoticed.
#
# tests/api_link.c is deliberately still C. It includes only the public header,
# whose extern "C" guard is what keeps the exported surface callable from C;
# compiling it with $(CC) is what actually tests that, and it would silently
# stop testing it if it became C++ along with everything else.
CC       ?= gcc
CXX      ?= g++
CXXFLAGS ?= -O2 -g -std=c++20 -Wall -Wextra -Wno-unused-parameter \
            -Wno-missing-field-initializers
CFLAGS   ?= -O2 -g -Wall -Wextra
# -MMD -MP: without header deps, editing internal.h leaves stale objects with
# the old struct layout linked against new ones. That corrupts the device
# struct at runtime and presents as a wild pointer, not a build problem.
DEPFLAGS = -MMD -MP
INCS     = -I. -Iinclude
LDLIBS   = -lusb-1.0 -lpthread
# Everything here EXCEPT the devourer-side backend. Mt7612uRadio.cpp implements
# devourer's IRadio and includes its headers (IRadio.h, logger.h, DeviceConfig.h),
# which this standalone build has no include path for and should not - the point
# of this Makefile is that the library builds with nothing but libusb. CMake
# compiles that file; this does not.
LIBSRCS  = $(filter-out Mt7612uRadio.cpp,$(wildcard *.cpp))
TESTS    = tests/api_link tests/frame_shape tests/field_macros tests/log_sink
CXXSRCS  = $(LIBSRCS) tools/bringup.cpp tests/frame_shape.cpp tests/field_macros.cpp \
           tests/log_sink.cpp
CSRCS    = tests/api_link.c
OBJS     = $(CXXSRCS:.cpp=.o) $(CSRCS:.c=.o)
LIBOBJS  = $(LIBSRCS:.cpp=.o)
DEPS     = $(OBJS:.o=.d)

all: bringup $(TESTS)

# Linked with $(CXX): the library objects are C++ and need libstdc++, including
# on the api_link path whose own object is C.
bringup: $(LIBOBJS) tools/bringup.o
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDLIBS)

# Links against the public header only, so a declaration with no definition
# is a link error here rather than a surprise at the first caller. Its object is
# compiled as C, so this also fails if the extern "C" guard ever regresses.
tests/api_link: $(LIBOBJS) tests/api_link.o
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDLIBS)

tests/frame_shape: $(LIBOBJS) tests/frame_shape.o
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDLIBS)

tests/field_macros: $(LIBOBJS) tests/field_macros.o
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDLIBS)

tests/log_sink: $(LIBOBJS) tests/log_sink.o
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDLIBS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(DEPFLAGS) $(INCS) -c -o $@ $<

%.o: %.c
	$(CC) $(CFLAGS) $(DEPFLAGS) $(INCS) -c -o $@ $<

# No hardware and no privileges needed: safe to run anywhere. `bringup` is a
# dependency but is not run: nothing else compiles it, so without it here a
# change that breaks a gate ships and no test notices.
check: $(TESTS) bringup
	@for t in $(TESTS); do ./$$t || exit 1; done

clean:
	rm -f $(OBJS) $(DEPS) bringup $(TESTS)
-include $(DEPS)
.PHONY: all check clean
