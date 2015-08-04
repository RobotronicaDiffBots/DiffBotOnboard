
# The name of your project (used to name the compiled .hex file)
TARGET = main
CLOCK_RATE = 96000000

# configurable options
OPTIONS = -DUSB_SERIAL

# directory to build in
BUILDDIR = "build"

#************************************************************************
# Toolchain settings
#************************************************************************

# path location for Teensy Loader, teensy_post_compile and teensy_reboot
TOOLSPATH = tools

# path location for the arm-none-eabi compiler
COMPILERPATH = /usr/bin

COREPATH = ./teensy3
LDSCRIPT = $(COREPATH)/mk20dx256.ld

#************************************************************************
# Settings below this point usually do not need to be edited
#************************************************************************

# options needed by many Arduino libraries to configure for Teensy 3.0
OPTIONS += -DF_CPU=$(CLOCK_RATE) -DLAYOUT_US_ENGLISH -DUSING_MAKEFILE -D__MK20DX256__ -DARDUINO=10600 -DTEENSYDUINO=121

# compiler options for C++ only
CXXFLAGS = -std=gnu++0x -felide-constructors -fno-exceptions -fno-rtti

# CPPFLAGS = compiler options for C and C++
CPPFLAGS = -Wall -g -O3 -ffunction-sections -fdata-sections -nostdlib -MMD -mthumb -mcpu=cortex-m4 $(OPTIONS) -I$(COREPATH) -Iinclude

# compiler options for C only
CFLAGS =

# linker options
#LDFLAGS = -O3 -Wl,--gc-sections,--defsym=__rtc_localtime=0 --specs=nano.specs -mthumb -mcpu=cortex-m4 -T$(LDSCRIPT)
LDFLAGS = -O3 -Wl,--gc-sections,--defsym=__rtc_localtime=0 --specs=nano.specs -mthumb -mcpu=cortex-m4 -T$(LDSCRIPT)

# additional libraries to link
LIBS = -lm

# names for the compiler programs
#CC = $(abspath $(COMPILERPATH))/arm-none-eabi-gcc
CC = /home/evangel/QUT/Robotronica_2015/compiler/gcc-arm-none-eabi-4_9-2015q2-20150609/install-native/bin/arm-none-eabi-gcc
#CXX = $(abspath $(COMPILERPATH))/arm-none-eabi-g++
CXX = /home/evangel/QUT/Robotronica_2015/compiler/gcc-arm-none-eabi-4_9-2015q2-20150609/install-native/bin/arm-none-eabi-g++
#OBJCOPY = $(abspath $(COMPILERPATH))/arm-none-eabi-objcopy
OBJCOPY = /home/evangel/QUT/Robotronica_2015/compiler/gcc-arm-none-eabi-4_9-2015q2-20150609/install-native/bin/arm-none-eabi-objcopy
#SIZE = $(abspath $(COMPILERPATH))/arm-none-eabi-size
SIZE = /home/evangel/QUT/Robotronica_2015/compiler/gcc-arm-none-eabi-4_9-2015q2-20150609/install-native/bin/arm-none-eabi-size

# automatically create lists of the sources and objects
# TODO: this does not handle Arduino libraries yet...
TC_FILES := $(wildcard $(COREPATH)/*.c)
TCPP_FILES := $(wildcard $(COREPATH)/*.cpp)
#TCXX_FILES := $(addprefix $(COREPATH)/, $(ARD_SOURCES))
#TC_FILES := $(filter %.c, $(TCXX_FILES))
#TCPP_FILES := $(filter %.cpp, $(TCXX_FILES))

C_FILES := $(wildcard src/*.c)
CPP_FILES := $(wildcard src/*.cpp)

SOURCES := $(C_FILES:.c=.o) $(CPP_FILES:.cpp=.o) $(TC_FILES:.c=.o) $(TCPP_FILES:.cpp=.o)
OBJS := $(foreach src,$(SOURCES), $(BUILDDIR)/$(src))


# the actual makefile rules (all .o files built by GNU make's default implicit rules)

all: hex

build: $(TARGET).elf

hex: $(TARGET).hex

post_compile: $(TARGET).hex
	@$(abspath $(TOOLSPATH))/teensy_post_compile -file="$(basename $<)" -path=$(CURDIR) -tools="$(abspath $(TOOLSPATH))"

reboot:
	@-$(abspath $(TOOLSPATH))/teensy_reboot

upload: post_compile reboot

$(BUILDDIR)/%.o: %.c
	@echo "[CC] $< $@"
	@mkdir -p "$(dir $@)"
	@$(CC) $(CPPFLAGS) $(CFLAGS) -o "$@" -c "$<"

$(BUILDDIR)/%.o: %.cpp
	@echo "[CXX] $< $@"
	@mkdir -p "$(dir $@)"
	@$(CXX) $(CPPFLAGS) $(CXXFLAGS) -o "$@" -c "$<"

$(TARGET).elf: $(OBJS) $(LDSCRIPT)
	@echo "[LD] $@"
	@$(CC) $(LDFLAGS) -o "$@" $(OBJS) $(LIBS)

%.hex: %.elf
	@echo "[HEX] $@"
	@$(SIZE) "$<"
	@$(OBJCOPY) -O ihex -R .eeprom "$<" "$@"

# compiler generated dependency info
-include $(OBJS:.o=.d)

clean:
	@echo Cleaning...
	@rm -rf "$(BUILDDIR)"
	@rm -f "$(TARGET).elf" "$(TARGET).hex"
	
install:
	@echo Downloading teensy3