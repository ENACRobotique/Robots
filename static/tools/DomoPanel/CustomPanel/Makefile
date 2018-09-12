#Makefile modify from http://forum.arduino.cc/index.php/topic,37734.0.html
#Modifications:
#add pins_arduino.h path (-I)
#Remove old pins_arduino.c from src
#Remove useless avrdude compil option
#Use a build dir (not compile in arduino src
#Add a SRC_PATH var => you can change it !
#Add a build checker

# Arduino 0018 Makefile
# Arduino adaptation by mellis, eighthave, oli.keller
#
# This makefile allows you to build sketches from the command line
# without the Arduino environment (or Java).
#
# Detailed instructions for using the makefile:
#
#  1. Copy this sources file in src folder.
#
#  2. Modify the line containg "INSTALL_DIR" to point to the directory that
#     contains the Arduino installation (for example, under Mac OS X, this
#     might be /Applications/arduino-0012).
#
#  3. Modify the line containing "PORT" to refer to the filename
#     representing the USB or serial connection to your Arduino board
#     (e.g. PORT = /dev/tty.USB0).  If the exact name of this file
#     changes, you can use * as a wildcard (e.g. PORT = /dev/tty.usb*).
#
#  4. Set the line containing "MCU" to match your board's processor.
#     Older one's are atmega8 based, newer ones like Arduino Mini, Bluetooth
#     or Diecimila have the atmega168.  If you're using a LilyPad Arduino,
#     change F_CPU to 8000000.
#
#  5. At the command line, change to the directory containing your
#     program's file and the makefile.
#
#  6. Type "make" and press enter to compile/verify your program.
#
#  7. Type "make upload", reset your Arduino board, and press enter to
#     upload your program to the Arduino board.
#
# $Id$

INSTALL_DIR = /usr/share/arduino

TARGET = $(notdir $(CURDIR))
BUILD_PATH= build
SRC_PATH= src
#Libraries
INCLUDE_EXTERN_SRC_PATH=../../libraries/RGBmatrixDriver/Adafruit_GFX_Library \
					         ../../libraries/RGBmatrixDriver/RGB_matrix_Panel_master\
					         ../../libraries/SPI

PORT = /dev/ttyACM*
#UPLOAD_RATE = 57600
AVRDUDE_PROGRAMMER = arduino #stk500v1
#MCU = atmega168
MCU = atmega328p
F_CPU = 16000000

############################################################################
# Below here nothing should be changed...
.init_build:
	@if [ ! -d $(BUILD_PATH) ]; then \
		mkdir $(BUILD_PATH); \
	fi

VERSION=105
ARDUINO = $(INSTALL_DIR)/hardware/arduino/cores/arduino
#AVR_TOOLS_PATH = $(INSTALL_DIR)/hardware/tools/avr/bin
AVR_TOOLS_PATH = /usr/bin
AVRDUDE_PATH = $(INSTALL_DIR)/hardware/tools
C_MODULES =  \
$(ARDUINO)/wiring_pulse.c \
$(ARDUINO)/wiring_analog.c \
$(ARDUINO)/wiring.c \
$(ARDUINO)/wiring_digital.c \
$(ARDUINO)/WInterrupts.c \
$(ARDUINO)/wiring_shift.c \
$(wildcard $(addsuffix /*.c,$(INCLUDE_EXTERN_SRC_PATH)) )
# end of C_MODULES

CXX_MODULES = \
$(ARDUINO)/Tone.cpp \
$(ARDUINO)/main.cpp \
$(ARDUINO)/WMath.cpp \
$(ARDUINO)/Print.cpp \
$(ARDUINO)/HardwareSerial.cpp \
$(wildcard $(addsuffix /*.cpp,$(INCLUDE_EXTERN_SRC_PATH)) )
# end of CXX_MODULES

CXX_APP = $(wildcard $(SRC_PATH)/*.cpp)
MODULES = $(C_MODULES) $(CXX_MODULES)
SRC = $(C_MODULES)
CXXSRC = $(CXX_MODULES) $(CXX_APP)
FORMAT = ihex


# Name of this Makefile (used for "make depend").
MAKEFILE = Makefile

# Debugging format.
# Native formats for AVR-GCC's -g are stabs [default], or dwarf-2.
# AVR (extended) COFF requires stabs, plus an avr-objcopy run.
#DEBUG = stabs
DEBUG =

OPT = s

# Place -D or -U options here
CDEFS = -DF_CPU=$(F_CPU)L -DARDUINO=$(VERSION)
CXXDEFS = -DF_CPU=$(F_CPU)L -DARDUINO=$(VERSION)

# Place -I options here
CINCS = -I$(ARDUINO) -I$(INSTALL_DIR)/hardware/arduino/variants/standard/ \
$(addprefix -I,$(INCLUDE_EXTERN_SRC_PATH)) 
CXXINCS = $CINCS

# Compiler flag to set the C Standard level.
# c89   - "ANSI" C
# gnu89 - c89 plus GCC extensions
# c99   - ISO C99 standard (not yet fully implemented)
# gnu99 - c99 plus GCC extensions
#CSTANDARD = -std=gnu99
CDEBUG = -g$(DEBUG)
#CWARN = -Wall -Wstrict-prototypes
#CWARN = -Wall   # show all warnings
CWARN = -w      # suppress all warnings
####CTUNING = -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CTUNING = -ffunction-sections -fdata-sections
CXXTUNING = -fno-exceptions -ffunction-sections -fdata-sections
#CEXTRA = -Wa,-adhlns=$(<:.c=.lst)

CFLAGS = $(CDEBUG) -O$(OPT) $(CWARN) $(CTUNING) $(CDEFS) $(CINCS) $(CSTANDARD) $(CEXTRA)
CXXFLAGS = $(CDEBUG) -O$(OPT) $(CWARN) $(CXXTUNING) $(CDEFS) $(CINCS)
#ASFLAGS = -Wa,-adhlns=$(<:.S=.lst),-gstabs
LDFLAGS = -O$(OPT) -lm -Wl,--gc-sections


# Programming support using avrdude. Settings and variables.
AVRDUDE_PORT = $(PORT)
AVRDUDE_WRITE_FLASH = -U flash:w:$(BUILD_PATH)/$(TARGET).hex

#AVRDUDE_FLAGS = -V -F -C $(INSTALL_DIR)/hardware/tools/avr/etc/avrdude.conf \

AVRDUDE_FLAGS = -V -p $(MCU) -P $(AVRDUDE_PORT) -c $(AVRDUDE_PROGRAMMER) #-b $(UPLOAD_RATE)

# Program settings
CC = $(AVR_TOOLS_PATH)/avr-gcc
CXX = $(AVR_TOOLS_PATH)/avr-g++
LD = $(AVR_TOOLS_PATH)/avr-gcc
OBJCOPY = $(AVR_TOOLS_PATH)/avr-objcopy
OBJDUMP = $(AVR_TOOLS_PATH)/avr-objdump
AR  = $(AVR_TOOLS_PATH)/avr-ar
SIZE = $(AVR_TOOLS_PATH)/avr-size
NM = $(AVR_TOOLS_PATH)/avr-nm
AVRDUDE = $(AVRDUDE_PATH)/avrdude
REMOVE = rm -f
MV = mv -f

# Define all object files.
OBJ = $(SRC:.c=.o) $(CXXSRC:.cpp=.o) $(ASRC:.S=.o)
OBJ_MODULES = $(C_MODULES:.c=.o) $(CXX_MODULES:.cpp=.o)

# Define all listing files.
LST = $(ASRC:.S=.lst) $(CXXSRC:.cpp=.lst) $(SRC:.c=.lst)

# Combine all necessary flags and optional flags.
# Add target processor to flags.
ALL_CFLAGS = $(CFLAGS) -mmcu=$(MCU)
ALL_CXXFLAGS = $(CXXFLAGS) -mmcu=$(MCU)
ALL_ASFLAGS = -x assembler-with-cpp $(ASFLAGS) -mmcu=$(MCU)
ALL_LDFLAGS = $(LDFLAGS) -mmcu=$(MCU)

# Default target.
all: build sizeafter

build: .init_build elf hex

elf: .init_build $(BUILD_PATH)/$(TARGET).elf
hex: .init_build $(BUILD_PATH)/$(TARGET).hex
eep: .init_build $(BUILD_PATH)/$(TARGET).eep
lss: .init_build $(BUILD_PATH)/$(TARGET).lss
sym: .init_build $(BUILD_PATH)/$(TARGET).sym

# Program the device.  
upload: .init_build $(BUILD_PATH)/$(TARGET).hex
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH)


     # Display size of file.
HEXSIZE = $(SIZE) --target=$(FORMAT) $(BUILD_PATH)/$(TARGET).hex
ELFSIZE = $(SIZE)  $(BUILD_PATH)/$(TARGET).elf
sizebefore:
	@if [ -f $(BUILD_PATH)/$(TARGET).elf ]; then echo; echo $(MSG_SIZE_BEFORE); $(HEXSIZE); echo; fi

sizeafter:
	@if [ -f $(BUILD_PATH)/$(TARGET).elf ]; then echo; echo $(MSG_SIZE_AFTER); $(HEXSIZE); echo; fi


# Convert ELF to COFF for use in debugging / simulating in AVR Studio or VMLAB.
COFFCONVERT=$(OBJCOPY) --debugging \
--change-section-address .data-0x800000 \
--change-section-address .bss-0x800000 \
--change-section-address .noinit-0x800000 \
--change-section-address .eeprom-0x810000


coff: $(BUILD_PATH)/$(TARGET).elf
	$(COFFCONVERT) -O coff-avr $(BUILD_PATH)/$(TARGET).elf $(TARGET).cof


extcoff: $(TARGET).elf
	$(COFFCONVERT) -O coff-ext-avr $(BUILD_PATH)/$(TARGET).elf $(TARGET).cof


.SUFFIXES: .elf .hex .eep .lss .sym

.elf.hex:
	$(OBJCOPY) -O $(FORMAT) -R .eeprom $< $@

.elf.eep:
	$(OBJCOPY) -O $(FORMAT) -j .eeprom --set-section-flags=.eeprom="alloc,load" \
	--no-change-warnings \
	--change-section-lma .eeprom=0 $< $@

# Create extended listing file from ELF output file.
.elf.lss:
	$(OBJDUMP) -h -S $< > $@

# Create a symbol table from ELF output file.
.elf.sym:
	$(NM) -n $< > $@

     # Link: create ELF output file from library.
$(BUILD_PATH)/$(TARGET).elf: $(CXX_APP:.cpp=.o) $(BUILD_PATH)/core.a
	$(LD) $(ALL_LDFLAGS) -o $@ $(addprefix $(BUILD_PATH)/,$(notdir $(CXX_APP:.cpp=.o))) $(BUILD_PATH)/core.a

$(BUILD_PATH)/core.a: $(OBJ_MODULES)	
	@for i in $(addprefix $(BUILD_PATH)/,$(notdir $(OBJ_MODULES))); do \
		echo $(AR) rcs $(BUILD_PATH)/core.a $$i; \
		$(AR) rcs $(BUILD_PATH)/core.a $$i; \
	done

# Compile: create object files from C++ source files.
.cpp.o:
	$(CXX) -c $(ALL_CXXFLAGS) $< -o $(BUILD_PATH)/$(notdir $@)

# Compile: create object files from C source files.
.c.o:
	$(CC) -c $(ALL_CFLAGS) $< -o $(BUILD_PATH)/$(notdir $@) 

# Compile: create assembler files from C source files.
.c.s:
	$(CC) -S $(ALL_CFLAGS) $< -o $(BUILD_PATH)/$(notdir $@)

# Assemble: create object files from assembler source files.
.S.o:
	$(CC) -c $(ALL_ASFLAGS) $< -o $(BUILD_PATH)/$(notdir $@)

# Automatic dependencies
%.d: %.c
	$(CC) -M $(ALL_CFLAGS) $< | sed "s;$(notdir $*).o:;$*.o $*.d:;" > $@

%.d: %.cpp
	$(CXX) -M $(ALL_CXXFLAGS) $< | sed "s;$(notdir $*).o:;$*.o $*.d:;" > $@


# Target: clean project.
clean:
	$(REMOVE) $(BUILD_PATH)/*

.PHONY: all build elf hex eep lss sym program coff extcoff clean sizebefore sizeafter .init_build

#include $(SRC:.c=.d)
#include $(CXXSRC:.cpp=.d)
