.PHONY: all clean flash

# setup
COMPILE_OPTS = -mcpu=cortex-m3 -march=armv7-m -mthumb -Wall -Wextra -Wconversion -g -O2 -std=c11
INCLUDE_DIRS = -I include -I lib/inc
LIBRARY_DIRS = -L lib

CC = arm-none-eabi-gcc
CFLAGS = $(COMPILE_OPTS) $(INCLUDE_DIRS)

CXX = arm-none-eabi-g++
CXXFLAGS = $(COMPILE_OPTS) $(INCLUDE_DIRS)

AS = arm-none-eabi-gcc
ASFLAGS = $(COMPILE_OPTS) -c

LD = arm-none-eabi-gcc
LDFLAGS = $(COMPILE_OPTS) -Wl,--gc-sections,-Map=$@.map,-cref $(INCLUDE_DIRS) $(LIBRARY_DIRS) -T STM32F103RCTx_FLASH.ld

OBJCP = arm-none-eabi-objcopy
OBJCPFLAGS = -O binary

AR = arm-none-eabi-ar
ARFLAGS = cr

MAIN_OUT = main
MAIN_OUT_ELF = $(MAIN_OUT).elf
MAIN_OUT_BIN = $(MAIN_OUT).bin

SRC_FILES = $(wildcard src/*.c) $(wildcard lib/src/*.c)
ASM_FILES = $(wildcard asm/*.s)
OBJ_FILES = $(SRC_FILES:.c=.o) $(ASM_FILES:.s=.o) src/nds_cart_key.o


# all
all: $(MAIN_OUT_ELF) $(MAIN_OUT_BIN)

# main
$(MAIN_OUT_ELF): $(OBJ_FILES)
	$(LD) $(LDFLAGS) $^ -o $@

src/nds_cart_key.o: biosnds7.rom biosdsi7.rom
		./gen_keys.sh | $(CC) $(CFLAGS) -x c -c -o $@ -

$(MAIN_OUT_BIN): $(MAIN_OUT_ELF)
	$(OBJCP) $(OBJCPFLAGS) $< $@

clean:
	rm -f src/*.o lib/src/*.o asm/*.o $(MAIN_OUT_ELF) $(MAIN_OUT_BIN)

flash: $(MAIN_OUT_BIN)
	stm32flash -w $(MAIN_OUT_BIN) COM3
