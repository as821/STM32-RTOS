
# Put your source files here (or *.c, etc)
SRCS=src/*.c src/*.s src/*.c src/*.s

# Binaries will be generated with this name (.elf, .bin, .hex, etc)
PROJ_NAME=rtos
BUILD_DIR=bin

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy

CFLAGS  = -g -O2 -Wall -ffreestanding -nostdlib -Tstm32_flash.ld
CFLAGS += -mthumb -mcpu=cortex-m4 -mthumb-interwork
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -I. -Iinclude -Iinc/sys_headers # -Iinc/cmsis_headers

OBJS = $(SRCS:.c=.o)




.PHONY: proj

all: proj

proj: $(BUILD_DIR)/$(PROJ_NAME).elf

$(BUILD_DIR)/$(PROJ_NAME).elf: $(SRCS)
	$(CC) $(CFLAGS) $^ -o $@ 
	$(OBJCOPY) -O ihex $(BUILD_DIR)/$(PROJ_NAME).elf $(BUILD_DIR)/$(PROJ_NAME).hex
	$(OBJCOPY) -O binary $(BUILD_DIR)/$(PROJ_NAME).elf $(BUILD_DIR)/$(PROJ_NAME).bin

clean:
	rm -f *.o $(BUILD_DIR)/$(PROJ_NAME).elf $(BUILD_DIR)/$(PROJ_NAME).hex $(BUILD_DIR)/$(PROJ_NAME).bin

# Flash the STM32F4
burn: proj
	/usr/local/bin/st-flash write $(BUILD_DIR)/$(PROJ_NAME).bin 0x8000000
