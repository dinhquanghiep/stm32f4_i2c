# ------------------------------------------------------------------------------
# Toolchain Configuration
# ------------------------------------------------------------------------------
# TOOLCHAIN_DIR				:= D:/Work/6._IDE/gcc-arm-none-eabi/bin/
TOOLCHAIN_DIR			:= 
TOOLCHAIN_PREFIX 	:= arm-none-eabi-
# TOOLCHAIN_PREFIX 	:= arm-atollic-eabi-
CC								:= $(TOOLCHAIN_DIR)$(TOOLCHAIN_PREFIX)gcc
# CC								:= $(TOOLCHAIN_DIR)$(TOOLCHAIN_PREFIX)g++
AR								:= $(TOOLCHAIN_DIR)$(TOOLCHAIN_PREFIX)ar
RANDLIB						:= $(TOOLCHAIN_DIR)$(TOOLCHAIN_PREFIX)ranlib
OBJCOPY						:= $(TOOLCHAIN_DIR)$(TOOLCHAIN_PREFIX)objcopy
SIZE							:= $(TOOLCHAIN_DIR)$(TOOLCHAIN_PREFIX)size

CC_FLAG 					:= -std=c99 -c -Wall -fdiagnostics-color=auto
# CC_FLAG 					:= -c -Wall -fdiagnostics-color=auto
CC_FLAG 					+= -Wno-unused-function
CC_FLAG 					+= -ffunction-sections -fdata-sections
CC_FLAG_DEBUG			:= -g -Os
CC_FLAG_RELEASE 	:= -Os
CC_FLAG_LIBRARY 	:= -I
# ------------------------------------------------------------------------------
# Project Configuration
# ------------------------------------------------------------------------------
PROJ_DIR	:= .
PROJ_NAME	:= stm32f4_i2c

MCU_DEFS 			:= -DSTM32F4 -DSTM32F407 -DSTM32F407xx -DSTM32F40_41xxx 
MCU_DEFS			+= -DUSE_STDPERIPH_DRIVER -D_DEFAULT_SOURCE -DHSE_VALUE=8000000
# For DSP library
MCU_DEFS 			+= -DARM_MATH_CM4 -D__FPU_PRESENT=1
MCU_SETTINGS 	:= -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 

INC_DIRS	:= $(CC_FLAG_LIBRARY)D:/Work/6._IDE/vcs/arm/lib/cmsis/cores
INC_DIRS	+= $(CC_FLAG_LIBRARY)D:/Work/6._IDE/vcs/arm/lib/cmsis/variant
INC_DIRS	+= $(CC_FLAG_LIBRARY)$(PROJ_DIR)/spl/inc
INC_DIRS	+= $(CC_FLAG_LIBRARY)$(PROJ_DIR)/spl/src
INC_DIRS	+= $(CC_FLAG_LIBRARY)$(PROJ_DIR)/inc
INC_DIRS	+= $(CC_FLAG_LIBRARY)D:/Work/6._IDE/gcc-arm-none-eabi/arm-none-eabi/include
# INC_DIRS	+= $(CC_FLAG_LIBRARY)"C:/Program Files (x86)/Atollic/TrueSTUDIO for STM32 9.0.1/ARMTools/arm-atollic-eabi/include"
# STARTUP_DIR, LINKER_SCRIPT_FILE, SYSTEM_CONFIG_DIR
LINKER_STARTUP_DIR	:= $(PROJ_DIR)/etc 
ARM_DSP_LIB					:= D:/Work/6._IDE/vcs/arm/lib/cmsis/dsp_lib/GCC

PROJ_OUTPUT_DIR	:= $(PROJ_DIR)/output

PROJ_SRC_DIR		:= $(PROJ_DIR)/src
PROJ_SRC_FILES	:= $(wildcard $(PROJ_SRC_DIR)/*.c)
PROJ_OBJ_DIR 		:= $(PROJ_OUTPUT_DIR)/src
PROJ_OBJ_FILES	:= $(patsubst $(PROJ_SRC_DIR)/%.c, $(PROJ_OBJ_DIR)/%.o, $(PROJ_SRC_FILES))
PROJ_DEP_FILES	:= $(patsubst $(PROJ_SRC_DIR)/%.c, $(PROJ_OBJ_DIR)/%.d, $(PROJ_SRC_FILES))

SPL_SRC_DIR			:= $(PROJ_DIR)/spl/src
SPL_SRC_FILES		:= $(wildcard $(SPL_SRC_DIR)/*.c)
SPL_OBJ_DIR 		:= $(PROJ_OUTPUT_DIR)/lib
SPL_OBJ_FILES		:= $(patsubst $(SPL_SRC_DIR)/%.c, $(SPL_OBJ_DIR)/%.o, $(SPL_SRC_FILES))
SPL_DEP_FILES		:= $(patsubst $(SPL_SRC_DIR)/%.c, $(SPL_OBJ_DIR)/%.d, $(SPL_SRC_FILES))
# ------------------------------------------------------------------------------
# Configuration Flags for compiler, assembler and linker
# ------------------------------------------------------------------------------
CC_FLAGS 					:= $(CC_FLAG) $(CC_FLAG_DEBUG) $(MCU_SETTINGS) $(MCU_DEFS) $(INC_DIRS)
AS_FLAGS 					:= $(CC_FLAGS) -x assembler-with-cpp
LD_FLAGS 					:= $(MCU_SETTINGS) -Wl,--cref -Xlinker --gc-sections -specs=nano.specs -specs=rdimon.specs 
AR_FLAGS 					:= rc
SIZE_FLAGS 				:= --format=berkeley --radix=10
OBJCOPY_FLAGS_HEX	:= -O ihex
OBJCOPY_FLAGS_BIN	:= -O binary
# ------------------------------------------------------------------------------
# Configuration Flags for IAR compiler
# ------------------------------------------------------------------------------
IAR_COMPILER_DIR 	:= "C:/Program Files (x86)/IAR Systems/Embedded Workbench 8.1/common/bin/IarBuild.exe"
IAR_PROGRAM_DIR		:= "C:\Program Files (x86)\IAR Systems\Embedded Workbench 8.1\common\bin\IarIdePm.exe"
IAR_PROJECT_FILE	:= $(PROJ_DIR)/IAR/$(PROJ_NAME).ewp
IAR_FLAGS					:= -make Release -log all -parallel 4
IAR_F_BUILD				:= -build Release -log all -parallel 4
IAR_WP_FILE				:= $(PROJ_DIR)/IAR/$(PROJ_NAME).eww

# ------------------------------------------------------------------------------
# Command line interface
# ------------------------------------------------------------------------------
.PHONY: run makelibrary buildsource link iar iarfbuild\
cleanall cleansrc cleanlib

run: create_output_dir makelibrary buildsource link hex binary size

create_output_dir:
	@mkdir -p $(PROJ_OUTPUT_DIR)
	@mkdir -p $(PROJ_OBJ_DIR)
	@mkdir -p $(SPL_OBJ_DIR)

iar:
	@echo "Compiled by IAR....."
	$(IAR_COMPILER_DIR) $(IAR_PROJECT_FILE) $(IAR_FLAGS)

iarfbuild:
	@echo "Clean all output from IAR....."
	$(IAR_COMPILER_DIR) $(IAR_PROJECT_FILE) $(IAR_F_BUILD)

iaropen:
	@echo "Open IAR Workspace....."
	@start $(IAR_PROGRAM_DIR) $(IAR_WP_FILE)

cleanall: 
	@echo "Remove output files..."
	@rm -rf $(PROJ_OUTPUT_DIR)/*.* $(SPL_OBJ_DIR)/*.* $(PROJ_OBJ_DIR)/*.*

cleansrc: 
	@echo "Remove output source files..."
	@rm -rf $(PROJ_OBJ_DIR)/*.*

cleanlib: 
	@echo "Remove output library files..."
	@rm -rf $(SPL_OBJ_DIR)/*.*

makelibrary: $(PROJ_OUTPUT_DIR)/lib$(PROJ_NAME).a
$(PROJ_OUTPUT_DIR)/lib$(PROJ_NAME).a: $(SPL_OBJ_FILES) 
	$(AR) $(AR_FLAGS) $@ $+
	$(RANDLIB) $@
	
$(SPL_OBJ_DIR)/%.o: $(SPL_SRC_DIR)/%.c
	$(CC) $(CC_FLAGS) -MMD -MP -o $@ $<

buildsource: $(PROJ_OBJ_DIR)/startup_stm32f40_41xxx.o $(PROJ_OBJ_FILES)
$(PROJ_OBJ_DIR)/startup_stm32f40_41xxx.o: $(PROJ_DIR)/etc/startup_stm32f40_41xxx.s
	$(CC) $(AS_FLAGS) -o $@ $+

$(PROJ_OBJ_DIR)/%.o: $(PROJ_SRC_DIR)/%.c
	$(CC) $(CC_FLAGS) -MMD -MP -o $@ $<

-include $(PROJ_DEP_FILES)
-include $(SPL_DEP_FILES)

link: $(PROJ_OUTPUT_DIR)/$(PROJ_NAME).elf
$(PROJ_OUTPUT_DIR)/%.elf: $(PROJ_OBJ_FILES) $(PROJ_OBJ_DIR)/startup_stm32f40_41xxx.o \
$(PROJ_OUTPUT_DIR)/lib$(PROJ_NAME).a $(PROJ_DIR)/etc/STM32F417IGHx_FLASH.ld
	$(CC) $(LD_FLAGS) -o $@ -T$(PROJ_DIR)/etc/STM32F417IGHx_FLASH.ld -Wl,-Map=$(PROJ_OUTPUT_DIR)/$(PROJ_NAME).map \
$(PROJ_OBJ_FILES) $(PROJ_OBJ_DIR)/startup_stm32f40_41xxx.o $(PROJ_OUTPUT_DIR)/lib$(PROJ_NAME).a \
$(ARM_DSP_LIB)/libarm_cortexM4lf_math.a

size:
	@echo "----------------Program size----------------"
	@$(SIZE) $(SIZE_FLAGS) $(PROJ_OUTPUT_DIR)/*.elf
	@$(SIZE) $(SIZE_FLAGS) $(PROJ_OUTPUT_DIR)/*.elf > $(PROJ_OUTPUT_DIR)/$(PROJ_NAME)_size.txt

hex: $(PROJ_OUTPUT_DIR)/$(PROJ_NAME).hex
%.hex: %.elf
	@$(OBJCOPY) $(OBJCOPY_FLAGS_HEX) $< $@

binary: $(PROJ_OUTPUT_DIR)/$(PROJ_NAME).bin
%.bin: %.elf
	@$(OBJCOPY) $(OBJCOPY_FLAGS_BIN) $< $@
