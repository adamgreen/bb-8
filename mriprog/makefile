# Copyright 2015 Adam Green (https://github.com/adamgreen)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# User can set VERBOSE variable to have all commands echoed to console for debugging purposes.
ifdef VERBOSE
    Q :=
else
    Q := @
endif


#######################################
#  Forwards Declaration of Main Rules
#######################################
.PHONY : all clean

all:
clean:


#  Names of tools for cross-compiling ARMv7-M binaries.
ARMV7M_GCC     := arm-none-eabi-gcc
ARMV7M_GPP     := arm-none-eabi-g++
ARMV7M_AS      := arm-none-eabi-gcc
ARMV7M_LD      := arm-none-eabi-g++
ARMV7M_AR      := arm-none-eabi-ar
ARMV7M_SIZE    := arm-none-eabi-size
ARMV7M_OBJDUMP := arm-none-eabi-objdump
ARMV7M_OBJCOPY := arm-none-eabi-objcopy

#  Names of tools for compiling binaries to run on this host system.
HOST_GCC := gcc
HOST_GPP := g++
HOST_AS  := gcc
HOST_LD  := g++
HOST_AR  := ar

# Handle Windows and *nix differences.
ifeq "$(OS)" "Windows_NT"
    MAKEDIR = mkdir $(subst /,\,$(dir $@))
    REMOVE := del /q
    REMOVE_DIR := rd /s /q
    QUIET := >nul 2>nul & exit 0
    EXE := .exe
else
    MAKEDIR = mkdir -p $(dir $@)
    REMOVE := rm
    REMOVE_DIR := rm -r -f
    QUIET := > /dev/null 2>&1 ; exit 0
    EXE :=
endif

# Flags to use when cross-compiling ARMv7-M binaries.
ARMV7M_GCCFLAGS := -Os -g3 -mcpu=cortex-m3 -mthumb -mthumb-interwork -Wall -Wextra -Werror -Wno-unused-parameter -MMD -MP
ARMV7M_GCCFLAGS += -ffunction-sections -fdata-sections -fno-exceptions -fno-delete-null-pointer-checks -fomit-frame-pointer
ARMV7M_GPPFLAGS := $(ARMV7M_GCCFLAGS) -fno-rtti
ARMV7M_GCCFLAGS += -std=gnu99
ARMV7M_ASFLAGS  := -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -g3 -x assembler-with-cpp -MMD -MP
ARMV7M_LDFLAGS  := -mcpu=cortex-m3 -mthumb -specs=../gcc4mbed/build/startfile.spec -specs=../gcc4mbed/build/nano.specs
ARMV7M_LDFLAGS  += -Wl,--cref,--gc-sections

# Flags to use when compiling binaries to run on this host system.
HOST_GCCFLAGS := -O2 -g3 -Wall -Wextra -Werror -Wno-unused-parameter -MMD -MP
HOST_GCCFLAGS += -ffunction-sections -fdata-sections -fno-common
HOST_GPPFLAGS := $(HOST_GCCFLAGS)
HOST_GCCFLAGS += -std=gnu90
HOST_ASFLAGS  := -g -x assembler-with-cpp -MMD -MP

# Output directories for intermediate object files.
OBJDIR        := obj
ARMV7M_OBJDIR := $(OBJDIR)/armv7-m
HOST_OBJDIR   := $(OBJDIR)/host

# Start out with empty pre-req lists.  Add modules as we go.
ALL_TARGETS  :=

# Start out with an empty header file dependency list.  Add module files as we go.
DEPS :=

# Useful macros.
objs = $(addprefix $2/,$(addsuffix .o,$(basename $(wildcard $1/*.c $1/*.cpp $1/*.S))))
armv7m_objs = $(call objs,$1,$(ARMV7M_OBJDIR))
host_objs = $(call objs,$1,$(HOST_OBJDIR))
add_deps = $(patsubst %.o,%.d,$(HOST_$1_OBJ) $(GCOV_HOST_$1_OBJ))
includes = $(patsubst %,-I%,$1)
define link_exe
	@echo Building $@
	$Q $(MAKEDIR)
	$Q $($1_LD) $($1_LDFLAGS) $^ -o $@
endef
define host_make_app # ,APP2BUILD,app_src_dirs,includes,other_libs
    HOST_$1_APP_OBJ        := $(foreach i,$2,$(call host_objs,$i))
    HOST_$1_APP_EXE        := $1
    DEPS                   += $$(call add_deps,$1_APP)
    ALL_TARGETS += $$(HOST_$1_APP_EXE)
    $$(HOST_$1_APP_EXE) : INCLUDES := $3
    $$(HOST_$1_APP_EXE) : $$(HOST_$1_APP_OBJ) $4
		$$(call link_exe,HOST)
endef
define armv7m_make_app # ,APP2BUILD,app_src_dirs,includes,link_script,other_libs
    ARMV7M_$1_APP_OBJ      := $(foreach i,$2,$(call armv7m_objs,$i))
    ARMV7M_$1_APP_EXE      := $1
    ARMV7M_$1_APP_BIN      := $(basename $1).bin
    DEPS                   += $$(call add_deps,$1_APP)
    ALL_TARGETS += $$(ARMV7M_$1_APP_BIN)
    $$(ARMV7M_$1_APP_EXE) : INCLUDES := $3
    $$(ARMV7M_$1_APP_EXE) : $$(ARMV7M_$1_APP_OBJ) $5
		@echo Building $$@
		$Q $$(MAKEDIR)
		$Q $(ARMV7M_LD) $(ARMV7M_LDFLAGS) -Wl,-Map=$(ARMV7M_OBJDIR)/$(basename $(notdir $1)).map -T$4 $$^ -o $$@
		$Q $(ARMV7M_OBJDUMP) -d -f -M reg-names-std --demangle >$(ARMV7M_OBJDIR)/$(basename $(notdir $1)).disasm $$@
    $$(ARMV7M_$1_APP_BIN) : $$(ARMV7M_$1_APP_EXE)
		@echo Extracting $$@
		$Q $(ARMV7M_OBJCOPY) -O binary $$< $$@
		$Q $(ARMV7M_SIZE) $$<
endef


#######################################
# bin2h Executable
$(eval $(call host_make_app,bin2h,bin2h-src,,))

#######################################
# LPC1768 bootloader
$(eval $(call armv7m_make_app,boot-lpc1768.elf,boot-lpc1768,includes boot-lpc1768,boot-lpc1768/LPC1768.ld,))

#######################################
# mriprog Executable
$(eval $(call host_make_app,mriprog,mriprog-src,includes generated,))

#######################################
# mriprog needs bootloaders in
# generated header files.
$(HOST_OBJDIR)/mriprog-src/main.o : generated/boot-lpc1768.h

generated/boot-lpc1768.h : bin2h boot-lpc1768.bin
	@echo Auto generating $@
	$Q $(MAKEDIR)
	$Q bin2h boot-lpc1768.bin $@ g_lpc1768Bootloader



#######################################
#  Actual Definition of Main Rules
#######################################
all : $(ALL_TARGETS)

clean :
	@echo Cleaning mriprog
	$Q $(REMOVE_DIR) $(OBJDIR) $(QUIET)
	$Q $(REMOVE_DIR) generated/ $(QUIET)
	$Q $(REMOVE) $(ALL_TARGETS) $(QUIET)
	$Q $(REMOVE) *.elf $(QUIET)


# *** Pattern Rules ***
$(ARMV7M_OBJDIR)/%.o : %.c
	@echo Compiling $<
	$Q $(MAKEDIR)
	$Q $(ARMV7M_GCC) $(ARMV7M_GCCFLAGS) $(call includes,$(INCLUDES)) -c $< -o $@

$(HOST_OBJDIR)/%.o : %.c
	@echo Compiling $<
	$Q $(MAKEDIR)
	$Q $(EXTRA_COMPILE_STEP)
	$Q $(HOST_GCC) $(HOST_GCCFLAGS) $(call includes,$(INCLUDES)) -c $< -o $@


# *** Pull in header dependencies if not performing a clean build. ***
ifneq "$(findstring clean,$(MAKECMDGOALS))" "clean"
    -include $(DEPS)
endif
