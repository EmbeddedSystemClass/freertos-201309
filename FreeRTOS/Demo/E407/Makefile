#
# Makefile - 
#
# Developed by Werner Almesberger for Actility S.A., and
# licensed under LGPLv2 by Actility S.A.
#

SHELL = /bin/bash

NAME = demo

ARCH = arm-linux-gnueabi
CC = $(ARCH)-gcc
OBJCOPY = $(ARCH)-objcopy

LD_SCRIPT = stm32_flash.ld
STARTUP_S = startup_stm32f40xx.s
SYSTEM_C = system_stm32f4xx.c

CFLAGS = -Wall -mcpu=cortex-m4 -march=armv7e-m -mthumb \
	-DSTM32F4XX \
	-I$(DEMO_DIR)/Project/Core/Devices/STM32F4xx \
	-I$(DEMO_DIR)/Libraries/STM32F4xx_StdPeriph_Driver/inc \
	-I$(DEMO_DIR)/Libraries/CMSIS/Device/ST/STM32F4xx/Include \
	-I$(DEMO_DIR)/Libraries/CMSIS/Include

LDFLAGS = -nostartfiles -T$(LD_SCRIPT)

SRC = main.c $(SYSTEM_C)

# .../ARM_CM3/port.c (and not ARM_CM4F/port.c) because the latter uses
# unsupported instructions

OBJS = $(SRC:.c=.o)

.PHONY:	all dfu clean spotless

# ----- Verbosity control -----------------------------------------------------

CC_normal	:= $(CC)
CP_normal	:= cp
BUILD_normal	:=
DEPEND_normal	:= $(CPP) $(CFLAGS) -MM -MG

CC_quiet	= @echo "  CC       " $@ && $(CC_normal)
CP_quiet	= @echo "  CP       " `echo "$<" | sed sX/.*/X/.../X` && \
		    $(CP_normal)
BUILD_quiet	= @echo "  BUILD    " $@ && $(BUILD_normal)
DEPEND_quiet	= @$(DEPEND_normal)

ifeq ($(V),1)
    CC		= $(CC_normal)
    CP		= $(CP_normal)
    BUILD	= $(BUILD_normal)
    DEPEND	= $(DEPEND_normal)
else
    CC		= $(CC_quiet)
    CP		= $(CP_quiet)
    BUILD	= $(BUILD_quiet)
    DEPEND	= $(DEPEND_quiet)
endif

#----- Compilation ------------------------------------------------------------

all:		$(NAME).bin

$(NAME).bin:	$(NAME).elf
		$(BUILD) $(OBJCOPY) -j .text -j .data -O binary $< $@ || \
		    { rm -f $@; exit 1; }

$(NAME).elf:	$(OBJS) $(STARTUP_S) $(LD_SCRIPT) Makefile
		$(CC) $(CFLAGS) -o $@ $(OBJS) $(STARTUP_S) $(LDFLAGS)

#----- Download the ST demo builder and cherry-pick the files we need ---------

#
# STSW-STM32106STM32F2 and STM32F4 demonstration builder platform
# http://www.st.com/web/catalog/tools/FM147/CL1794/SC961/SS1743/PF258142
#

#
# Note that ST's ZIP contains files we can't legally redistribute, even though
# few enough people seem to care about such details. Also note that ST only
# provide a file of the latest version, thus not allowing us to pin a known to
# be good version.
#

DEMO_DIR=STM32F2-F4_Demonstration_Builder_V1.3.0
DEMO_ZIP=stm32f2-f4_demobuild.zip
TEMPL=$(DEMO_DIR)/Libraries/CMSIS/Device/ST/STM32F4xx/Source/Templates/
STM_DEMO=http://www.st.com/st-web-ui/static/active/en/st_prod_software_internet/resource/technical/software/firmware/$(DEMO_ZIP)

$(DEMO_ZIP):
		wget $(STM_DEMO) || { rm -f $@; exit 1; }

$(BUILD_DIR):	$(DEMO_ZIP)
		unzip $< || { rm -rf $@; exit 1; }

$(STARTUP_S):	$(TEMPL)/gcc_ride7/$(STARTUP_S)
		$(CP) $< $@ || { rm -f $@; exit 1; }

$(SYSTEM_C):	$(TEMPL)/$(SYSTEM_C)
		$(CP) $< $@ || { rm -f $@; exit 1; }

$(LD_SCRIPT):	$(DEMO_DIR)/Project/TrueSTUDIO/STM324xG-EVAL/STM32F407IG_FLASH.ld
		$(CP) $< $@ || { rm -f $@; exit 1; }

#----- Flash firmware with DFU ------------------------------------------------

dfu:		$(NAME).bin
		dfu-util -d 0483:df11 -a 0 -s 0x8000000 -D $<

#----- Dependencies ----------------------------------------------------------

MKDEP =	$(DEPEND) $< |							\
	  sed 								\
	    -e 's|^$(basename $(notdir $<)).o:|$@:|'			\
	    -e '/^\(.*:\)\? */{p;s///;s/ *\\\?$$/ /;s/  */:\n/g;H;}'	\
	    -e '$${g;p;}'						\
	    -e d >$(basename $@).d;					\
	  [ "$${PIPESTATUS[*]}" = "0 0" ] ||				\
	  { rm -f $(basename $@).d; exit 1; }

%.o:            %.c
		$(CC) $(CFLAGS) -Os -c $<
		$(MKDEP)

-include $(OBJS:.o=.d)

#----- Cleanup ----------------------------------------------------------------

clean:
		rm -f $(OBJS)
		rm -f $(STARTUP_S) $(SYSTEM_C)
		rm -f $(LD_SCRIPT)
		rm -f $(NAME).elf $(NAME).bin

spotless:	clean
		rm -f STM32F2-F4_Demonstration_Builder_V1.3.0
		rm -f $(DEMO_ZIP)
