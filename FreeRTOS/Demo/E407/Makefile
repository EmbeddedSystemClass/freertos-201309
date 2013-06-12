#
# Makefile - 
#
# Developed by Werner Almesberger for Actility S.A., and
# licensed under LGPLv2 by Actility S.A.
#

#
# Note: we treat the M4F largely as if it was an M3, to avoid running into
# problems with unsupported floating-point instructions. @@@ revisit later
#

SHELL = /bin/bash

NAME = demo

ARCH = arm-linux-gnueabi
CC = $(ARCH)-gcc
OBJCOPY = $(ARCH)-objcopy

LD_SCRIPT = stm32_flash.ld

CFLAGS = -Wall -mcpu=cortex-m3 -march=armv7-m -mthumb \
	-I$(DEMO_DIR)/Project/Core/Devices/STM32F2xx \
	-I$(DEMO_DIR)/Libraries/STM32F2xx_StdPeriph_Driver/inc \
	-I$(DEMO_DIR)/Libraries/CMSIS/Device/ST/STM32F2xx/Include \
	-I$(DEMO_DIR)/Libraries/CMSIS/Include

LDFLAGS = -nostartfiles -T$(LD_SCRIPT)

SRC = main.c system_stm32f2xx.c

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

$(NAME).elf:	$(OBJS) startup_stm32f2xx.s $(LD_SCRIPT) Makefile
		$(CC) $(CFLAGS) -o $@ $(OBJS) startup_stm32f2xx.s $(LDFLAGS)

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
TEMPL=$(DEMO_DIR)/Libraries/CMSIS/Device/ST/STM32F2xx/Source/Templates/
STM_DEMO=http://www.st.com/st-web-ui/static/active/en/st_prod_software_internet/resource/technical/software/firmware/$(DEMO_ZIP)

$(DEMO_ZIP):
		wget $(STM_DEMO) || { rm -f $@; exit 1; }

$(BUILD_DIR):	$(DEMO_ZIP)
		unzip $< || { rm -rf $@; exit 1; }

startup_stm32f2xx.s: $(TEMPL)/gcc_ride7/startup_stm32f2xx.s
		$(CP) $< $@ || { rm -f $@; exit 1; }

system_stm32f2xx.c: $(TEMPL)/system_stm32f2xx.c
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
		rm -f startup_stm32f2xx.s system_stm32f2xx.c
		rm -f stm32_flash.ld
		rm -f $(NAME).elf $(NAME).bin

spotless:	clean
		rm -f STM32F2-F4_Demonstration_Builder_V1.3.0
		rm -f $(DEMO_ZIP)
