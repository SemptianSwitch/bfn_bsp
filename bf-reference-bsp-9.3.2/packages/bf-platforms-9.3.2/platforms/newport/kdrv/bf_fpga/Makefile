###########################################################################
#
# Local source generation targets.
#
###############################################################################

krn = bf_fpga

ifneq ($(KERNELRELEASE),)

obj-m := $(krn).o
$(krn)-y := bf_fpga_main.o bf_fpga_ioctl.o bf_fpga_sysfs.o i2c/bf_fpga_i2c.o i2c/bf_fpga_i2c_ctrl.o i2c/bf_fpga_i2c_porting.o

ccflags-y := -I$(src)

else

KDIR ?= /lib/modules/$$(uname -r)/build
ARCH ?= x86

all: default

default:
	            $(MAKE) ARCH=$(ARCH) -C $(KDIR) M=$$BUILD_DIR src=$$PWD modules

clean:
	            $(MAKE) ARCH=$(ARCH) -C $(KDIR) M=$$BUILD_DIR src=$$PWD clean

.PHONY : clean
endif
