scriptsdir = @bindir@
scripts_DATA = \
@srcdir@/bf_fpga_mod_load \
@srcdir@/bf_fpga_mod_unload

install-data-hook:
	chmod +x @bindir@/bf_fpga_mod_load
	chmod +x @bindir@/bf_fpga_mod_unload

all-local:
		cd $(abs_srcdir)/bf_fpga && (test -d "$(abs_builddir)/bf_fpga/i2c" || ((mkdir -p $(abs_builddir)/bf_fpga && mkdir -p $(abs_builddir)/bf_fpga/i2c) && cp $(abs_srcdir)/bf_fpga/Makefile $(abs_builddir)/bf_fpga)) && $(MAKE) BUILD_DIR=$(abs_builddir)/bf_fpga $(AM_MAKEFLAGS) all

check-local:

clean-local:
		-cd $(abs_builddir)/bf_fpga  && $(MAKE) BUILD_DIR=$(abs_builddir)/bf_fpga $(AM_MAKEFLAGS) clean

install-data-local:
		mkdir -p $(libdir)/modules 
		cp $(abs_builddir)/bf_fpga/bf_fpga.ko $(libdir)/modules

install-exec-local:

uninstall-local:
		rm -f $(libdir)/modules/bf_fpga.ko

EXTRA_DIST = bf_fpga/Makefile bf_fpga/bf_fpga_main.c bf_fpga/bf_fpga_ioctl.c bf_fpga/bf_fpga_sysfs.c bf_fpga/bf_fpga_ioctl.h bf_fpga/bf_fpga_priv.h bf_fpga/i2c/bf_fpga_i2c.c bf_fpga/i2c/bf_fpga_i2c.h bf_fpga/i2c/bf_fpga_ctrl.c bf_fpga/i2c/bf_fpga_porting.c bf_fpga/i2c/bf_fpga_porting.h bf_fpga/i2c/bf_fpga_priv.h bf_fpga/i2c/bf_fpga_reg.h

