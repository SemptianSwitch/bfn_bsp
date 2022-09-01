bin_PROGRAMS = fpga_util bf_fpga_update
fpga_util_CFLAGS = $(AM_CFLAGS) $(BFPLATFORMS_CFLAGS)
fpga_util_CPPFLAGS = -I$(includedir) -I$(top_srcdir)/drivers/include/ -I$(top_srcdir)/platforms/newport/kdrv -I$(top_srcdir)/platforms/include -I$(top_srcdir)/platforms/newport/include/ -DDEVICE_IS_ASIC

fpga_util_SOURCES = \
fpga_util.c \
../src/fpga_i2c/fpga_i2c_lib.c

fpga_util_LDFLAGS = $(AM_LDFLAGS) -L$(top_builddir)/platforms/newport/src/

fpga_util_LDADD = -lbfsys

bf_fpga_update_CFLAGS = $(AM_CFLAGS)

bf_fpga_update_SOURCES = fpga_update.c

bf_fpga_update_LDFLAGS = $(AM_LDFLAGS)
