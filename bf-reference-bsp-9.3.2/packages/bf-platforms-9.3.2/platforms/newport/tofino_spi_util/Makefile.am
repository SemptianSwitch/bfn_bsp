bin_PROGRAMS = spi_i2c_util
spi_i2c_util_CFLAGS = $(AM_CFLAGS) $(BFPLATFORMS_CFLAGS)
spi_i2c_util_CPPFLAGS = -I$(includedir) -I$(top_srcdir)/drivers/include/ -I$(top_srcdir)/platforms/newport/kdrv -I$(top_srcdir)/platforms/include -I$(top_srcdir)/platforms/newport/include -DDEVICE_IS_ASIC

spi_i2c_util_SOURCES = \
tofino_spi_if.c \
tofino_porting_spi.c \
../src/fpga_i2c/fpga_i2c_lib.c

spi_i2c_util_LDFLAGS = $(AM_LDFLAGS) -L$(top_builddir)/platforms/newport/src/

spi_i2c_util_LDADD = -lbfsys -lcurl
