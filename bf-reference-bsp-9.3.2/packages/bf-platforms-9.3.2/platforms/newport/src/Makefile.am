AM_CPPFLAGS += -I$(srcdir)/../include

libnewport_platform_la_CFLAGS = $(AM_CFLAGS) -I$(top_srcdir)/platforms/newport/include -I$(top_srcdir)/platforms/newport/kdrv -I$(top_srcdir)/platforms/accton-bf/include $(BFPLATFORMS_CFLAGS) -DINC_PLTFM_UCLI
if IS_THRIFT_ENABLED
libnewport_platform_la_CFLAGS += -DTHRIFT_ENABLED
endif

if TCL_ONLY
libnewport_platform_la_CFLAGS += -DTCL_ONLY
endif

libnewport_qsfp_la_CFLAGS = $(AM_CFLAGS) $(BFPLATFORMS_CFLAGS) -I$(top_srcdir)/platforms/common/devices/include -DINC_PLTFM_UCLI
libacctonbf_chss_mgmt_la_CFLAGS = $(AM_CFLAGS) -I$(top_srcdir)/platforms/accton-bf/include $(BFPLATFORMS_CFLAGS) -DINC_PLTFM_UCLI

if IS_ASIC
libacctonbf_chss_mgmt_la_CPPFLAGS = $(AM_CPPFLAGS) -DDEVICE_IS_ASIC
else
libacctonbf_chss_mgmt_la_CPPFLAGS = $(AM_CPPFLAGS) -DDEVICE_IS_SW_MODEL
endif

json_targetdir = $(datadir)/platforms/board-maps/accton

install-data-hook:
	mkdir -p $(json_targetdir); \
	cp @srcdir@/board-maps/board_lane_map*.json $(json_targetdir);

uninstall-hook:
	rm -rf $(json_targetdir);

if TCL_ONLY
libnewport_platform_la_SOURCES = \
platform_mgr/platform.c
else
libnewport_platform_la_SOURCES = \
platform_mgr/platform.c \
platform_mgr/platform_board.c \
platform_mgr/platform_health_mntr.c \
platform_mgr/platform_stub.c \
bf_pltfm_slave_i2c/bf_pltfm_slave_i2c.c \
bf_pltfm_led/bf_pltfm_newport_led.c \
bf_pltfm_led/bf_pltfm_newport_cpld_led.c \
../../accton-bf/src/bf_pltfm_bmc_tty/bmc_tty.c \
../../accton-bf/src/bf_pltfm_spi/bf_pltfm_spi.c \
fpga_i2c/fpga_i2c_lib.c \
fpga_i2c/fpga_i2c_ucli.c \
bf_pltfm_lmk5318/bf_pltfm_lmk5318.c
endif

if TCL_ONLY
noinst_LTLIBRARIES = \
libnewport_platform.la
else
libnewport_qsfp_la_SOURCES = \
../../common/devices/pca953x/bf_pcal9535.c \
qsfp/bf_newport_qsfp.c

libacctonbf_chss_mgmt_la_SOURCES = \
../../accton-bf/src/bf_pltfm_chss_mgmt/bf_pltfm_chss_mgmt_intf.c \
../../accton-bf/src/bf_pltfm_chss_mgmt/bf_pltfm_bd_eeprom.c \
../../accton-bf/src/bf_pltfm_chss_mgmt/bf_pltfm_bd_eeprom.h \
../../accton-bf/src/bf_pltfm_chss_mgmt/bf_pltfm_chss_mgmt_tmp.c \
../../accton-bf/src/bf_pltfm_chss_mgmt/bf_pltfm_chss_mgmt_vrail.c \
../../accton-bf/src/bf_pltfm_chss_mgmt/bf_pltfm_chss_mgmt_ps.c \
../../accton-bf/src/bf_pltfm_chss_mgmt/bf_pltfm_chss_mgmt_fan.c \
../../accton-bf/src/bf_pltfm_chss_mgmt/bf_pltfm_chss_mgmt_mac.c \
../../accton-bf/src/bf_pltfm_chss_mgmt/bf_pltfm_chss_mgmt_ucli.c

libacctonbf_chss_mgmt_la_LIBADD = -lcurl

noinst_LTLIBRARIES = \
libnewport_platform.la \
libacctonbf_chss_mgmt.la \
libnewport_qsfp.la
endif

lib_LTLIBRARIES = \
libnewport_driver.la

libnewport_driver_la_CFLAGS = $(AM_CFLAGS) $(BFPLATFORMS_CFLAGS)

libnewport_driver_la_SOURCES =

if TCL_ONLY
libnewport_driver_la_LIBADD = \
libnewport_platform.la
else
libnewport_driver_la_LIBADD = \
libnewport_platform.la \
libacctonbf_chss_mgmt.la \
libnewport_qsfp.la
endif

libnewport_driver_la_LIBADD += $(builddir)/../../accton-bf/tcl_server/libtcl_server.la
if !TCL_ONLY
if IS_THRIFT_ENABLED
libnewport_driver_la_LIBADD += $(builddir)/../../accton-bf/thrift/libplatform_thrift.la
endif
endif

bin_SCRIPTS = \
../utils/bmc_tofino2_pci_dbg_dump.py \
../utils/tofino2_pci_dbg_dump.py \
../utils/tofino_i2c_rd_np.sh \
../utils/tofino_i2c_wr_np.sh \
../utils/tofino_i2c_rd_local_np.sh \
../utils/tofino_i2c_wr_local_np.sh
