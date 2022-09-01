AM_CPPFLAGS += -I$(srcdir)/../include

libacctonbf_platform_la_CFLAGS = $(AM_CFLAGS) $(BFPLATFORMS_CFLAGS) -DINC_PLTFM_UCLI
if IS_THRIFT_ENABLED
libacctonbf_platform_la_CFLAGS += -DTHRIFT_ENABLED
endif

if TCL_ONLY
libacctonbf_platform_la_CFLAGS += -DTCL_ONLY
endif

libacctonbf_qsfp_la_CFLAGS = $(AM_CFLAGS) $(BFPLATFORMS_CFLAGS) -DINC_PLTFM_UCLI
libacctonbf_cp2112_la_CFLAGS = $(AM_CFLAGS) $(BFPLATFORMS_CFLAGS) -DINC_PLTFM_UCLI
libacctonbf_chss_mgmt_la_CFLAGS = $(AM_CFLAGS) $(BFPLATFORMS_CFLAGS) -DINC_PLTFM_UCLI

if IS_ASIC
libacctonbf_chss_mgmt_la_CPPFLAGS = $(AM_CPPFLAGS) -DDEVICE_IS_ASIC
else
libacctonbf_chss_mgmt_la_CPPFLAGS = $(AM_CPPFLAGS) -DDEVICE_IS_SW_MODEL
endif

if TCL_ONLY
libacctonbf_platform_la_SOURCES = \
platform_mgr/platform.c
else
libacctonbf_platform_la_SOURCES = \
bf_pltfm_bmc_tty/bmc_tty.c \
platform_mgr/platform.c \
platform_mgr/platform_board.c \
platform_mgr/platform_health_mntr.c \
bf_pltfm_led/bf_pltfm_mav_led.c \
bf_pltfm_led/bf_pltfm_mav_cpld_led.c \
bf_pltfm_rptr/rptr.h \
bf_pltfm_rptr/bf_pltfm_rptr.c \
bf_pltfm_rptr/bf_pltfm_rptr_ucli.c \
bf_pltfm_rtmr/bf_pltfm_rtmr_priv.h \
bf_pltfm_rtmr/bf_pltfm_rtmr.c \
bf_pltfm_rtmr/bf_pltfm_rtmr_ucli.c \
bf_pltfm_slave_i2c/bf_pltfm_slave_i2c.c \
bf_pltfm_spi/bf_pltfm_spi.c \
bf_pltfm_si5342/bf_pltfm_si5342.c
endif

if TCL_ONLY
noinst_LTLIBRARIES = \
libacctonbf_platform.la
else
libacctonbf_qsfp_la_SOURCES = \
qsfp/bf_mav_qsfp_module.c \
qsfp/bf_mav_qsfp_sub_module.c

libacctonbf_chss_mgmt_la_SOURCES = \
bf_pltfm_chss_mgmt/bf_pltfm_chss_mgmt_intf.c \
bf_pltfm_chss_mgmt/bf_pltfm_bd_eeprom.c \
bf_pltfm_chss_mgmt/bf_pltfm_bd_eeprom.h \
bf_pltfm_chss_mgmt/bf_pltfm_chss_mgmt_tmp.c \
bf_pltfm_chss_mgmt/bf_pltfm_chss_mgmt_vrail.c \
bf_pltfm_chss_mgmt/bf_pltfm_chss_mgmt_ps.c \
bf_pltfm_chss_mgmt/bf_pltfm_chss_mgmt_fan.c \
bf_pltfm_chss_mgmt/bf_pltfm_chss_mgmt_mac.c \
bf_pltfm_chss_mgmt/bf_pltfm_chss_mgmt_ucli.c

libacctonbf_chss_mgmt_la_LIBADD = -lcurl

libacctonbf_cp2112_la_SOURCES = \
bf_pltfm_cp2112/bf_pltfm_cp2112_intf.c \
bf_pltfm_cp2112/bf_pltfm_cp2112_intf_ucli.c

libacctonbf_cp2112_la_LIBADD = -lusb-1.0 -lm


noinst_LTLIBRARIES = \
libacctonbf_platform.la \
libacctonbf_cp2112.la \
libacctonbf_chss_mgmt.la \
libacctonbf_qsfp.la
endif

lib_LTLIBRARIES = \
libacctonbf_driver.la

libacctonbf_driver_la_CFLAGS = $(AM_CFLAGS) $(BFPLATFORMS_CFLAGS)

libacctonbf_driver_la_SOURCES =

if TCL_ONLY
libacctonbf_driver_la_LIBADD = \
libacctonbf_platform.la
else
libacctonbf_driver_la_LIBADD = \
libacctonbf_platform.la \
libacctonbf_cp2112.la \
libacctonbf_chss_mgmt.la \
libacctonbf_qsfp.la
endif

libacctonbf_driver_la_LIBADD += $(builddir)/../tcl_server/libtcl_server.la
if !TCL_ONLY
if IS_THRIFT_ENABLED
libacctonbf_driver_la_LIBADD += $(builddir)/../thrift/libplatform_thrift.la
endif
endif

bin_SCRIPTS = \
../utils/tofino_i2c_rd.sh \
../utils/tofino_i2c_wr.sh \
../utils/tofino_i2c_rd_local.sh \
../utils/tofino_i2c_wr_local.sh \
../utils/tofino_pci_bringup.sh \
../utils/credo_firmware.bin
