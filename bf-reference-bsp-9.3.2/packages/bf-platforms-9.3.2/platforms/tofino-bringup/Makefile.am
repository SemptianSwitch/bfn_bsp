libtofbringup_la_CFLAGS = $(AM_CFLAGS) $(BFPLATFORMS_CFLAGS) -DINC_PLTFM_UCLI
if IS_THRIFT_ENABLED
libtofbringup_la_CFLAGS += -DTHRIFT_ENABLED
endif

libtofbringup_la_SOURCES = \
platform.c \
platform_board.c \
platform_stub.c

noinst_LTLIBRARIES = \
libtofbringup.la

lib_LTLIBRARIES = \
libtofbringup_driver.la

json_targetdir = $(datadir)/platforms/tofino-bringup

install-data-hook:
	mkdir -p $(json_targetdir); \
	cp @srcdir@/board_lane_map.json $(json_targetdir);

uninstall-hook:
	rm -rf $(json_targetdir);

libtofbringup_driver_la_CFLAGS = $(AM_CFLAGS) $(BFPLATFORMS_CFLAGS)

libtofbringup_driver_la_SOURCES =

libtofbringup_driver_la_LIBADD = \
libtofbringup.la
