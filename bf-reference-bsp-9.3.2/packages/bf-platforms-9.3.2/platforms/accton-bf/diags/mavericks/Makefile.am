lib_LTLIBRARIES = libaccton_mav_diags.la

AM_CPPFLAGS += -I$(includedir) -I$(srcdir)/../../include

libaccton_mav_diags_la_CFLAGS = $(AM_CFLAGS) $(BFPLATFORMS_CFLAGS)
libaccton_mav_diags_la_CPPFLAGS = -I$(top_builddir)/p4-build $(AM_CPPFLAGS)
if IS_ASIC
libaccton_mav_diags_la_CPPFLAGS += -DDEVICE_IS_ASIC
else
libaccton_mav_diags_la_CPPFLAGS += -DDEVICE_IS_SW_MODEL
endif

libaccton_mav_diags_la_SOURCES = \
diag_server.c \
diag_handler.c \
diag_ucli.c

libaccton_mav_diags_la_LIBADD =

bin_SCRIPTS = \
swutil

SUBDIRS = targets
