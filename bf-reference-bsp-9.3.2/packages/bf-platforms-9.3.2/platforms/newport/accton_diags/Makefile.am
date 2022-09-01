lib_LTLIBRARIES = libaccton_newport_diags.la

AM_CPPFLAGS += -I$(includedir) -I$(srcdir)/../../accton-bf/include

libaccton_newport_diags_la_CFLAGS = $(AM_CFLAGS) $(BFPLATFORMS_CFLAGS)
libaccton_newport_diags_la_CPPFLAGS = -I$(top_builddir)/p4-build $(AM_CPPFLAGS)
if IS_ASIC
libaccton_newport_diags_la_CPPFLAGS += -DDEVICE_IS_ASIC
else
libaccton_newport_diags_la_CPPFLAGS += -DDEVICE_IS_SW_MODEL
endif

libaccton_newport_diags_la_SOURCES = \
diag_server.c \
diag_handler.c \
diag_ucli.c

libaccton_newport_diags_la_LIBADD =

bin_SCRIPTS = \
swutil

SUBDIRS = targets
