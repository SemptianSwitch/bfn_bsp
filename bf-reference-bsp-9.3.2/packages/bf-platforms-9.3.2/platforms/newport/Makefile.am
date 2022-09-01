ACLOCAL_AMFLAGS = ${ACLOCAL_FLAGS} -I m4

SUBDIRS =
SUBDIRS += ../accton-bf/tcl_server
if !TCL_ONLY
if IS_THRIFT_ENABLED
SUBDIRS += ../accton-bf/thrift
endif
SUBDIRS += include kdrv src fpga_util tofino_spi_util
if BUILD_ACCTONBF_DIAGS
SUBDIRS += accton_diags
endif
else
SUBDIRS += src
endif
