Barefoot Networks Newport Platforms Software
======================================================
The <bf-platforms/platforms/newport/>
  Package contains source code for the drivers and agents for Barefoot Networks
  Platforms, Newport. It is organized as follow.


  src/

    Driver source files made up of following directories.

    bf_pltfm_chss_mgmt:
      Chassis Mgmt API (e.g. fans, temp sensors, power-mgmt, eeprom)
      *** currently compiles the accton-bf Chassis Mgmt files
    fpga_i2c:
      user space library to perform i2c thru FPGA
    bf_pltfm_led:
      System and port LED Mgmt API
    bf_pltfm_qsfp:
      QSFP access, RESET, INTERRUPT, PRESENCE and LPMODE
    bf_pltfm_lmk5318:
      APIs for configuring clock sync device lmk5318
    bf_pltfm_mgr:
      Platform initialization
    bf_pltfm_slave_i2c:
      API to access Barefoot ASIC over i2c bus
    bf_pltfm_spi:
      API to access PCIe serdes EEPROM over SPI interface of Barefoot ASIC
      *** currently compiles the accton-bf files

  kdrv/
    fpga kernel mode driver

  include/
    Header files with forward declarations of APIs exported by modules
    under src/

  thrift/
    Thrift IDL to expose APIs over thrift
    *** compiles files from accton-bf

  tcl_server/
    tcl server implementation
    *** compiles files from accton-bf

  diags/
    manufacturing data plane diagnostic test suite 
    *** compiles files from accton-bf

  ptf-tests/
    Example scripts that demonstrate invoking of API over thrift in python
    *** compiles files from accton-bf

Dependencies
============
The <bf-platforms/platforms/newport> package expects a few dependencies like 
libcurl to be pre-installed before it can be successfully built. All 
the necessary packages can be installed in the following manner
can be installed in the following manner

    cd $SDE/bf-platforms/platforms/newport
    ./install_pltfm_deps.sh

Artifacts installed
===================
Here're the artifacts that get installed for <bf-platforms>

configure the build system using the option, --with-newport-plat
    to override the default platform (which is accton-bf)

Build artifacts:

    header files for driver API to $BSP_INSTALL/include/bf_pltfm/newport/
                                   and $BSP_INSTALL/include/bf_pltfm/accton-bf/

    libnewport_driver.[a,la,so] to $BSP_INSTALL/lib/
      driver library to manage the newport platform components

Thrift mode additional build artifacts:

    libplatform_thrift.[a,la,so] to $BSP_INSTALL/lib/
      library that provides thrift API for platform API

