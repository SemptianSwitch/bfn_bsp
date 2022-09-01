Tofino bringup platforms Software
====================================
The directory implements a sample of a small scope platform based on Tofino
chip. Its purposes are to use it for a quick bringup of such a platform and to
fully implement any vendor platforms starting from this basis. Refer to the
instructions at the end of the file <bf-platforms/README>, for implementing a
new platforms code. This platform, called "tofino-bringup" platform, is put
together following those instructions.


This package is organized as follows

  platforms/tofino-bringup/

    board_lane_map.json
      Describes the front panel port/lane settings. This platform uses a json
      file to describe the port/lane mapping, unlike the accton-bf
      implementation, that uses C header files. Build system packages this file
      in the runtime install folder. The contents of this file is for a
      particular bringup platform with 33 QSFPs. It can be changed and no
      recompilation is needed.

    platform_board.c
      Parses the above json file to support the generic bf_bd_cfg APIs.

    platform.c
      It has the minimalistic platforms manager initialization and exit-cleanup
      code.

    platforms_stub.c
      It has the stubs for all the mandatory implementations (per platforms/
      include/*.h). These stubs are adequate to bring up the ports using
      copper loopback modules or a very short length copper cables.


Building and installing
=======================

    Makefile.am
      Compiles all the above files and produces lib_tofbringup_driver.la that
      gets linked to libpltfm_mgr.la

    Again, refer to the instructions to modify the makefiles in the file,
    <bf-platforms/README>

    Here're the steps to build and install the <bf-platforms> package for
    tofino-bringup platform

    cd <bf-platforms>
    ./configure --with-tof-brgup-plat --prefix=$BSP_INSTALL
                [--enable-thrift]
                [-host=i386-linux-gnu CFLAGS=-m32 CXXFLAGS=-m32 LDFLAGS=-m32]
    make
    make install
