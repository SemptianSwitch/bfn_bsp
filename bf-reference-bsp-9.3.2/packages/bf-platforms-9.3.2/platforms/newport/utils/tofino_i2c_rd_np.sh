#!/bin/sh
#
#
usage() {
  echo "Usage: $0 <register offset bits7-0> <bits 15-8> <bits 23-16> <bits<bar 31-28 | bits 27-24>" >&2
}

if [ "$#" -lt 4 ]; then
    usage
    exit 1
fi

cmd=0xa0
i2c_addr=0x58

addr_1=$1
addr_2=$2
addr_3=$3
addr_4=$4

#set the mux
./fpga_util i2c_write 0 32 0xff 0 0x74 1 0x40

./fpga_util i2c_write 0 32 0xff 0  $i2c_addr 5 $cmd $addr_1 $addr_2 $addr_3 $addr_4

./fpga_util i2c_read 0 32 0xff 0 $i2c_addr 4

#reset the mux
./fpga_util i2c_write 0 32 0xff 0 0x74 1 0x00

