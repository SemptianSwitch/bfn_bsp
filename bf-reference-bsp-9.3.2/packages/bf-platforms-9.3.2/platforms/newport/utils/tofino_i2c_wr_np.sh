#!/bin/sh
#
#
usage() {
    echo "Usage: $0 <register offset bits7-0> <bits 15-8> <bits 23-16> <bits<bar 31-28 | bits 27-24> <data offset 7-0> <data 15-8> <data 23-16> <data 31-24>" >&2
}

if [ "$#" -lt 8 ]; then
    usage
    exit 1
fi

cmd=0x80
i2c_addr=0x58


addr_1=$1
addr_2=$2
addr_3=$3
addr_4=$4
data_1=$5
data_2=$6
data_3=$7
data_4=$8

#set the mux
./fpga_util i2c_write 0 32 0xff 0 0x74 1 0x40

./fpga_util i2c_write 0 32 0xff 0  $i2c_addr 9 $cmd $addr_1 $addr_2 $addr_3 $addr_4 $data_1 $data_2 $data_3 $data_4

#reset the mux
./fpga_util i2c_write 0 32 0xff 0 0x74 1 0x00

