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

i2c_addr=0x58

addr_1=$1
addr_2=$2
addr_3=$3
addr_4=$4

#set the mux
./fpga_util i2c_write 0 32 0xff 0 0x74 1 0x40

#fill up S_ADDR
./fpga_util i2c_write 0 32 0xff 0 $i2c_addr 2 0 $addr_1
./fpga_util i2c_write  0 32 0xff 0 $i2c_addr 2 1 $addr_2
./fpga_util i2c_write 0 32 0xff 0 $i2c_addr 2 2 $addr_3
./fpga_util i2c_write  0 32 0xff 0 $i2c_addr 2 3 $addr_4

#read and execute
cmd=0x08
data_1=0x21
./fpga_util 0 32 0xff 0 $i2c_addr 2 $cmd $data_1

sleep 1

echo "reading status of write"
cmd=0x28
./fpga_util i2c_addr_read 0 32 0xff 0 $i2c_addr 1 1 $cmd

#read the data back
cmd=0x24
./fpga_util i2c_addr_read 0 32 0xff 0 $i2c_addr 1 1 $cmd

cmd=0x25
./fpga_util i2c_addr_read 0 32 0xff 0 $i2c_addr 1 1 $cmd

cmd=0x26
./fpga_util i2c_addr_read 0 32 0xff 0 $i2c_addr 1 1 $cmd

cmd=0x27
./fpga_util i2c_addr_read 0 32 0xff 0 $i2c_addr 1 1 $cmd

#reset the mux
./fpga_util i2c_write 0 32 0xff 0 0x74 1 0x00

