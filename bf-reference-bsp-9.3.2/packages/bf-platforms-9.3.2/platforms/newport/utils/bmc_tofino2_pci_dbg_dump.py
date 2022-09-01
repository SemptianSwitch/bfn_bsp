#!/usr/bin/python
import shlex
import subprocess

# This file to execute on BMC shell !! (scp to BMC system and run)

shell_cmd_read = "i2c_set_get 11 0x58 5 4 0xa0"

def run_cmd(addr):
    """
    addr: as int
    """

    addr_h8 = '{:08x}'.format(addr)
    addr_param = "0x{} 0x{} 0x{} 0x{}".format(addr_h8[6:], addr_h8[4:6], addr_h8[2:4], addr_h8[0:2])
    cmd_str = "{} {}".format(shell_cmd_read, addr_param)
    cmd = shlex.split(cmd_str)
    p=subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=False)
    o,e = p.communicate()
    if o:
        rdata = o.strip().split()
        rdata_r = "0x{:02x}{:02x}{:02x}{:02x}".format( int(rdata[3],16),int(rdata[2],16),
                                                        int(rdata[1],16),int(rdata[0],16))

    print( "0x{:08x} : {}".format(addr, rdata_r))



def main():
#dump 128 32 bit registers in PCIE bar4 space addresses 0 thru 0x200
#dump 160 32 bit registers in MIS space 0 thru 0x100
#dump 1024 32 bit registers in misc_tv80 space 0x00084000 thru 0x00085000
    pci_reg_addr = 0x00000000
    pci_reg_cnt = 512
    pci_bar4_addr = 0x20000000
    pci_bar4_cnt = 128
    misc_addr = 0x00080000
    misc_cnt = 160
    misc_tv80_addr = 0x00084000
    misc_tv80_cnt = 1024
    pci_bar4_ext_addr = 0x20002000
    pci_bar4_ext_cnt = 256
    print "pci bar4 space"
    addr=pci_bar4_addr
    for x in range(pci_bar4_cnt):
      run_cmd(addr)
      addr = addr + 4

    print "pci bar4 extended space"
    addr=pci_bar4_ext_addr
    for x in range(pci_bar4_ext_cnt):
      run_cmd(addr)
      addr = addr + 4

    print "pci reg space"
    addr=pci_reg_addr
    for x in range(pci_reg_cnt):
      run_cmd(addr)
      addr = addr + 4

    print "misc space"
    addr=misc_addr
    for x in range(misc_cnt):
      run_cmd(addr)
      addr = addr + 4

    print "misc tv80 space"
    addr=misc_tv80_addr
    for x in range(misc_tv80_cnt):
      run_cmd(addr)
      addr = addr + 4


if __name__ == "__main__":
    main()


