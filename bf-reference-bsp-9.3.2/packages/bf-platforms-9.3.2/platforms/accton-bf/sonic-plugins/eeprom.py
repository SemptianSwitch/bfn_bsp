#!/usr/bin/python

import importlib
import time
import os
import re
import pltfm_pm_rpc
import pltfm_mgr_rpc
from pltfm_pm_rpc.ttypes import *
from pltfm_mgr_rpc.ttypes import *

from thrift.transport import TSocket
from thrift.transport import TTransport
from thrift.protocol import TBinaryProtocol
from thrift.protocol import TMultiplexedProtocol

from argparse import ArgumentParser

thrift_server = 'localhost'
transport = None
pltfm_mgr = None

def thriftSetup():
    global thrift_server, transport, pltfm_mgr
    transport = TSocket.TSocket(thrift_server, 9090)

    transport = TTransport.TBufferedTransport(transport)
    bprotocol = TBinaryProtocol.TBinaryProtocol(transport)

    pltfm_mgr_client_module = importlib.import_module(".".join(["pltfm_mgr_rpc", "pltfm_mgr_rpc"]))
    pltfm_mgr_protocol = TMultiplexedProtocol.TMultiplexedProtocol(bprotocol, "pltfm_mgr_rpc")
    pltfm_mgr = pltfm_mgr_client_module.Client(pltfm_mgr_protocol)

    transport.open()
    #transport_diag.open()

def pltfm_mgr_eeprom_show():
    global pltfm_mgr
    '''
    '''
    eeprom = pltfm_mgr.pltfm_mgr_sys_eeprom_get()
    print "version: %s" % eeprom.version
    print "name: %s" % eeprom.prod_name
    print "part #: %s" % eeprom.prod_part_num
    print "System assembly #: %s" % eeprom.sys_asm_part_num
    print "bfn pcba part #: %s" % eeprom.bfn_pcba_part_num
    print "bfn pcbb part #: %s" % eeprom.bfn_pcbb_part_num
    print "odb pcba part #: %s" % eeprom.odm_pcba_part_num
    print "odm pcba serial #: %s" % eeprom.odm_pcba_ser_num
    print "product serial #: %s" % eeprom.prod_ser_num
    print "product asset tag: %s" % eeprom.prod_ast_tag
    print "system mfger: %s" % eeprom.sys_mfger
    print "system mfg date: %s" % eeprom.sys_mfg_date
    print "pcb mfger: %s" % eeprom.pcb_mfger
    print "assembled at: %s" % eeprom.assembled_at
    print "loc mac address number: %s" % eeprom.loc_mac_addr
    print "ext mac address: %s" % eeprom.ext_mac_addr
    print "location: %s" % eeprom.location

def thriftTeardown():
    global transport
    transport.close()


thriftSetup()
pltfm_mgr_eeprom_show()
thriftTeardown()
