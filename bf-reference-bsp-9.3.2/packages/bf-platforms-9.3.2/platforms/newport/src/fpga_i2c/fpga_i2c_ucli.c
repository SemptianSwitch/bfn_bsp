/*******************************************************************************
 * Copyright (c) 2015-2020 Barefoot Networks, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * $Id: $
 *
 ******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <signal.h>
#include <dvm/bf_dma_types.h>
#include <dvm/bf_drv_intf.h>
#include <bfutils/uCli/ucli.h>
#include <bfutils/uCli/ucli_argparse.h>
#include <bfutils/uCli/ucli_handler_macros.h>
#include <bf_pltfm_types/bf_pltfm_types.h>

#include "bf_pltfm_fpga.h"

static ucli_status_t bf_pltfm_fpga_ucli_ucli__reg_write32__(
    ucli_context_t *uc) {
  int fpga_id;
  uint32_t offset;
  uint32_t val;

  UCLI_COMMAND_INFO(uc, "reg-write32", 3, "<fpga-id> <reg_addr> <32-bit val>");

  fpga_id = atoi(uc->pargs->args[0]);
  offset = strtoul(uc->pargs->args[1], NULL, 0);
  val = strtoul(uc->pargs->args[2], NULL, 0);
  bf_fpga_reg_write32(fpga_id, offset, val);
  aim_printf(
      &uc->pvs, "wrote 0x%x to offset 0x%x on fpga %d\n", val, offset, fpga_id);
  return UCLI_STATUS_OK;
}

static ucli_status_t bf_pltfm_fpga_ucli_ucli__reg_write8__(ucli_context_t *uc) {
  int fpga_id;
  uint32_t offset;
  uint8_t val;

  UCLI_COMMAND_INFO(uc, "reg-write8", 3, "<fpga-id> <reg_addr> <8-bit val>");

  fpga_id = atoi(uc->pargs->args[0]);
  offset = strtoul(uc->pargs->args[1], NULL, 0);
  val = (uint8_t)strtoul(uc->pargs->args[2], NULL, 0);
  bf_fpga_reg_write8(fpga_id, offset, val);
  aim_printf(
      &uc->pvs, "wrote 0x%x to offset 0x%x on fpga %d\n", val, offset, fpga_id);
  return UCLI_STATUS_OK;
}

static ucli_status_t bf_pltfm_fpga_ucli_ucli__reg_read32__(ucli_context_t *uc) {
  int fpga_id;
  uint32_t offset;
  uint32_t val;

  UCLI_COMMAND_INFO(uc, "reg-read32", 2, "<fpga-id> <reg_addr>");

  fpga_id = atoi(uc->pargs->args[0]);
  offset = strtoul(uc->pargs->args[1], NULL, 0);
  bf_fpga_reg_read32(fpga_id, offset, &val);
  aim_printf(&uc->pvs,
             "read 0x%x from offset 0x%x on fpga %d\n",
             val,
             offset,
             fpga_id);
  return UCLI_STATUS_OK;
}

static ucli_status_t bf_pltfm_fpga_ucli_ucli__reg_read8__(ucli_context_t *uc) {
  int fpga_id;
  uint32_t offset;
  uint8_t val;

  UCLI_COMMAND_INFO(uc, "reg-read8", 2, "<fpga-id> <reg_addr>");

  fpga_id = atoi(uc->pargs->args[0]);
  offset = strtoul(uc->pargs->args[1], NULL, 0);
  bf_fpga_reg_read8(fpga_id, offset, &val);
  aim_printf(&uc->pvs,
             "read 0x%x from offset 0x%x on fpga %d\n",
             val,
             offset,
             fpga_id);
  return UCLI_STATUS_OK;
}

static ucli_status_t bf_pltfm_fpga_ucli_ucli__i2c_start__(ucli_context_t *uc) {
  int fpga_id, bus, start;
  bool st;

  UCLI_COMMAND_INFO(uc, "i2c-start", 3, "<fpga-id> <bus> <1:start, 0:stop>");

  fpga_id = atoi(uc->pargs->args[0]);
  bus = atoi(uc->pargs->args[1]);
  start = atoi(uc->pargs->args[2]);
  if (start) {
    st = true;
  } else {
    st = false;
  }
  if (bf_fpga_i2c_start(fpga_id, bus, st)) {
    aim_printf(&uc->pvs, "error changing the run state\n");
  } else {
    aim_printf(&uc->pvs,
               "fpga %d bus %d %s\n",
               fpga_id,
               bus,
               (st ? "started" : "stopped"));
  }
  return UCLI_STATUS_OK;
}

static ucli_status_t bf_pltfm_fpga_ucli_ucli__i2c_en__(ucli_context_t *uc) {
  int fpga_id, bus, inst_id, enable;
  bool en;

  UCLI_COMMAND_INFO(uc, "i2c-en", 4, "<fpga-id> <bus> <inst_id> <1:en, 0:dis>");

  fpga_id = atoi(uc->pargs->args[0]);
  bus = atoi(uc->pargs->args[1]);
  inst_id = atoi(uc->pargs->args[2]);
  enable = atoi(uc->pargs->args[3]);
  if (enable) {
    en = true;
  } else {
    en = false;
  }
  if (bf_fpga_i2c_en(fpga_id, bus, inst_id, en)) {
    aim_printf(&uc->pvs, "error changing the enable state\n");
  } else {
    aim_printf(&uc->pvs,
               "fpga %d bus %d inst %d %s\n",
               fpga_id,
               bus,
               inst_id,
               (en ? "enabled" : "disabled"));
  }
  return UCLI_STATUS_OK;
}

static ucli_status_t bf_pltfm_fpga_ucli_ucli__i2c_is_running__(
    ucli_context_t *uc) {
  int fpga_id, bus;
  bool running;

  UCLI_COMMAND_INFO(uc, "i2c-running", 2, "<fpga-id> <bus>");

  fpga_id = atoi(uc->pargs->args[0]);
  bus = atoi(uc->pargs->args[1]);
  if (bf_fpga_i2c_is_running(fpga_id, bus, &running)) {
    aim_printf(&uc->pvs, "error in getting the run state\n");
  } else {
    aim_printf(&uc->pvs,
               "fpga %d bus %d is %s\n",
               fpga_id,
               bus,
               (running ? "running" : "stopped"));
  }
  return UCLI_STATUS_OK;
}

static ucli_status_t bf_pltfm_fpga_ucli_ucli__i2c_read__(ucli_context_t *uc) {
  int fpga_id, bus, ret;
  unsigned int i;
  uint8_t mux_addr, mux_chn;
  uint8_t i2c_addr;
  uint8_t rd_size;
  uint8_t rd_buf[128];

  UCLI_COMMAND_INFO(uc,
                    "i2c-read",
                    6,
                    "<fpga-id> <bus> <mux-i2c_addr 0xff:no mux> <mux_chn> "
                    "<i2c_addr> <length>");

  fpga_id = atoi(uc->pargs->args[0]);
  bus = atoi(uc->pargs->args[1]);
  mux_addr = (uint8_t)strtol(uc->pargs->args[2], NULL, 0);
  mux_chn = (uint8_t)strtol(uc->pargs->args[3], NULL, 0);
  i2c_addr = (uint8_t)strtol(uc->pargs->args[4], NULL, 0);
  rd_size = (uint8_t)strtol(uc->pargs->args[5], NULL, 0);

  if (rd_size > sizeof(rd_buf)) {
    aim_printf(&uc->pvs, "Too large a read size\n");
    return UCLI_STATUS_E_PARAM;
  }
  aim_printf(
      &uc->pvs,
      "fpga:: i2c-read <dev=%d> <bus=%d> <mux-addr=0x%02x> <mux_cnh=0x%02x> "
      "<i2c_addr=0x%02x> <length=%d>\n",
      fpga_id,
      bus,
      mux_addr,
      mux_chn,
      (unsigned)i2c_addr,
      rd_size);

  if (mux_addr >= 0x80) {
    ret = bf_fpga_i2c_read(fpga_id, bus, 0, i2c_addr, rd_buf, rd_size);
  } else {
    ret = bf_fpga_i2c_read_mux(
        fpga_id, bus, 0, mux_addr, mux_chn, i2c_addr, rd_buf, rd_size);
  }
  if (ret) {
    aim_printf(&uc->pvs, "Error in i2c-read\n");
    return UCLI_STATUS_E_ERROR;
  }
  for (i = 0; i < rd_size; i++) {
    aim_printf(&uc->pvs, "0x%02x ", rd_buf[i]);
  }
  aim_printf(&uc->pvs, "\n");
  return UCLI_STATUS_OK;
}

static ucli_status_t bf_pltfm_fpga_ucli_ucli__i2c_write__(ucli_context_t *uc) {
  int fpga_id, bus, ret;
  unsigned int i;
  uint8_t mux_addr, mux_chn;
  uint8_t i2c_addr;
  uint8_t wr_size;
  uint8_t wr_buf[129];

  UCLI_COMMAND_INFO(uc,
                    "i2c-write",
                    -1,
                    "<fpga-id> <bus> <mux-i2c_addr 0xff:no mux> <mux_chn> "
                    "<i2c_addr> <length> <byte1> [<byte2> ...]");

  if (uc->pargs->count < 7) {
    aim_printf(&uc->pvs, "fpga:: Insufficient arguments\n");
    return UCLI_STATUS_E_PARAM;
  }

  fpga_id = atoi(uc->pargs->args[0]);
  bus = atoi(uc->pargs->args[1]);
  mux_addr = (uint8_t)strtol(uc->pargs->args[2], NULL, 0);
  mux_chn = (uint8_t)strtol(uc->pargs->args[3], NULL, 0);
  i2c_addr = (uint8_t)strtol(uc->pargs->args[4], NULL, 0);
  wr_size = (uint8_t)strtol(uc->pargs->args[5], NULL, 0);

  if (uc->pargs->count < (int)(6 + wr_size)) {
    aim_printf(&uc->pvs, "fpga:: Insufficient arguments\n");
    return UCLI_STATUS_E_PARAM;
  }

  aim_printf(&uc->pvs,
             "fpga:: i2c-write <dev=%d> <bus=%d> <mux-addr=0x%02x> "
             "<mux_cnh=0x%02x> <i2c_addr=0x%02x> <length=%d> "
             "<byte1=0x%02x> ...\n",
             fpga_id,
             bus,
             mux_addr,
             mux_chn,
             (unsigned)i2c_addr,
             wr_size,
             (uint8_t)strtol(uc->pargs->args[6], NULL, 0));

  for (i = 0; i < wr_size; i++) {
    wr_buf[i] = strtol(uc->pargs->args[6 + i], NULL, 0);
  }

  if (mux_addr >= 0x80) {
    ret = bf_fpga_i2c_write(fpga_id, bus, 0, i2c_addr, wr_buf, wr_size);
  } else {
    ret = bf_fpga_i2c_write_mux(
        fpga_id, bus, 0, mux_addr, mux_chn, i2c_addr, wr_buf, wr_size);
  }
  if (ret) {
    aim_printf(&uc->pvs, "Error in i2c-write\n");
    return UCLI_STATUS_E_ERROR;
  }
  aim_printf(&uc->pvs, "fpga i2c-write OK\n");
  return UCLI_STATUS_OK;
}

static ucli_status_t bf_pltfm_fpga_ucli_ucli__i2c_addr_read__(
    ucli_context_t *uc) {
  int fpga_id, bus, ret;
  unsigned int i;
  uint8_t mux_addr, mux_chn;
  uint8_t i2c_addr;
  uint8_t rd_size, wr_size;
  uint8_t buf[128];

  UCLI_COMMAND_INFO(uc,
                    "i2c-addr-read",
                    -1,
                    "<fpga-id> <bus> <mux-i2c_addr 0xff:no mux> <mux_chn> "
                    "<i2c_addr> <read_length> <write_length> <byte1> [<byte2> "
                    "...]");

  if (uc->pargs->count < 8) {
    aim_printf(&uc->pvs, "fpga:: Insufficient arguments\n");
    return UCLI_STATUS_E_PARAM;
  }

  fpga_id = atoi(uc->pargs->args[0]);
  bus = atoi(uc->pargs->args[1]);
  mux_addr = (uint8_t)strtol(uc->pargs->args[2], NULL, 0);
  mux_chn = (uint8_t)strtol(uc->pargs->args[3], NULL, 0);
  i2c_addr = (uint8_t)strtol(uc->pargs->args[4], NULL, 0);
  rd_size = (uint8_t)strtol(uc->pargs->args[5], NULL, 0);
  wr_size = (uint8_t)strtol(uc->pargs->args[6], NULL, 0);

  if (uc->pargs->count < (int)(7 + wr_size)) {
    aim_printf(&uc->pvs, "fpga:: Insufficient arguments\n");
    return UCLI_STATUS_E_PARAM;
  }
  if (rd_size > sizeof(buf) || wr_size > sizeof(buf)) {
    aim_printf(&uc->pvs, "Too large a read  or write size\n");
    return UCLI_STATUS_E_PARAM;
  }

  aim_printf(&uc->pvs,
             "fpga:: addr read <dev=%d> <bus=%d> <mux-addr=0x%02x> "
             "<mux_cnh=0x%02x> <i2c_addr=0x%02x> "
             "<read_length=%d> "
             "<write_length=%d> <byte1=0x%02x> ...\n",
             fpga_id,
             bus,
             mux_addr,
             mux_chn,
             (unsigned)i2c_addr,
             rd_size,
             wr_size,
             (uint8_t)strtol(uc->pargs->args[7], NULL, 0));

  for (i = 0; i < wr_size; i++) {
    buf[i] = strtol(uc->pargs->args[7 + i], NULL, 0);
  }

  if (mux_addr >= 0x80) {
    ret = bf_fpga_i2c_addr_read(
        fpga_id, bus, 0, i2c_addr, buf, buf, wr_size, rd_size);
  } else {
    ret = bf_fpga_i2c_addr_read_mux(fpga_id,
                                    bus,
                                    0,
                                    mux_addr,
                                    mux_chn,
                                    i2c_addr,
                                    buf,
                                    buf,
                                    wr_size,
                                    rd_size);
  }
  if (ret) {
    aim_printf(&uc->pvs, "Error in i2c-addr-read\n");
    return UCLI_STATUS_E_ERROR;
  }
  for (i = 0; i < rd_size; i++) {
    aim_printf(&uc->pvs, "0x%02x ", buf[i]);
  }
  aim_printf(&uc->pvs, "\n");
  return UCLI_STATUS_OK;
}

static ucli_status_t bf_pltfm_fpga_ucli_ucli__i2c_read_add_pr__(
    ucli_context_t *uc) {
  int fpga_id, bus, inst_id, ret;
  uint8_t mux_addr, mux_chn;
  uint8_t i2c_addr;
  uint8_t rd_size;

  UCLI_COMMAND_INFO(uc,
                    "i2c-read-add-pr",
                    6,
                    "<fpga-id> <bus> <mux-i2c_addr 0xff:no mux> <mux_chn> "
                    "<i2c_addr> <length>");

  fpga_id = atoi(uc->pargs->args[0]);
  bus = atoi(uc->pargs->args[1]);
  mux_addr = (uint8_t)strtol(uc->pargs->args[2], NULL, 0);
  mux_chn = (uint8_t)strtol(uc->pargs->args[3], NULL, 0);
  i2c_addr = (uint8_t)strtol(uc->pargs->args[4], NULL, 0);
  rd_size = (uint8_t)strtol(uc->pargs->args[5], NULL, 0);

  if (rd_size > 128) {
    aim_printf(&uc->pvs, "Too large a read size\n");
    return UCLI_STATUS_E_PARAM;
  }
  aim_printf(&uc->pvs,
             "fpga:: i2c-read-add-pr <dev=%d> <bus=%d> <mux-addr=0x%02x> "
             "<mux_cnh=0x%02x> <i2c_addr=0x%02x> <length=%d>\n",
             fpga_id,
             bus,
             mux_addr,
             mux_chn,
             (unsigned)i2c_addr,
             rd_size);

  if (mux_addr >= 0x80) {
    ret =
        bf_fpga_i2c_read_add_pr(fpga_id, bus, 10, i2c_addr, rd_size, &inst_id);
  } else {
    ret = bf_fpga_i2c_read_add_pr_mux(
        fpga_id, bus, 10, mux_addr, mux_chn, i2c_addr, rd_size, &inst_id);
  }
  if (ret) {
    aim_printf(&uc->pvs, "Error in i2c-read-add-pr\n");
    return UCLI_STATUS_E_ERROR;
  }
  aim_printf(&uc->pvs, "i2c-read-add-pr OK inst_id is %d\n", inst_id);
  return UCLI_STATUS_OK;
}

static ucli_status_t bf_pltfm_fpga_ucli_ucli__i2c_addr_read_add_pr__(
    ucli_context_t *uc) {
  int fpga_id, bus, inst_id, ret;
  unsigned int i;
  uint8_t mux_addr, mux_chn;
  uint8_t i2c_addr;
  uint8_t rd_size, wr_size;
  uint8_t buf[128];

  UCLI_COMMAND_INFO(uc,
                    "i2c-addr-read-add-pr",
                    -1,
                    "<fpga-id> <bus> <mux-i2c_addr 0xff:no mux> <mux_chn> "
                    "<i2c_addr> <read_length> <write_length> <byte1> [<byte2> "
                    "...]");

  if (uc->pargs->count < 8) {
    aim_printf(&uc->pvs, "fpga:: Insufficient arguments\n");
    return UCLI_STATUS_E_PARAM;
  }

  fpga_id = atoi(uc->pargs->args[0]);
  bus = atoi(uc->pargs->args[1]);
  mux_addr = (uint8_t)strtol(uc->pargs->args[2], NULL, 0);
  mux_chn = (uint8_t)strtol(uc->pargs->args[3], NULL, 0);
  i2c_addr = (uint8_t)strtol(uc->pargs->args[4], NULL, 0);
  rd_size = (uint8_t)strtol(uc->pargs->args[5], NULL, 0);
  wr_size = (uint8_t)strtol(uc->pargs->args[6], NULL, 0);

  if (uc->pargs->count < (int)(7 + wr_size)) {
    aim_printf(&uc->pvs, "fpga:: Insufficient arguments\n");
    return UCLI_STATUS_E_PARAM;
  }
  if (rd_size > sizeof(buf) || wr_size > sizeof(buf)) {
    aim_printf(&uc->pvs, "Too large a read  or write size\n");
    return UCLI_STATUS_E_PARAM;
  }

  aim_printf(&uc->pvs,
             "fpga:: addr read <dev=%d> <bus=%d> <mux-addr=0x%02x> "
             "<mux_cnh=0x%02x> <i2c_addr=0x%02x> "
             "<read_length=%d> "
             "<write_length=%d> <byte1=%02x> ...\n",
             fpga_id,
             bus,
             mux_addr,
             mux_chn,
             (unsigned)i2c_addr,
             rd_size,
             wr_size,
             (uint8_t)strtol(uc->pargs->args[7], NULL, 0));

  for (i = 0; i < wr_size; i++) {
    buf[i] = strtol(uc->pargs->args[7 + i], NULL, 0);
  }

  if (mux_addr >= 0x80) {
    ret = bf_fpga_i2c_addr_read_add_pr(
        fpga_id, bus, 10, i2c_addr, buf, wr_size, rd_size, &inst_id);
  } else {
    ret = bf_fpga_i2c_addr_read_add_pr_mux(fpga_id,
                                           bus,
                                           10,
                                           mux_addr,
                                           mux_chn,
                                           i2c_addr,
                                           buf,
                                           wr_size,
                                           rd_size,
                                           &inst_id);
  }
  if (ret) {
    aim_printf(&uc->pvs, "Error in i2c-addr-read-add-pr\n");
    return UCLI_STATUS_E_ERROR;
  }
  aim_printf(&uc->pvs, "i2c-addr-read-add-pr OK inst_id is %d\n", inst_id);
  return UCLI_STATUS_OK;
}

static ucli_status_t bf_pltfm_fpga_ucli_ucli__i2c_del_pr__(ucli_context_t *uc) {
  int fpga_id, bus, inst_id, mux_en, ret;

  UCLI_COMMAND_INFO(uc, "i2c-del-pr", 4, "<fpga-id> <bus> <inst_id> <mux_en>");

  fpga_id = atoi(uc->pargs->args[0]);
  bus = atoi(uc->pargs->args[1]);
  inst_id = atoi(uc->pargs->args[2]);
  mux_en = atoi(uc->pargs->args[3]);

  aim_printf(&uc->pvs,
             "fpga:: i2c-del-pr <dev=%d> <bus=%d> <inst_id=%d> <mux_en=%d>\n",
             fpga_id,
             bus,
             inst_id,
             mux_en);

  ret = bf_fpga_i2c_del_pr(fpga_id, bus, inst_id);
  if (mux_en) {
    ret |= bf_fpga_i2c_del_pr(fpga_id, bus, inst_id + 1);
    ret |= bf_fpga_i2c_del_pr(fpga_id, bus, inst_id + 2);
  }
  if (ret) {
    aim_printf(&uc->pvs, "Error in i2c-del-pr\n");
    return UCLI_STATUS_E_ERROR;
  }
  aim_printf(&uc->pvs, "i2c-del-pr OK\n");
  return UCLI_STATUS_OK;
}

static ucli_status_t bf_pltfm_fpga_ucli_ucli__i2c_read_data__(
    ucli_context_t *uc) {
  int fpga_id, bus, inst_id, mux_en, ret, i;
  uint8_t rd_sz, offset, rd_buf[128];

  UCLI_COMMAND_INFO(uc,
                    "i2c-read-data",
                    6,
                    "<fpga-id> <bus> <inst_id> <mux_en> <rd_sz> <offset>");

  fpga_id = atoi(uc->pargs->args[0]);
  bus = atoi(uc->pargs->args[1]);
  inst_id = atoi(uc->pargs->args[2]);
  mux_en = atoi(uc->pargs->args[3]);
  rd_sz = (uint8_t)strtol(uc->pargs->args[4], NULL, 0);
  offset = (uint8_t)strtol(uc->pargs->args[5], NULL, 0);

  aim_printf(&uc->pvs,
             "fpga:: i2c-read-data <dev=%d> <bus=%d> <inst_id=%d> <mux_en=%d> "
             "<rd_sz=%d> <offset=%d>\n",
             fpga_id,
             bus,
             inst_id,
             mux_en,
             rd_sz,
             offset);

  if (mux_en) {
    inst_id += 1;
  }
  ret = bf_fpga_i2c_read_data(fpga_id, bus, inst_id, rd_buf, rd_sz, offset);
  if (ret) {
    aim_printf(&uc->pvs, "Error in i2c-read-data\n");
    return UCLI_STATUS_E_ERROR;
  }
  for (i = 0; i < rd_sz; i++) {
    aim_printf(&uc->pvs, "0x%02x ", rd_buf[i]);
  }
  aim_printf(&uc->pvs, "\ni2c-read-data OK\n");
  return UCLI_STATUS_OK;
}

static ucli_command_handler_f bf_pltfm_fpga_ucli_ucli_handlers__[] = {
    bf_pltfm_fpga_ucli_ucli__reg_read32__,
    bf_pltfm_fpga_ucli_ucli__reg_write32__,
    bf_pltfm_fpga_ucli_ucli__reg_read8__,
    bf_pltfm_fpga_ucli_ucli__reg_write8__,
    bf_pltfm_fpga_ucli_ucli__i2c_start__,
    bf_pltfm_fpga_ucli_ucli__i2c_en__,
    bf_pltfm_fpga_ucli_ucli__i2c_is_running__,
    bf_pltfm_fpga_ucli_ucli__i2c_read__,
    bf_pltfm_fpga_ucli_ucli__i2c_write__,
    bf_pltfm_fpga_ucli_ucli__i2c_addr_read__,
    bf_pltfm_fpga_ucli_ucli__i2c_read_add_pr__,
    bf_pltfm_fpga_ucli_ucli__i2c_addr_read_add_pr__,
    bf_pltfm_fpga_ucli_ucli__i2c_del_pr__,
    bf_pltfm_fpga_ucli_ucli__i2c_read_data__,
    NULL};

static ucli_module_t bf_pltfm_fpga_ucli_module__ = {
    "bf_pltfm_fpga_ucli", NULL, bf_pltfm_fpga_ucli_ucli_handlers__, NULL, NULL,
};

ucli_node_t *bf_pltfm_fpga_ucli_node_create(ucli_node_t *m) {
  ucli_node_t *n;
  ucli_module_init(&bf_pltfm_fpga_ucli_module__);
  n = ucli_node_create("fpga", m, &bf_pltfm_fpga_ucli_module__);
  ucli_node_subnode_add(n, ucli_module_log_node_create("fpga"));
  return n;
}
