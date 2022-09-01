/*******************************************************************************
 * Copyright (c) 2015-2020 Barefoot Networks, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * $Id: $
 *
 ******************************************************************************/
#include <string.h>

#include <bfsys/bf_sal/bf_sys_timer.h>
#include <dvm/bf_drv_intf.h>
#include <lld/lld_gpio_if.h>
#include <bf_pltfm_types/bf_pltfm_types.h>
#include <bf_bd_cfg/bf_bd_cfg_intf.h>
#include <bf_pltfm_chss_mgmt_intf.h>
#include <bf_pltfm_fpga.h>
#include <bf_pltfm_syscpld.h>
#include <bf_led/bf_led.h>
#include <bf_pltfm_bd_cfg.h>
#include <bf_pltfm_led.h>
#include <bf_newport_led.h>
#include <bfutils/uCli/ucli.h>
#include <bfutils/uCli/ucli_argparse.h>
#include <bfutils/uCli/ucli_handler_macros.h>

/**** NEWPORT LED holding buffer organization ******
 * sysCPLD proxies as the LED driver. Each sysCPLD has got 64  8bit
 * registers. each register serves two Leds (each nibble = 1 Led). Each QSFP
 * has got 4 channels. Each channel has one LED. So, 32 ports require
 * 32*4 = 128 leds ->64 registers in sysCPLD.
 *
 * Tofino's state-out buffer is streamed out to 64 sysCPLD (+64 ) registers to
 * display the LED. so, our design of mapping of LEds contents in state-out
 * buffer is as follows:
 *
 *    byte    port/chn    port/chn
 *   _______________________________
 *
 *    0       2/0           1/0       |
 *    1       2/1           1/1       |
 *    2       2/2           1/2       |
 *    3       2/3           1/3       |
 *    4       4/0           3/0       |--upper 32 ports
 *    5       4/1           3/1       |
 *    6       4/2           3/2       |
 *    7       4/3           3/3       |
 *    8       6/0           5/0       |
 *                                    |
 *                                    |
 *                                    |
 *   63      32/3          31/3       |
 *
 *   64      34/0          33/0       |--lower 32 ports
 *   65      34/1          33/1       |
 *                                    |
 *  127      64/3          63/3       |
 *
 *   128     cpu/1        cpu/0       |--cpu port (irrespective of platform)
 *   129     cpu/3        cpu/2       |
 *
 * Note that  state-out buffer is organized to keep odd or even ports
 * contiguously together. A different organization can also be designed, but
 * that has to be implemented in the APIs accordingly. APIs in this module
 * assume above organization of state-out buffer
 */

/* The code also assumes that all i2c ports used for led belong to a single
 * ethgpio block in tofino.
 */

static int bf_pltfm_fpga_id = 0L;
static int bf_pltfm_led_cpld_initialized = 0;

/* cached value of current state-out buffer */
#define BF_LED_STATEOUT_BUF_SIZE 128

uint8_t st_out_buf[BF_LED_STATEOUT_BUF_SIZE];

/* compile time, sanity check */
#if ((BF_LED_STATEOUT_BUF_SIZE + 4) > 256)
#error Tofino cannot take so large a buffer
#endif

#if 0
static void bf_pltfm_np_cpld_i2c_reset(int fpga_id) {
  uint32_t rst_ctl;

  bf_fpga_reg_read32(fpga_id, BF_NEWPORT_FPGA_MISC_RST_CTRL_MISC, &rst_ctl);
  rst_ctl |= (1 << BF_NEWPORT_FPGA_CPLD_I2C_RST_SFT);
  /* issue reset */
  bf_fpga_reg_write32(fpga_id, BF_NEWPORT_FPGA_MISC_RST_CTRL_MISC, rst_ctl);
  bf_sys_usleep(10);
  /* issue un-reset */
  rst_ctl &= (~(1 << BF_NEWPORT_FPGA_CPLD_I2C_RST_SFT));
  bf_fpga_reg_write32(fpga_id, BF_NEWPORT_FPGA_MISC_RST_CTRL_MISC, rst_ctl);
  bf_sys_usleep(50);
}
#endif

/** platform port led subsystem initialization (for Mavericks)
 *
 *  @param chip_id
 *   chip_id
 *  @param upper
 *   cp2112 domain the leds belog to
 *  @return
 *   0 on success and -1 on error
 */
static int bf_pltfm_i2c_led_init(int chip_id, int fpga_id) {
  uint8_t wr_val[2], rd_val;

  if (chip_id >= BF_MAX_DEV_COUNT || bf_pltfm_led_cpld_initialized) {
    return -1;
  }

  /* enable tofino led access in sysCPLD(s) */
  wr_val[0] = BF_MAV_SYSCPLD_REG_LED_CTRL;
  if (bf_fpga_i2c_addr_read(fpga_id,
                            BF_NEWPORT_FPGA_BUS_CPLD,
                            0,
                            BF_NP_SYSCPLD_I2C_ADDR,
                            wr_val,
                            &rd_val,
                            1,
                            1)) {
    LOG_ERROR("Unable to read cpld LED_CTRL\n");
    return -1;
  }
  rd_val |= BF_MAV_SYSCPLD_TOF_LED_EN; /* Tofino led_en */
  wr_val[1] = rd_val;
  if (bf_fpga_i2c_write(fpga_id,
                        BF_NEWPORT_FPGA_BUS_CPLD,
                        0,
                        BF_NP_SYSCPLD_I2C_ADDR,
                        wr_val,
                        2)) {
    LOG_ERROR("Unable to write cpld LED_CTRL\n");
    return -1;
  }
  return 0;
}

/* return the byte offset in st_out_buf corresponding to a given port and chn */
#define GET_BYTE_OFF_STOUT(port, chn) (((port - 1) / 2) * QSFP_NUM_CHN + chn)

/** Platform port led set (for Mavericks) using sysCPLD access
 *
 *  @param chip_id
 *   chip_id
 *  @param port
 *   port ( -1 for all the ports)
 *  @param led_col
 *   bit-wise or of BF_NEWPORT_PORT_LED_RED, BF_NEWPORT_PORT_LED_GREEN and
 *   BF_NEWPORT_PORT_LED_BLUE
 *  @param led_blink
 *   LED_BLINK or LED_NO_BLINK
 *  @return
 *   0 on success and -1 on error
 */
int bf_pltfm_port_led_by_cpld_set(int chip_id,
                                  bf_pltfm_port_info_t *port_info,
                                  int led_col,
                                  bf_led_blink_t led_blink) {
  int port;
  int num_ports = platform_num_ports_get();
  int fpga_id = 0;
  uint8_t wr_val[2];
  uint8_t val;

  if (bf_pltfm_led_cpld_initialized == 0) {
    return -1;
  }

  val = led_col & (BF_NEWPORT_PORT_LED_RED | BF_NEWPORT_PORT_LED_GREEN |
                   BF_NEWPORT_PORT_LED_BLUE);
  if (led_blink & LED_BLINK) {
    val |= BF_NEWPORT_PORT_LED_BLINK;
  }

  if (!port_info) {
    port = -1; /* all the ports */
  } else {
    if (port_info->conn_id <= 0 || port_info->conn_id > (uint32_t)num_ports ||
        port_info->chnl_id > 7) {
      return -1;
    }
    port = port_info->conn_id;
  }

  if (port >= num_ports) {
    return -1;
  }
  if (port_info->chnl_id > 3) {
    // FIXME temporary for new port. Pretend that led is set
    return 0;
  }
  wr_val[1] = val;
  if (port != -1) { /* write to a single sysCPLD register */
    uint32_t byte_offset =
        GET_BYTE_OFF_STOUT(port_info->conn_id, port_info->chnl_id);
    uint8_t st_out_byte = st_out_buf[byte_offset];
    if (port % 2) { /* use lower nibble for odd ports */
      st_out_byte &= 0xF0;
      st_out_byte |= val;
    } else {
      st_out_byte &= 0x0F;
      st_out_byte |= (val << 4);
    }
    wr_val[0] = BF_MAV_SYSCPLD_REG_LED_START + byte_offset;
    wr_val[1] = st_out_byte;
    if (bf_fpga_i2c_write(fpga_id,
                          BF_NEWPORT_FPGA_BUS_CPLD,
                          0,
                          BF_NP_SYSCPLD_I2C_ADDR,
                          wr_val,
                          2)) {
      return -1;
    }
    /* update cached value of st_out_buf if everything went well */
    st_out_buf[byte_offset] = st_out_byte;
  }
  if (port == -1) {
    int i;
    wr_val[1] |= (val << 4); /* duplice the adjecent nibbles */
    for (i = 0; i < BF_MAV_SYSCPLD_NUM_REG_LED; i++) {
      wr_val[0] = BF_MAV_SYSCPLD_REG_LED_START + i;
      if (bf_fpga_i2c_write(fpga_id,
                            BF_NEWPORT_FPGA_BUS_CPLD,
                            0,
                            BF_NP_SYSCPLD_I2C_ADDR,
                            wr_val,
                            2)) {
        return -1;
      }
    }
    memset(st_out_buf, val, sizeof(st_out_buf)); /* update cached buffer */
  }
  return 0;
}

/** Platform port led set (for Mavericks) using sysCPLD access
 *
 *  @param chip_id
 *   chip_id
 *  @param port_info
 *    bf_pltfm_port_info_t *, NULL for all ports
 *  @param led_cond
 *   bf_led_condition_t
 *  @return
 *   0 on success and -1 on error
 */
int bf_pltfm_port_led_set(int chip_id,
                          bf_pltfm_port_info_t *port_info,
                          bf_led_condition_t led_cond) {
  uint8_t val;

  if (bf_pltfm_led_cpld_initialized == 0) {
    return -1;
  }

  switch (led_cond) {
    case BF_LED_POST_PORT_DIS:
      val = BF_NEWPORT_PORT_LED_RED | BF_NEWPORT_PORT_LED_GREEN;
      break;
    case BF_LED_PRE_PORT_EN:
    case BF_LED_PORT_LINK_DOWN:
      val = BF_NEWPORT_PORT_LED_BLUE | BF_NEWPORT_PORT_LED_GREEN;
      break;
    case BF_LED_PORT_LINK_UP:
      val = BF_NEWPORT_PORT_LED_GREEN;
      break;
    case BF_LED_PORT_RED:
      val = BF_NEWPORT_PORT_LED_RED;
      break;
    case BF_LED_PORT_GREEN:
      val = BF_NEWPORT_PORT_LED_GREEN;
      break;
    case BF_LED_PORT_BLUE:
      val = BF_NEWPORT_PORT_LED_BLUE;
      break;
    case BF_LED_POST_PORT_DEL:
    default:
      val = BF_NEWPORT_PORT_LED_OFF;
      break;
  }
  return (bf_pltfm_port_led_by_cpld_set(chip_id, port_info, val, LED_NO_BLINK));
}

/* Init function  */
int bf_pltfm_led_cpld_init(int chip_id) {
  bf_pltfm_status_t sts;
  int num_ports, i, j;
  bf_pltfm_port_info_t port_info;

  bf_pltfm_fpga_id = 0;
  /* pin pair 0:1 go to FPGA, 2:3 not used on Newport */
  if (bf_pltfm_i2c_led_init(chip_id, bf_pltfm_fpga_id)) {
    LOG_ERROR("error upper i2c_led_init\n");
    return -1;
  }

  bf_pltfm_led_cpld_initialized = 1;

  num_ports = platform_num_ports_get();
  for (i = 1; i <= (num_ports - 1); i++) {
    port_info.conn_id = i;
    for (j = 0; j < QSFP_NUM_CHN; j++) {
      port_info.chnl_id = j;
      sts = bf_pltfm_port_led_set(chip_id, &port_info, BF_LED_POST_PORT_DEL);
      if (sts != BF_PLTFM_SUCCESS) {
        LOG_ERROR("Unable to reset LED on port %d/%d",
                  port_info.conn_id,
                  port_info.chnl_id);
      }
    }
  }

  return 0;
}
