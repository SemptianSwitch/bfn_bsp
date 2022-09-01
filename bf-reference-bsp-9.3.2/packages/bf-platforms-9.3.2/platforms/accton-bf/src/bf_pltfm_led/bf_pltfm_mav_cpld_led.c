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
#include <bf_pltfm_cp2112_intf.h>
#include <bf_pltfm_syscpld.h>
#include <bf_led/bf_led.h>
#include <bf_pltfm_bd_cfg.h>
#include <bf_pltfm_led.h>
#include <bf_mav_led.h>
#include <bfutils/uCli/ucli.h>
#include <bfutils/uCli/ucli_argparse.h>
#include <bfutils/uCli/ucli_handler_macros.h>

/**** MAVERICKS LED holding buffer organization ******
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

static bf_pltfm_cp2112_device_ctx_t *led_hndl[CP2112_ID_MAX] = {NULL, NULL};
static int bf_pltfm_led_cpld_initialized = 0;

#define MAV_LED_PIN_PAIR_UPPER 0
#define MAV_LED_PIN_PAIR_LOWER 2

#define TIMEOUT_100MS 100
#define PERIOD_400KHZ_IN_10_NSEC               \
  0x2C /* value acquired from ASSIC simulation \
          */

/* cached value of current state-out buffer */
#define BF_LED_STATEOUT_BUF_SIZE 128

uint8_t st_out_buf[BF_LED_STATEOUT_BUF_SIZE];
uint8_t st_out_cpu[4]; /* 2 bytes: CPU led, 2 extra: for 32 bit access`*/

/* compile time, sanity check */
#if ((BF_LED_STATEOUT_BUF_SIZE + 4) > 256)
#error Tofino cannot take so large a buffer
#endif

/* cached value of current led display selection `*/
// static bool even_ports_selected = false;

/** platform port led subsystem initialization (for Mavericks)
 *
 *  @param chip_id
 *   chip_id
 *  @param upper
 *   cp2112 domain the leds belog to
 *  @return
 *   0 on success and -1 on error
 */
static int bf_pltfm_i2c_led_init(int chip_id, bf_pltfm_cp2112_id_t upper) {
  bf_pltfm_cp2112_device_ctx_t *hndl = led_hndl[upper];
  uint8_t wr_val[2], rd_val;

  if (chip_id >= BF_MAX_DEV_COUNT || bf_pltfm_led_cpld_initialized || !hndl) {
    return -1;
  }

  /* enable tofino led access in sysCPLD(s) */
  wr_val[0] = BF_MAV_SYSCPLD_REG_LED_CTRL;
  if (bf_pltfm_cp2112_write_read_unsafe(hndl,
                                        BF_MAV_SYSCPLD_I2C_ADDR,
                                        wr_val,
                                        &rd_val,
                                        1,
                                        1,
                                        TIMEOUT_100MS)) {
    return -1;
  }
  rd_val |= BF_MAV_SYSCPLD_TOF_LED_EN; /* Tofino led_en */
  wr_val[1] = rd_val;
  if (bf_pltfm_cp2112_write(
          hndl, BF_MAV_SYSCPLD_I2C_ADDR, wr_val, 2, TIMEOUT_100MS)) {
    return -1;
  }
  return 0;
}

/* return the byte offset in st_out_buf corresponding to a given port and chn */
#define GET_BYTE_OFF_STOUT(port, chn) (((port - 1) / 2) * QSFP_NUM_CHN + chn)

/** Platform cpu port led set by writing into cpld
 *
 *  @param chip_id
 *   chip_id
 *  @param chn
 *   channel (0-3)
 *  @param val
 *   bit-wise or of BF_MAV_PORT_LED_RED, BF_MAV_PORT_LED_GREEN,
 *   BF_MAV_PORT_LED_BLUE and BF_MAV_PORT_LED_BLINK
 *  @return
 *   0 on success and -1 on error
 */
static int bf_pltfm_cpu_port_led_by_cpld_set(int chip_id,
                                             int chn,
                                             uint8_t val) {
  bf_pltfm_cp2112_device_ctx_t *hndl_u;
  uint8_t wr_val[3];

  if (chn > QSFP_NUM_CHN) {
    return -1;
  }
  hndl_u = led_hndl[CP2112_ID_1];

  /* update cached value of state-out-buf */
  val &= 0xF; /* take only a nibble */
  if (chn == 0) {
    st_out_cpu[0] &= 0xF0;
    st_out_cpu[0] |= val;
  } else if (chn == 1) {
    st_out_cpu[0] &= 0x0F;
    st_out_cpu[0] |= (val << 4);
  } else if (chn == 2) {
    st_out_cpu[1] &= 0xF0;
    st_out_cpu[1] |= val;
  } else {
    st_out_cpu[1] &= 0x0F;
    st_out_cpu[1] |= (val << 4);
  }
  /* write to LED */
  wr_val[0] = BF_MAV_SYSCPLD_REG_CPU_LED_START;
  wr_val[1] = st_out_cpu[0];
  wr_val[2] = st_out_cpu[1];
  if (bf_pltfm_cp2112_write(
          hndl_u, BF_MAV_SYSCPLD_I2C_ADDR, wr_val, 3, TIMEOUT_100MS)) {
    return -1;
  }
  return 0;
}

/** Platform port led set (for Mavericks) using sysCPLD access
 *
 *  @param chip_id
 *   chip_id
 *  @param port
 *   port ( -1 for all the ports)
 *  @param led_col
 *   bit-wise or of BF_MAV_PORT_LED_RED, BF_MAV_PORT_LED_GREEN and
 *   BF_MAV_PORT_LED_BLUE
 *  @param led_blink
 *   LED_BLINK or LED_NO_BLINK
 *  @return
 *   0 on success and -1 on error
 */
int bf_pltfm_port_led_by_cpld_set(int chip_id,
                                  bf_pltfm_port_info_t *port_info,
                                  int led_col,
                                  bf_led_blink_t led_blink) {
  bf_pltfm_cp2112_device_ctx_t *hndl_l, *hndl_u;
  bf_pltfm_board_id_t board_id;
  bf_pltfm_status_t sts;
  int port;
  int num_ports = platform_num_ports_get();
  uint8_t wr_val[2];
  uint8_t val;

  if (bf_pltfm_led_cpld_initialized == 0) {
    return -1;
  }

  sts = bf_pltfm_chss_mgmt_bd_type_get(&board_id);
  if (sts != BF_PLTFM_SUCCESS) {
    return -1;
  }
  val = led_col &
        (BF_MAV_PORT_LED_RED | BF_MAV_PORT_LED_GREEN | BF_MAV_PORT_LED_BLUE);
  if (led_blink & LED_BLINK) {
    val |= BF_MAV_PORT_LED_BLINK;
  }
  hndl_u = led_hndl[CP2112_ID_1];
  hndl_l = led_hndl[CP2112_ID_2];

  if (!port_info) {
    port = -1; /* all the ports */
  } else {
    if (port_info->conn_id <= 0 || port_info->conn_id > (uint32_t)num_ports ||
        port_info->chnl_id > 3) {
      return -1;
    }
    port = port_info->conn_id;
  }

  if (port == num_ports) {
    return bf_pltfm_cpu_port_led_by_cpld_set(chip_id, port_info->chnl_id, val);
  }

  wr_val[1] = val;
  if (port != -1) { /* write to a single sysCpld register */
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
    wr_val[1] = st_out_byte;
    if ((board_id == BF_PLTFM_BD_ID_MONTARA_P0B) ||
        (board_id == BF_PLTFM_BD_ID_MONTARA_P0C) ||
        (board_id == BF_PLTFM_BD_ID_MONTARA_P0A)) {
      wr_val[0] = BF_MAV_SYSCPLD_REG_LED_START + byte_offset;
      if (bf_pltfm_cp2112_write(
              hndl_u, BF_MAV_SYSCPLD_I2C_ADDR, wr_val, 2, TIMEOUT_100MS)) {
        return -1;
      }
    } else if ((board_id == BF_PLTFM_BD_ID_MAVERICKS_P0B) ||
               (board_id == BF_PLTFM_BD_ID_MAVERICKS_P0A) ||
               (board_id == BF_PLTFM_BD_ID_MAVERICKS_P0C)) {
      if (port <= 32) { /* upper cpld */
        wr_val[0] = BF_MAV_SYSCPLD_REG_LED_START + byte_offset;
        if (bf_pltfm_cp2112_write(
                hndl_u, BF_MAV_SYSCPLD_I2C_ADDR, wr_val, 2, TIMEOUT_100MS)) {
          return -1;
        }
      } else {
        wr_val[0] = BF_MAV_SYSCPLD_REG_LED_START + byte_offset -
                    BF_MAV_SYSCPLD_NUM_REG_LED;
        if (bf_pltfm_cp2112_write(
                hndl_l, BF_MAV_SYSCPLD_I2C_ADDR, wr_val, 2, TIMEOUT_100MS)) {
          return -1;
        }
      }
    }
    /* update cached value of st_out_buf if everything went well */
    st_out_buf[byte_offset] = st_out_byte;
  }
  if (port == -1) {
    int i;
    wr_val[1] |= (val << 4); /* duplice the adjecent nibbles */
    for (i = 0; i < BF_MAV_SYSCPLD_NUM_REG_LED; i++) {
      wr_val[0] = BF_MAV_SYSCPLD_REG_LED_START + i;
      if (bf_pltfm_cp2112_write(
              hndl_u, BF_MAV_SYSCPLD_I2C_ADDR, wr_val, 2, TIMEOUT_100MS)) {
        return -1;
      }

      if ((board_id == BF_PLTFM_BD_ID_MAVERICKS_P0B) ||
          (board_id == BF_PLTFM_BD_ID_MAVERICKS_P0A) ||
          (board_id == BF_PLTFM_BD_ID_MAVERICKS_P0C)) {
        if (bf_pltfm_cp2112_write(
                hndl_l, BF_MAV_SYSCPLD_I2C_ADDR, wr_val, 2, TIMEOUT_100MS)) {
          //  return -1;
        }
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
  bf_pltfm_board_id_t board_id;
  bf_pltfm_status_t sts;
  uint8_t val;

  if (bf_pltfm_led_cpld_initialized == 0) {
    return -1;
  }

  sts = bf_pltfm_chss_mgmt_bd_type_get(&board_id);
  if (sts != BF_PLTFM_SUCCESS) {
    return -1;
  }
  switch (led_cond) {
    case BF_LED_POST_PORT_DIS:
      val = BF_MAV_PORT_LED_RED | BF_MAV_PORT_LED_GREEN;
      break;
    case BF_LED_PRE_PORT_EN:
    case BF_LED_PORT_LINK_DOWN:
      val = BF_MAV_PORT_LED_BLUE | BF_MAV_PORT_LED_GREEN;
      break;
    case BF_LED_PORT_LINK_UP:
      val = BF_MAV_PORT_LED_GREEN;
      break;
    case BF_LED_PORT_RED:
      val = BF_MAV_PORT_LED_RED;
      break;
    case BF_LED_PORT_GREEN:
      val = BF_MAV_PORT_LED_GREEN;
      break;
    case BF_LED_PORT_BLUE:
      val = BF_MAV_PORT_LED_BLUE;
      break;
    case BF_LED_POST_PORT_DEL:
    default:
      val = BF_MAV_PORT_LED_OFF;
      break;
  }
  return (bf_pltfm_port_led_by_cpld_set(chip_id, port_info, val, LED_NO_BLINK));
}

/* Init function  */
int bf_pltfm_led_cpld_init(int chip_id) {
  bf_pltfm_status_t sts;
  bf_pltfm_board_id_t board_id;
  int num_ports, i, j;
  bf_pltfm_port_info_t port_info;

  sts = bf_pltfm_chss_mgmt_bd_type_get(&board_id);
  if (sts != BF_PLTFM_SUCCESS) {
    return -1;
  }
  led_hndl[CP2112_ID_1] = bf_pltfm_cp2112_get_handle(CP2112_ID_1);
  if (!led_hndl[CP2112_ID_1]) {
    LOG_ERROR("error getting upper cp2112 handle in led_init\n");
    return -1;
  }
  /* pin pair 0:1 go to upper CPLD, 2:3 go to lower CPLD */
  if (bf_pltfm_i2c_led_init(chip_id, CP2112_ID_1)) {
    LOG_ERROR("error upper i2c_led_init\n");
    return -1;
  }
  if ((board_id == BF_PLTFM_BD_ID_MAVERICKS_P0B) ||
      (board_id == BF_PLTFM_BD_ID_MAVERICKS_P0A) ||
      (board_id == BF_PLTFM_BD_ID_MAVERICKS_P0C)) {
    led_hndl[CP2112_ID_2] = bf_pltfm_cp2112_get_handle(CP2112_ID_2);
    if (!led_hndl[CP2112_ID_2]) {
      LOG_ERROR("error getting lower cp2112 handle in led_init\n");
      return -1;
    }
    if (bf_pltfm_i2c_led_init(chip_id, CP2112_ID_2)) {
      LOG_ERROR("error lower i2c_led_init\n");
      return -1;
    }
  }

  bf_pltfm_led_cpld_initialized = 1;

  num_ports = platform_num_ports_get();
  for (i = 1; i <= num_ports; i++) {
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
