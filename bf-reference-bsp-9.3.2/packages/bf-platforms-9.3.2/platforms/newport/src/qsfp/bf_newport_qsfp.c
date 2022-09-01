/*******************************************************************************
 * Copyright (c) 2015-2020 Barefoot Networks, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * $Id: $
 *
 ******************************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>

#include <bf_types/bf_types.h>
#include <bf_pltfm_types/bf_pltfm_types.h>
#include <bf_bd_cfg/bf_bd_cfg_intf.h>
#include <bf_qsfp/bf_qsfp.h>
#include <bf_pltfm_bd_cfg.h>
#include <bf_pltfm_qsfp.h>
#include <bf_pltfm_fpga.h>

#include "bf_newport_qsfp_priv.h"
#include <bf_pcal9535.h>

static bool qsfp_present_st[NP_MAX_NUM_OF_QSFP + 1];

// Port-1 to 32; ignore 0.
static uint8_t qsfp_port_to_pca_bitmap_for_np_P0A[NP_MAX_NUM_OF_QSFP + 1] = {
    0xff, 0, 1, 2, 3, 4, 5, 6, 7, 9, 8,  11, 10, 13, 12, 15, 14,
    1,    0, 3, 2, 5, 4, 7, 6, 8, 9, 10, 11, 13, 12, 15, 14};

static int max_qsfp_ports = 0;
static int cur_bank[NP_MAX_NUM_OF_QSFP + 1];  // default bank is 0
static int cur_page[NP_MAX_NUM_OF_QSFP + 1];  // default page is 0

// Gaint-lock for everything
static bf_sys_mutex_t np_mod_access_lock;
#define NP_MODULE_LOCK_INIT bf_sys_mutex_init(&np_mod_access_lock)
#define NP_MODULE_LOCK_DEL bf_sys_mutex_del(&np_mod_access_lock)
#define NP_MODULE_LOCK bf_sys_mutex_lock(&np_mod_access_lock)
#define NP_MODULE_UNLOCK bf_sys_mutex_unlock(&np_mod_access_lock)

static int bf_np_mod_write(unsigned int port,
                           int offset,
                           int len,
                           uint8_t *buf);

static void bf_np_qsfp_set_max_qsfp_ports(unsigned int num_of_ports) {
  max_qsfp_ports = num_of_ports;
}

// Default GPIO states
static int bf_np_pca9535_init(void) {
  uint16_t port;
  uint8_t device, i;
  uint16_t device_channel = 0;
  qsfp_i2c_msg_t msg;
  bf_pltfm_status_t ret = BF_PLTFM_SUCCESS;
  bool latch = true;
  msg.mux_present = true;
  msg.delay = 0;
  msg.mux_addr = NP_FPGA_QSFP_GPIO_I2CMUX_ADDR;
  port = 1;                 // lower-port
  device_channel = 0xffff;  // qsfp_port_to_pca_bitmap_for_np_P0A[0];

  for (i = 0; i < 2; i++, port = 17) {
    // IO in input mode, enable latch, clear pol-inv
    msg.mux_channel = GPIO_NP_GET_MUX_CHAN_FOR_QSFP_PRSNT(port);
    device = GPIO_NP_GET_I2CADDR_FOR_QSFP_PRSNT(port);
    ret = pcal9535_bf_set_config_reg(
        device, 0, device_channel, true, (void *)&msg);
    if (ret != BF_PLTFM_SUCCESS) {
      LOG_ERROR(
          "%s Error in configuring qsfp present 0x%0x\n", __func__, device);
      goto end;
    }

    ret = pcal9535_bf_enable_input_latch(
        device, 0, device_channel, latch, (void *)&msg);
    if (ret != BF_PLTFM_SUCCESS) {
      LOG_ERROR(
          "%s Error in configuring input latch 0x%0x\n", __func__, device);
      goto end;
    }

    ret =
        pcal9535_bf_set_pol_inv(device, 0, device_channel, false, (void *)&msg);
    if (ret != BF_PLTFM_SUCCESS) {
      LOG_ERROR(
          "%s Error in configuring input latch 0x%0x\n", __func__, device);
      goto end;
    }

    // IO in output mode and LP in mode
    msg.mux_channel = GPIO_NP_GET_MUX_CHAN_FOR_QSFP_LP(port);
    device = GPIO_NP_GET_I2CADDR_FOR_QSFP_LP(port);
    ret = pcal9535_bf_set_config_reg(
        device, 0, device_channel, false, (void *)&msg);
    if (ret != BF_PLTFM_SUCCESS) {
      LOG_ERROR(
          "%s Error in configuring LP output pin 0x%0x\n", __func__, device);
      goto end;
    }

    ret = pcal9535_bf_set_output_port(
        device, 0, device_channel, true, (void *)&msg);
    if (ret != BF_PLTFM_SUCCESS) {
      LOG_ERROR("%s Error in setting LP output val 0x%0x\n", __func__, device);
      goto end;
    }

    // IO in output mode and keep in assert
    msg.mux_channel = GPIO_NP_GET_MUX_CHAN_FOR_QSFP_RST(port);
    device = GPIO_NP_GET_I2CADDR_FOR_QSFP_RST(port);
    ret = pcal9535_bf_set_config_reg(
        device, 0, device_channel, false, (void *)&msg);
    if (ret != BF_PLTFM_SUCCESS) {
      LOG_ERROR(
          "%s Error in configuring LP output pin 0x%0x\n", __func__, device);
      goto end;
    }
    ret = pcal9535_bf_set_output_port(
        device, 0, device_channel, 0 /* 0:reset */, (void *)&msg);
    if (ret != BF_PLTFM_SUCCESS) {
      LOG_ERROR("%s Error in setting RST output val 0x%0x\n", __func__, device);
      goto end;
    }

    // Interrupts are masked by default
  }

  return 0;

end:
  return -1;
}

/** detect a qsfp presence
 *
 *  @param module
 *   port (1 based)
 *  @return
 *   true if present and false if not present
 */
bool bf_pltfm_detect_qsfp(unsigned int port) {
  if (port > (unsigned int)max_qsfp_ports) {
    LOG_ERROR("Error Invalid QSFP: %d index\n", port);
    return false;
  }

  // Since we latch the presence, simply return cached value.
  return qsfp_present_st[port];
}

static int bf_np_mod_read(unsigned int port,
                          int offset,
                          int rd_len,
                          uint8_t *buf) {
  uint8_t device;
  bf_pltfm_status_t ret = BF_PLTFM_SUCCESS;
  uint8_t wr_buf = offset;
  uint8_t wr_len = 1;
  int err = 0;

  if ((port > (unsigned int)max_qsfp_ports) || (!buf)) {
    LOG_ERROR("%s Error Invalid QSFP: %d index or buf ptr\n", __func__, port);
    return -1;
  }

  NP_MODULE_LOCK;
  device = NP_FPGA_QSFP_I2CBUS_ADDR(port);

  while (rd_len) {
    if (rd_len >= BF_NEWPORT_FPGA_MAX_ACCESS_LEN) {
      ret |= bf_fpga_i2c_addr_read(0,
                                   device,
                                   0,
                                   QSFP_I2C_ADDR,
                                   &wr_buf,
                                   buf,
                                   wr_len,
                                   BF_NEWPORT_FPGA_MAX_ACCESS_LEN);
      buf += BF_NEWPORT_FPGA_MAX_ACCESS_LEN;
      wr_buf += BF_NEWPORT_FPGA_MAX_ACCESS_LEN;
      rd_len -= BF_NEWPORT_FPGA_MAX_ACCESS_LEN;
    } else {
      ret |= bf_fpga_i2c_addr_read(
          0, device, 0, QSFP_I2C_ADDR, &wr_buf, buf, wr_len, rd_len);
      rd_len = 0;
    }
    // Eventhough costly, some modules tested needed this delay
    usleep(NP_QSFP_RD_ACCESS_SLEEP_TIME);
  }

  if (ret != BF_PLTFM_SUCCESS) {
    LOG_ERROR("QSFP    %2d : Error reading on device 0x%0x\n", port, device);
    err = -1;
  }

  NP_MODULE_UNLOCK;
  return err;
}

// Invalidates the remembered current bank and page
static void bf_np_invalidate_bankpage(unsigned int port) {
  cur_page[port] = -1;
  cur_bank[port] = -1;
}

// Checks if the requested bank or page differs from what is active
// if yes, changes them
static int bf_np_update_bank_page(unsigned int port,
                                  uint8_t bank,
                                  uint8_t page) {
  if (bf_qsfp_is_cmis(port)) {
    if (bank != cur_bank[port]) {
      // CMIS spec requires that the bank and page registers be written in the
      // same I2C transaction if the bank is changing, even if the page isn't
      uint8_t bankpage[2];
      bankpage[0] = bank;
      bankpage[1] = page;
      if (bf_np_mod_write(port, 126, 2, bankpage) < 0) {
        return -1;
      }
      LOG_DEBUG("QSFP    %2d : bank page change, was %d/%d, is %d/%d",
                port,
                cur_bank[port],
                cur_page[port],
                bank,
                page);
    } else if (page != cur_page[port]) {
      // Only page is changing
      if (bf_np_mod_write(port, 127, 1, &page) < 0) {
        return -1;
      }
      LOG_DEBUG("QSFP    %2d : bank page change, was %d/%d, is %d/%d",
                port,
                cur_bank[port],
                cur_page[port],
                bank,
                page);
    }
    cur_bank[port] = bank;
    cur_page[port] = page;
  } else {  // SFF-8636
    if (page != cur_page[port]) {
      if (bf_np_mod_write(port, 127, 1, &page) < 0) {
        return -1;
      }
      LOG_DEBUG("QSFP    %2d : page change, was %d, is %d",
                port,
                cur_page[port],
                page);
      cur_page[port] = page;
    }
  }

  return 0;
}

static int bf_np_mod_write(unsigned int port,
                           int offset,
                           int len,
                           uint8_t *buf) {
  uint8_t device;
  bf_pltfm_status_t ret = BF_PLTFM_SUCCESS;
  uint8_t wr_buf[63];
  int wr_size;
  int err = 0;

  if ((port > (unsigned int)max_qsfp_ports) || (!buf)) {
    LOG_ERROR("%s Error Invalid QSFP: %d index or buf ptr\n", __func__, port);
    return -1;
  }

  NP_MODULE_LOCK;
  // usually nothing is written that is more than 2 bytes into QSFP. Add
  // cautionary log here.
  if (len > BF_NEWPORT_FPGA_MAX_ACCESS_LEN) {
    LOG_ERROR(
        "%s Error Invalid Write length for QSFP: %d index or buf ptr. Send "
        "less "
        "than bytes %d\n",
        __func__,
        port,
        BF_NEWPORT_FPGA_MAX_ACCESS_LEN);
    err = -1;
    goto wr_end;
  }

  device = NP_FPGA_QSFP_I2CBUS_ADDR(port);

  wr_buf[0] = offset;
  memcpy(&wr_buf[1], buf, len);
  wr_size = len + 1;

  ret = bf_fpga_i2c_write(0, device, 0, QSFP_I2C_ADDR, wr_buf, wr_size);
  if (ret != BF_PLTFM_SUCCESS) {
    LOG_ERROR("Error writing QSFP:%d on device:0x%0x\n", port, device);
    err = -1;
    goto wr_end;
  }
  // CMIS can take upto 10msec delay for command completion.
  // Plus sff-8636 modules tested, required this delay.
  usleep(NP_QSFP_WR_ACCESS_SLEEP_TIME);

wr_end:
  NP_MODULE_UNLOCK;
  return err;
}

/** reset qsfp module
 *
 *  @param port
 *   module (1 based)
 *  @param reg
 *   syscpld register to write
 *  @param val
 *   value to write
 *  @return
 *   0 on success and -1 in error
 */
int bf_pltfm_qsfp_module_reset(int port, bool reset) {
  uint8_t device;
  uint16_t device_channel = 0;
  qsfp_i2c_msg_t msg;
  bf_pltfm_status_t ret = BF_PLTFM_SUCCESS;
  int err = 0;

  if (port > max_qsfp_ports) {
    LOG_ERROR("Error Invalid QSFP: %d index\n", port);
    return -1;
  }

  NP_MODULE_LOCK;
  msg.mux_present = true;
  msg.delay = 0;
  msg.mux_addr = NP_FPGA_QSFP_GPIO_I2CMUX_ADDR;
  msg.mux_channel = GPIO_NP_GET_MUX_CHAN_FOR_QSFP_RST(port);
  device = GPIO_NP_GET_I2CADDR_FOR_QSFP_RST(port);
  device_channel = qsfp_port_to_pca_bitmap_for_np_P0A[port];
  ret = pcal9535_bf_set_output_port(
      device, 0, device_channel, !reset, (void *)&msg);
  if (ret != BF_PLTFM_SUCCESS) {
    LOG_ERROR("Error in QSFP:%d reset on device:0x%0x\n", port, device);
    err = -1;
    goto end;
  }
end:
  NP_MODULE_UNLOCK;
  return err;
}

typedef union pres_data_ {
  uint32_t data;
  uint16_t pres[2];
} pres_t;

/** get qsfp presence mask status
 *
 *  @param port_1_32_pres
 *   mask for lower 32 ports (1-32) 0: present, 1:not-present
 *  @param port_32_64_pres
 *   mask for upper 32 ports (33-64) 0: present, 1:not-present
 *  @param port_cpu_pres
 *   mask for cu port presence
 *  @return
 *   0 on success and -1 in error
 */
int bf_pltfm_qsfp_get_presence_mask(uint32_t *port_1_32_pres,
                                    uint32_t *port_32_64_pres,
                                    uint32_t *port_cpu_pres) {
  uint8_t device;
  uint16_t device_channel = 0;
  qsfp_i2c_msg_t msg;
  bf_pltfm_status_t ret = BF_PLTFM_SUCCESS;
  uint32_t port;
  // uint16_t pres_hi, pres_lo;
  pres_t presence_latched;
  pres_t presence;  // current
  uint32_t pos;
  int err = 0;

  if (!port_1_32_pres) return -1;
  if (port_32_64_pres) *port_32_64_pres = 0xFFFFFFFFUL;  // absent
  if (port_cpu_pres) *port_cpu_pres = 0xFFFFFFFFUL;

  *port_1_32_pres = presence.data = 0xFFFFFFFFUL;
  NP_MODULE_LOCK;
  msg.mux_present = true;
  msg.delay = 0;
  msg.mux_addr = NP_FPGA_QSFP_GPIO_I2CMUX_ADDR;
  port = 1;
  device_channel = 0xffff;
  msg.mux_channel = GPIO_NP_GET_MUX_CHAN_FOR_QSFP_PRSNT(port);
  device = GPIO_NP_GET_I2CADDR_FOR_QSFP_PRSNT(port);
  ret = pcal9535_bf_get_input_port(
      device, 0, device_channel, &presence_latched.pres[0], (void *)&msg);
  ret |= pcal9535_bf_get_input_port(
      device, 0, device_channel, &presence.pres[0], (void *)&msg);
  if (ret != BF_PLTFM_SUCCESS) {
    LOG_ERROR("Error in getting present val on device:0x%0x\n", device);
  }

  port = 17;
  device_channel = 0xffff;
  msg.mux_channel = GPIO_NP_GET_MUX_CHAN_FOR_QSFP_PRSNT(port);
  device = GPIO_NP_GET_I2CADDR_FOR_QSFP_PRSNT(port);
  ret = pcal9535_bf_get_input_port(
      device, 0, device_channel, &presence_latched.pres[1], (void *)&msg);
  ret |= pcal9535_bf_get_input_port(
      device, 0, device_channel, &presence.pres[1], (void *)&msg);
  if (ret != BF_PLTFM_SUCCESS) {
    LOG_ERROR("Error in getting present val on device:0x%0x\n", device);
    err = -1;
    goto pres_end;
  }

  // Plug-Unplug-Plug : The module would have changed here & application
  //                    needs to know inorder to clear cached info.
  //                    So read once.
  // Unplug-Plug-Unplug : Reread to avoid access error from the application.
  // All other cases read once.

  for (port = 1; port < NP_MAX_NUM_OF_QSFP + 1; port++) {
    pos = qsfp_port_to_pca_bitmap_for_np_P0A[port];
    if (port > 16) {
      pos += 16;
    }
    // unplugged-plug-unplug
    if ((!qsfp_present_st[port]) && (!(presence_latched.data & (1 << pos))) &&
        (presence.data & (1 << pos))) {
      LOG_DEBUG("QSFP    %2d : unplugged-plugged-unplugged\n", port);
      *port_1_32_pres |= (1 << (port - 1));
    } else {
      if (!(presence.data & (1 << pos))) {
        *port_1_32_pres &= ~(1 << (port - 1));
        if (!qsfp_present_st[port]) {
          bf_np_invalidate_bankpage(port);
          LOG_DEBUG("QSFP    %2d : plugged-in\n", port);
          qsfp_present_st[port] = true;
        }
      } else {
        *port_1_32_pres |= (1 << (port - 1));
        // unplugged
        if (qsfp_present_st[port]) {
          LOG_DEBUG("QSFP    %2d : unplugged\n", port);
          qsfp_present_st[port] = false;
        }
      }
    }
  }
pres_end:

  NP_MODULE_UNLOCK;
  return err;
}

/** get qsfp interrupt status
 *
 *  @param port_1_32_ints
 *   interrupt from lower 32 ports (1-32) 0: int-present, 1:not-present
 *  @param port_32_64_ints
 *   interrupt from upper 32 ports (33-64) 0: int-present, 1:not-present
 *  @param port_cpu_ints
 *   mask for cpu port interrupt
 */
void bf_pltfm_qsfp_get_int_mask(uint32_t *port_1_32_ints,
                                uint32_t *port_32_64_ints,
                                uint32_t *port_cpu_ints) {
  (void)port_1_32_ints;
  (void)port_32_64_ints;
  (void)port_cpu_ints;
  // not supported on newport.
}

/** get qsfp lpmode status
 *
 *  @param port_1_32_lpmode_
 *   lpmode of lower 32 ports (1-32) 0: no-lpmod 1: lpmode
 *  @param port_32_64_ints
 *   lpmode of upper 32 ports (33-64) 0: no-lpmod 1: lpmode
 *  @param port_cpu_ints
 *   lpmode of cpu port
 */
int bf_pltfm_qsfp_get_lpmode_mask(uint32_t *port_1_32_lpm,
                                  uint32_t *port_32_64_lpm,
                                  uint32_t *port_cpu_lpm) {
  uint8_t device;
  uint16_t device_channel = 0;
  qsfp_i2c_msg_t msg;
  bf_pltfm_status_t ret = BF_PLTFM_SUCCESS;
  int port = 1;
  uint16_t data16 = 0;

  if (!port_1_32_lpm) return -1;

  NP_MODULE_LOCK;
  msg.mux_present = true;
  msg.delay = 0;
  msg.mux_addr = NP_FPGA_QSFP_GPIO_I2CMUX_ADDR;
  msg.mux_channel = GPIO_NP_GET_MUX_CHAN_FOR_QSFP_LP(port);
  device = GPIO_NP_GET_I2CADDR_FOR_QSFP_LP(port);
  device_channel = 0xffff;  // qsfp_port_to_pca_bitmap_for_np_P0A[port];
  ret = pcal9535_bf_get_output_port(
      device, 0, device_channel, &data16, (void *)&msg);
  if (ret != BF_PLTFM_SUCCESS) {
    LOG_ERROR("Error in getting LP output val 0x%0x\n", device);
  }

  *port_1_32_lpm = data16;
  (void)port_32_64_lpm;
  (void)port_cpu_lpm;

  NP_MODULE_UNLOCK;
  return ret;
}

/** set qsfp lpmode
 *
 *  @param port
 *   port
 *  @param lpmode
 *   true : set lpmode, false : set no-lpmode
 *  @return
 *   0: success, -1: failure
 */
int bf_pltfm_qsfp_set_lpmode(int port, bool lpmode) {
  uint8_t device;
  uint16_t device_channel = 0;
  qsfp_i2c_msg_t msg;
  bf_pltfm_status_t ret = BF_PLTFM_SUCCESS;

  if (port > max_qsfp_ports) {
    LOG_ERROR("Error Invalid QSFP: %d index\n", port);
    return -1;
  }

  NP_MODULE_LOCK;
  // 0 is high-power mode on pin
  msg.mux_present = true;
  msg.delay = 0;
  msg.mux_addr = NP_FPGA_QSFP_GPIO_I2CMUX_ADDR;
  msg.mux_channel = GPIO_NP_GET_MUX_CHAN_FOR_QSFP_LP(port);
  device = GPIO_NP_GET_I2CADDR_FOR_QSFP_LP(port);
  device_channel = qsfp_port_to_pca_bitmap_for_np_P0A[port];
  ret = pcal9535_bf_set_output_port(
      device, 0, device_channel, lpmode, (void *)&msg);
  if (ret != 0) {
    LOG_ERROR(
        "Error in setting LP output val 0x%0x\n for QSFP:%d", device, port);
  }

  NP_MODULE_UNLOCK;
  return ret;
}

int debug_log_wr_en = 0;
static int bf_np_qsfp_gpio_write(uint8_t device,
                                 uint8_t device_offset,
                                 uint8_t len,
                                 uint16_t *buf,
                                 void *qsfp_msg) {
  qsfp_i2c_msg_t *msg = (qsfp_i2c_msg_t *)qsfp_msg;
  int delay;
  uint8_t mux_addr = 0, mux_channel = 0;
  bf_pltfm_status_t ret = BF_PLTFM_SUCCESS;
  uint16_t val = *buf;

  if ((!msg) || (!buf)) return -1;

  NP_MODULE_LOCK;
  delay = msg->delay;
  if (msg->mux_present) {
    mux_addr = msg->mux_addr;
    mux_channel = msg->mux_channel;
    uint8_t wr_buf[3] = {device_offset, (uint8_t)val, (uint8_t)(val >> 8)};
    uint8_t wr_len = len + 1;  // 3;
    ret = bf_fpga_i2c_write_mux(0,
                                NP_FPGA_QSFP_GPIO_I2CBUS_ADDR,
                                delay,
                                mux_addr,
                                mux_channel,
                                device,
                                wr_buf,
                                wr_len);
    if (ret != BF_PLTFM_SUCCESS) {
      LOG_ERROR(
          "Error in gpio write-op for mux:0x%0x, mux-ch:%d, device:0x%0x "
          "device-offset:0x%0x\n",
          mux_addr,
          mux_channel,
          device,
          device_offset);
    }
  }
  if (debug_log_wr_en) {
    LOG_ERROR(
        "GPIO WR op for mux:0x%0x, mux-ch:%d, device:0x%0x device-offset:0x%0x "
        "data:0x%0x\n",
        mux_addr,
        mux_channel,
        device,
        device_offset,
        val);
  }

  NP_MODULE_UNLOCK;
  return ret;
}

int debug_log_rd_en = 0;
static int bf_np_qsfp_gpio_read(uint8_t device,
                                uint8_t device_offset,
                                uint8_t len,
                                uint16_t *buf,
                                void *qsfp_msg) {
  qsfp_i2c_msg_t *msg = (qsfp_i2c_msg_t *)qsfp_msg;
  int delay;
  uint8_t mux_addr = 0, mux_channel = 0;
  bf_pltfm_status_t ret = BF_PLTFM_SUCCESS;
  uint8_t offset = device_offset;
  uint8_t offset_len = 1;

  if ((!msg) || (!buf)) return -1;

  NP_MODULE_LOCK;
  delay = msg->delay;
  if (msg->mux_present) {
    mux_addr = msg->mux_addr;
    mux_channel = msg->mux_channel;
    ret = bf_fpga_i2c_addr_read_mux(0,
                                    NP_FPGA_QSFP_GPIO_I2CBUS_ADDR,
                                    delay,
                                    mux_addr,
                                    mux_channel,
                                    device,
                                    &offset,
                                    (uint8_t *)buf,
                                    offset_len,
                                    len);
    if (ret != BF_PLTFM_SUCCESS) {
      LOG_ERROR(
          "Error in gpio read-op for mux:0x%0x, mux-ch:%d, device:0x%0x "
          "device-offset:0x%0x\n",
          mux_addr,
          mux_channel,
          device,
          device_offset);
    }
  }
  if (debug_log_rd_en) {
    LOG_ERROR(
        "GPIO RD op for mux:0x%0x, mux-ch:%d, device:0x%0x device-offset:0x%0x "
        "data_read:0x%0x\n",
        mux_addr,
        mux_channel,
        device,
        device_offset,
        *buf);
  }
  NP_MODULE_UNLOCK;
  return ret;
}

static int bf_np_qsfp_read_vector(unsigned int port,
                                  uint8_t bank,
                                  uint8_t page,
                                  int offset,
                                  int len,
                                  uint8_t *buf,
                                  uint32_t debug_flags,
                                  void *not_used) {
  int err = 0;
  int is_paged;

  if ((port > (unsigned int)max_qsfp_ports) || (!buf)) {
    LOG_ERROR("%s: Error Invalid QSFP: %u index or buf ptr\n", __func__, port);
    return -1;
  }

  is_paged = bf_qsfp_has_pages(port);
  if (page != 0 && (!is_paged)) {
    return -1;
  }

  if (offset >= (2 * MAX_QSFP_PAGE_SIZE)) {
    return -1;
  }

  NP_MODULE_LOCK;
  /* set the page */
  if (offset > (MAX_QSFP_PAGE_SIZE - 1) && is_paged) {
    if (bf_np_update_bank_page(port, bank, page) < 0) {
      err = -1;
      goto read_reg_end;
    }
  }

  /* read the register */
  if (bf_np_mod_read(port, offset, len, buf)) {
    err = -1;
    goto read_reg_end;
  }
read_reg_end:
  NP_MODULE_UNLOCK;
  return err;

  (void)not_used;
  (void)debug_flags;
}

static int bf_np_qsfp_write_vector(unsigned int port,
                                   uint8_t bank,
                                   uint8_t page,
                                   int offset,
                                   int len,
                                   uint8_t *buf,
                                   uint32_t debug_flags,
                                   void *not_used) {
  int err = 0;
  int is_paged;

  if ((port > (unsigned int)max_qsfp_ports) || (!buf)) {
    LOG_ERROR("%s: Error Invalid QSFP: %u index or buf ptr\n", __func__, port);
    return -1;
  }

  is_paged = bf_qsfp_has_pages(port);
  if (page != 0 && (!is_paged)) {
    return -1;
  }

  if (offset >= (2 * MAX_QSFP_PAGE_SIZE)) {
    return -1;
  }

  NP_MODULE_LOCK;

  if (offset > (MAX_QSFP_PAGE_SIZE - 1) && is_paged) {
    /* set the page */
    if (bf_np_update_bank_page(port, bank, page) < 0) {
      err = -1;
      goto wr_reg_end;
    }
  }

  /* read the register */
  if (bf_np_mod_write(port, offset, len, buf)) {
    err = -1;
    goto wr_reg_end;
  }

wr_reg_end:
  NP_MODULE_UNLOCK;
  return err;

  (void)not_used;
  (void)debug_flags;
}

/** platform qsfp subsystem initialization
 *
 *  @param arg
 *   arg __unused__
 *  @return
 *   0 on success and -1 on error
 */
int bf_pltfm_qsfp_init(void *arg) {
  pcal9535_info_t pca;
  bf_qsfp_vec_t vec;
  int num_ports = platform_num_ports_get();

  NP_MODULE_LOCK_INIT;
  if (bf_bd_is_this_port_internal(33, 0)) {
    num_ports = num_ports - 1;
  }

  bf_np_qsfp_set_max_qsfp_ports(num_ports);
  bf_qsfp_set_num(num_ports);
  vec.pltfm_read = bf_np_qsfp_read_vector;
  vec.pltfm_write = bf_np_qsfp_write_vector;
  bf_qsfp_vec_init(&vec);
  LOG_DEBUG("Number of QSFP ports:%d\n", max_qsfp_ports);

  // set-up platform vectors
  memset(&pca, 0, sizeof(pca));
  pca.pca_dev_id = PCAL9535;
  pca.read = bf_np_qsfp_gpio_read;
  pca.write = bf_np_qsfp_gpio_write;
  pca.debug = 0;
  if (pcal9535_platform_init(&pca)) {
    LOG_ERROR("QSFP platform init failed :%s\n", __func__);
    return -1;
  }

  int bus;
  for (bus = 0; bus < platform_num_ports_get(); bus++) {
    bf_fpga_i2c_set_clk(0, bus, 100000);
  }

  for (int curport = 1; curport <= num_ports; curport++) {
    bf_np_invalidate_bankpage(curport);
  }

  // configure qsfp GPIOs
  if (bf_np_pca9535_init()) {
    LOG_ERROR("QSFP GPIO init failed :%s\n", __func__);
    return -1;
  }

  (void)arg;
  return 0;
}

// Unsupported APIs
int bf_pltfm_qsfp_read_module(unsigned int port,
                              int offset,
                              int rd_len,
                              uint8_t *buf) {
  (void)port;
  (void)offset;
  (void)rd_len;
  (void)buf;

  LOG_ERROR("%s Unsupported API: %d\n", __func__, port);
  return -1;
}

// Read & Write are exposed through vectors
// Unsupported APIs
int bf_pltfm_qsfp_write_module(unsigned int port,
                               int offset,
                               int len,
                               uint8_t *buf) {
  (void)port;
  (void)offset;
  (void)len;
  (void)buf;
  return -1;
}

// Unsupported APIs
int bf_pltfm_qsfp_read_reg(unsigned int port,
                           uint8_t page,
                           int offset,
                           uint8_t *buf) {
  (void)port;
  (void)offset;
  (void)page;
  (void)buf;
  return -1;
}

// Unsupported APIs
int bf_pltfm_qsfp_write_reg(unsigned int port,
                            uint8_t page,
                            int offset,
                            uint8_t val) {
  (void)port;
  (void)offset;
  (void)page;
  (void)val;
  return -1;
}
