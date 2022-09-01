/*******************************************************************************
 * Copyright (c) 2015-2020 Barefoot Networks, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * $Id: $
 *
 ******************************************************************************/

#include <stdio.h>
#include <string.h>

#include <bf_types/bf_types.h>
#include <bfsys/bf_sal/bf_sys_sem.h>
#include <bf_pltfm_types/bf_pltfm_types.h>
#include <bf_bd_cfg/bf_bd_cfg_intf.h>
#include <bf_pltfm_cp2112_intf.h>
#include <bf_qsfp/bf_qsfp.h>
#include <bf_pltfm_qsfp.h>
#include "bf_mav_qsfp_module.h"
#include <bf_mav_qsfp_i2c_lock.h>

#define DEFAULT_TIMEOUT_MS 200

typedef enum {
  /* The PCA9548 switch selecting for 32 port controllers
  * Note that the address is shifted one to the left to
  * work with the underlying I2C libraries.
  */
  ADDR_SWITCH_32 = 0xe0,
  ADDR_SWITCH_COMM = 0xe8
} mav_mux_pca9548_addr_t;

/* i2c addresses of PCA9535 */
static const uint8_t pca9535_i2c_addr[MAV_PCA9535_MAX + 1] = {
    (0x20 << 1),
    (0x21 << 1),
    (0x22 << 1),
    (0x23 << 1),
    (0x24 << 1),
    (0x25 << 1),
    (0x26 << 1) /* CPU QSFP port */
};

static unsigned char pca9548_bitmap[] = {
    0x2, 0x1, 0x8, 0x4, 0x20, 0x10, 0x80, 0x40,
};

/* qsfp module lock macros */
#define MAV_QSFP_LOCK bf_pltfm_i2c_lock()
#define MAV_QSFP_UNLOCK bf_pltfm_i2c_unlock()

/* return i2c address of a PCA9535 device */
static void bf_mav_pca9535_i2c_addr_get(int pca9535, uint8_t *i2c_addr) {
  if (pca9535 < MAV_PCA9535_MAX) {
    *i2c_addr = pca9535_i2c_addr[pca9535];
  } else {
    *i2c_addr = 0;
  }
}

static uint8_t bf_mav_get_pca9548_chn(int pca9535) {
  uint8_t chn;

  switch (pca9535) {
    case MAV_PCA9535_LP0:
      chn = 0;
      break;
    case MAV_PCA9535_LP1:
      chn = 1;
      break;
    case MAV_PCA9535_PR0:
      chn = 2;
      break;
    case MAV_PCA9535_PR1:
      chn = 3;
      break;
    case MAV_PCA9535_INT0:
      chn = 4;
      break;
    case MAV_PCA9535_INT1:
      chn = 5;
      break;
    case MAV_PCA9535_MISC:
      chn = 6;
      break;
    case MAV_PCA9548_CPU_PORT:
      chn = 7;
      break;
    default:
      chn = 0;
      break;
  }
  return chn;
}

/* write to a PCA9535 register
 * *** this function does not set the direction of the port in pca9535
 * *** must be called with i2c lock held
 */
static bf_pltfm_status_t bf_mav_pca9535_reg_set(
    bf_pltfm_cp2112_device_ctx_t *hndl,
    int pca9535,
    uint8_t reg_addr,
    uint16_t val) {
  uint8_t buf[3];
  uint8_t chn, i2c_addr;
  int rc;

  if (!hndl || pca9535 > MAV_PCA9535_MAX) {
    return BF_PLTFM_INVALID_ARG;
  }

  chn = bf_mav_get_pca9548_chn(pca9535);

  /* open up access to PCA9548 channel */
  rc = bf_pltfm_cp2112_write_byte(
      hndl, ADDR_SWITCH_COMM, (1 << chn), DEFAULT_TIMEOUT_MS);
  if (rc != BF_PLTFM_SUCCESS) {
    LOG_ERROR("Error in opening the common PCA9548 for pca9535 %d\n", pca9535);
    return -1;
  }

  bf_mav_pca9535_i2c_addr_get(pca9535, &i2c_addr);

  buf[0] = reg_addr;
  buf[1] = val;
  buf[2] = (val >> 8);

  rc = bf_pltfm_cp2112_write(hndl, i2c_addr, buf, 3, DEFAULT_TIMEOUT_MS);

  if (rc != BF_PLTFM_SUCCESS) {
    LOG_ERROR("Error in setting PCA9535 pca %d reg %d\n", pca9535, reg_addr);
    /* intentional  fall-thru (no return) to close common PCA 9548 */
  }
  /* close access to all PCA9535 */
  rc |=
      bf_pltfm_cp2112_write_byte(hndl, ADDR_SWITCH_COMM, 0, DEFAULT_TIMEOUT_MS);
  if (rc != BF_PLTFM_SUCCESS) {
    LOG_ERROR("Error in closing common PCA9548\n");
    return BF_PLTFM_COMM_FAILED;
  }

  return BF_PLTFM_SUCCESS;
}

/* read a PCA9535 register
 * *** this function does not check the direction of the port in pca9535
 * *** must be called with i2c lock held
 */
static bf_pltfm_status_t bf_mav_pca9535_reg_get(
    bf_pltfm_cp2112_device_ctx_t *hndl,
    int pca9535,
    uint8_t reg_addr,
    uint16_t *val) {
  uint8_t buf_out[1];
  uint8_t buf_in[2];
  uint8_t chn, i2c_addr;
  int rc;

  if (!hndl || pca9535 > MAV_PCA9535_MAX) {
    return BF_PLTFM_INVALID_ARG;
  }

  chn = bf_mav_get_pca9548_chn(pca9535);

  /* open up access to PCA9548 channel */
  rc = bf_pltfm_cp2112_write_byte(
      hndl, ADDR_SWITCH_COMM, (1 << chn), DEFAULT_TIMEOUT_MS);
  if (rc != BF_PLTFM_SUCCESS) {
    LOG_ERROR("Error in opening the common PCA9548 for pca9535 %d\n", pca9535);
    return -1;
  }

  bf_mav_pca9535_i2c_addr_get(pca9535, &i2c_addr);
  buf_out[0] = reg_addr;

  rc = bf_pltfm_cp2112_write_read_unsafe(
      hndl, i2c_addr, buf_out, buf_in, 1, 2, DEFAULT_TIMEOUT_MS);

  if (rc != BF_PLTFM_SUCCESS) {
    LOG_DEBUG("Error in reading PCA9535 %d reg %d\n", pca9535, reg_addr);
    /* intentional  fall-thru (no return) to close common PCA 9548 */
  } else {
    *val = buf_in[0] | (buf_in[1] << 8);
  }

  /* close access to all PCA9535 */
  rc |=
      bf_pltfm_cp2112_write_byte(hndl, ADDR_SWITCH_COMM, 0, DEFAULT_TIMEOUT_MS);
  if (rc != BF_PLTFM_SUCCESS) {
    LOG_DEBUG("Error in closing common PCA9548\n");
    return BF_PLTFM_COMM_FAILED;
  }

  return BF_PLTFM_SUCCESS;
}

/* intialize PCA 9548 and PCA 9535 on the qsfp subsystem i2c bus to
 * ensure a consistent state on i2c addreesed devices
 */
static int qsfp_init_sub_bus(bf_pltfm_cp2112_device_ctx_t *hndl) {
  int i;
  int rc;

  /* Initialize all PCA9548 devices to select channel 0 */
  for (i = 0; i < MAV_QSFP_PCA9548_CNT; i++) {
    rc = bf_pltfm_cp2112_write_byte(
        hndl, ADDR_SWITCH_32 + (i * 2), 0, DEFAULT_TIMEOUT_MS);
    if (rc != BF_PLTFM_SUCCESS) {
      LOG_ERROR("Error in initializing the PCA9548 devices itr %d\n", i);
      return -1;
    }
  }

  /* Enable all downstream channels on pca9548 for PCA9535
   * and disable channel 7 connected to CPU port QSFP and channel 6
   * connected to Tofino, board EEPROM and syncE chip.
   */
  /* disable all channels of common PCA9548 */
  rc =
      bf_pltfm_cp2112_write_byte(hndl, ADDR_SWITCH_COMM, 0, DEFAULT_TIMEOUT_MS);
  if (rc != BF_PLTFM_SUCCESS) {
    LOG_ERROR("Error in enabling common PCA9548 LP0 channel\n");
    return -1;
  }
  MAV_QSFP_LOCK;

  /* Set input direction for presence and init and disable polarity inversion */
  bf_mav_pca9535_reg_set(hndl, MAV_PCA9535_PR0, MAV_PCA9535_REG_POL, 0);
  bf_mav_pca9535_reg_set(hndl, MAV_PCA9535_PR0, MAV_PCA9535_REG_CFG, 0xffff);
  bf_mav_pca9535_reg_set(hndl, MAV_PCA9535_PR1, MAV_PCA9535_REG_POL, 0);
  bf_mav_pca9535_reg_set(hndl, MAV_PCA9535_PR1, MAV_PCA9535_REG_CFG, 0xffff);
  bf_mav_pca9535_reg_set(hndl, MAV_PCA9535_INT0, MAV_PCA9535_REG_POL, 0);
  bf_mav_pca9535_reg_set(hndl, MAV_PCA9535_INT0, MAV_PCA9535_REG_CFG, 0xffff);
  bf_mav_pca9535_reg_set(hndl, MAV_PCA9535_INT1, MAV_PCA9535_REG_POL, 0);
  bf_mav_pca9535_reg_set(hndl, MAV_PCA9535_INT1, MAV_PCA9535_REG_CFG, 0xffff);

  /* Set output direction for LP mode and disable LP mode */
  /* make these 9535 direction = output */
  bf_mav_pca9535_reg_set(hndl, MAV_PCA9535_LP0, MAV_PCA9535_REG_OUT, 0);
  bf_mav_pca9535_reg_set(hndl, MAV_PCA9535_LP0, MAV_PCA9535_REG_CFG, 0);

  /* configure PCA9535_LP1 */
  rc = bf_pltfm_cp2112_write_byte(
      hndl, ADDR_SWITCH_COMM, (1 << MAV_PCA9535_LP1), DEFAULT_TIMEOUT_MS);
  if (rc != BF_PLTFM_SUCCESS) {
    LOG_ERROR("Error in enabling common PCA9548 LP1 channel\n");
    MAV_QSFP_UNLOCK;
    return -1;
  }
  /* Set output direction for LP mode and disable LP mode */
  /* make these 9535 direction = output */
  bf_mav_pca9535_reg_set(hndl, MAV_PCA9535_LP1, MAV_PCA9535_REG_OUT, 0);
  bf_mav_pca9535_reg_set(hndl, MAV_PCA9535_LP1, MAV_PCA9535_REG_CFG, 0);

  /* configure cpu port pca 9535.
   * bit 8 : LP mode (o/p) : bit 9: presence (i/p)
   * bit 10 : intr (i/p) : bit 11 : reset (o/p)
   */
  if (cp2112_dev_id_get(hndl) == CP2112_ID_1) {
    bf_mav_pca9535_reg_set(hndl, MAV_PCA9535_MISC, MAV_PCA9535_REG_POL, 0);
    bf_mav_pca9535_reg_set(hndl, MAV_PCA9535_MISC, MAV_PCA9535_REG_OUT, 0x0800);
    bf_mav_pca9535_reg_set(hndl, MAV_PCA9535_MISC, MAV_PCA9535_REG_CFG, 0xF6FF);
  }

  /* Close all channels of common pca 9548 */
  rc =
      bf_pltfm_cp2112_write_byte(hndl, ADDR_SWITCH_COMM, 0, DEFAULT_TIMEOUT_MS);
  if (rc != BF_PLTFM_SUCCESS) {
    LOG_ERROR("Error in initializing the common PCA9548\n");
    MAV_QSFP_UNLOCK;
    return -1;
  }
  MAV_QSFP_UNLOCK;
  /* other PCA9535 are configured input type, by default, pres and int */
  return 0;
}

/* initialize various devices on qsfp cp2112 subsystem i2c bus */
int bf_pltfm_init_cp2112_qsfp_bus(bf_pltfm_cp2112_device_ctx_t *hndl1,
                                  bf_pltfm_cp2112_device_ctx_t *hndl2) {
  if (qsfp_init_sub_bus(hndl1)) {
    return -1;
  }
  if (hndl2 != NULL) {
    if (qsfp_init_sub_bus(hndl2)) {
      return -1;
    }
  }
  return 0;
}

/* disable the channel of PCA 9548  connected to the CPU QSFP */
static int unselect_cpu_qsfp(bf_pltfm_cp2112_device_ctx_t *hndl) {
  int rc;
  uint8_t i2c_addr;

  i2c_addr = ADDR_SWITCH_COMM;
  rc = bf_pltfm_cp2112_write_byte(hndl, i2c_addr, 0, DEFAULT_TIMEOUT_MS);
  if (rc != BF_PLTFM_SUCCESS) {
    LOG_ERROR("Error in %s\n", __func__);
    return -1;
  }
  return 0;
}

/* enable the channel of PCA 9548  connected to the CPU QSFP */
static int select_cpu_qsfp(bf_pltfm_cp2112_device_ctx_t *hndl) {
  int rc;
  uint8_t bit;
  uint8_t i2c_addr;

  bit = (1 << MAV_PCA9548_CPU_PORT);
  i2c_addr = ADDR_SWITCH_COMM;
  rc = bf_pltfm_cp2112_write_byte(hndl, i2c_addr, bit, DEFAULT_TIMEOUT_MS);
  if (rc != BF_PLTFM_SUCCESS) {
    LOG_ERROR("Error in %s\n", __func__);
    return -1;
  }
  return 0;
}

/* disable the channel of PCA 9548 connected to the QSFP (sub_port)  */
/* sub_port is zero based */
static int unselect_qsfp(bf_pltfm_cp2112_device_ctx_t *hndl,
                         unsigned int sub_port) {
  int rc;
  uint8_t i2c_addr;

  if (sub_port == BF_MAV_SUB_PORT_CNT) {
    return (unselect_cpu_qsfp(hndl));
  } else if (sub_port > BF_MAV_SUB_PORT_CNT) {
    return -1;
  }

  i2c_addr = ADDR_SWITCH_32 + ((sub_port / 8) << 1);
  rc = bf_pltfm_cp2112_write_byte(hndl, i2c_addr, 0, DEFAULT_TIMEOUT_MS);
  if (rc != BF_PLTFM_SUCCESS) {
    LOG_ERROR("Error in %s port %d\n", __func__, sub_port);
    return -1;
  }
  return 0;
}

/* enable the channel of PCA 9548 connected to the QSFP (sub_port)  */
/* sub_port is zero based */
static int select_qsfp(bf_pltfm_cp2112_device_ctx_t *hndl,
                       unsigned int sub_port) {
  int rc;
  uint8_t bit;
  uint8_t i2c_addr;

  if (sub_port == BF_MAV_SUB_PORT_CNT) {
    return (select_cpu_qsfp(hndl));
  } else if (sub_port > BF_MAV_SUB_PORT_CNT) {
    return -1;
  }

  bit = (uint8_t)pca9548_bitmap[sub_port % 8];
  i2c_addr = ADDR_SWITCH_32 + ((sub_port / 8) << 1);
  rc = bf_pltfm_cp2112_write_byte(hndl, i2c_addr, bit, DEFAULT_TIMEOUT_MS);
  if (rc != BF_PLTFM_SUCCESS) {
    LOG_ERROR("Error in %s port %d\n", __func__, sub_port);
    return -1;
  }
  return 0;
}

/* disable the channel of PCA 9548  connected to the CPU QSFP */
static int unselect_cpu_pca9535_chn(bf_pltfm_cp2112_device_ctx_t *hndl) {
  return (unselect_cpu_qsfp(hndl));
}

/* enable the channel of PCA 9548  connected to the CPU port PCA9535 */
static int select_cpu_pca9535_chn(bf_pltfm_cp2112_device_ctx_t *hndl) {
  int rc;
  uint8_t bit;
  uint8_t i2c_addr;

  bit = (1 << MAV_PCA9535_MISC);
  i2c_addr = ADDR_SWITCH_COMM;
  rc = bf_pltfm_cp2112_write_byte(hndl, i2c_addr, bit, DEFAULT_TIMEOUT_MS);
  if (rc != BF_PLTFM_SUCCESS) {
    LOG_ERROR("Error in %s\n", __func__);
    return -1;
  }
  return 0;
}

/* disable the channel of PCA 9548  connected to the CPU QSFP
 * and release i2c lock
 */
int bf_mav_unselect_unlock_misc_chn(bf_pltfm_cp2112_device_ctx_t *hndl) {
  int rc;

  rc = unselect_cpu_qsfp(hndl);
  MAV_QSFP_UNLOCK;
  return rc;
}

/* get the i2c lock and
 * enable the channel of PCA 9548  connected to the CPU port PCA9535
 */
int bf_mav_lock_select_misc_chn(bf_pltfm_cp2112_device_ctx_t *hndl) {
  int rc;
  uint8_t bit;
  uint8_t i2c_addr;

  MAV_QSFP_LOCK;
  bit = (1 << MAV_PCA9535_MISC);
  i2c_addr = ADDR_SWITCH_COMM;
  rc = bf_pltfm_cp2112_write_byte(hndl, i2c_addr, bit, DEFAULT_TIMEOUT_MS);
  if (rc != BF_PLTFM_SUCCESS) {
    MAV_QSFP_UNLOCK;
    LOG_ERROR("Error in %s\n", __func__);
    return -1;
  }
  return 0;
}

/* get the i2c lock and
 * disable the channel of PCA 9548  connected to the CPU port PCA9535
 */
int bf_mav_lock_unselect_misc_chn(bf_pltfm_cp2112_device_ctx_t *hndl) {
  int rc;
  uint8_t bit;
  uint8_t i2c_addr;

  MAV_QSFP_LOCK;
  bit = 0;
  i2c_addr = ADDR_SWITCH_COMM;
  rc = bf_pltfm_cp2112_write_byte(hndl, i2c_addr, bit, DEFAULT_TIMEOUT_MS);
  if (rc != BF_PLTFM_SUCCESS) {
    MAV_QSFP_UNLOCK;
    LOG_ERROR("Error in %s\n", __func__);
    return -1;
  }
  return 0;
}
/** read a qsfp module within a single cp2112 domain
 *
 *  @param module
 *   module  0 based ; cpu qsfp = 32
 *  @param i2c_addr
 *   i2c_addr of qsfp module (left bit shitef by 1)
 *  @param offset
 *   offset in qsfp memory
 *  @param len
 *   num of bytes to read
 *  @param buf
 *   buf to read into
 *  @return bf_pltfm_status_t
 *   BF_PLTFM_SUCCESS on success and other codes on error
 */
bf_pltfm_status_t bf_mav_qsfp_sub_module_read(
    bf_pltfm_cp2112_device_ctx_t *hndl,
    unsigned int module,
    uint8_t i2c_addr,
    unsigned int offset,
    int len,
    uint8_t *buf) {
  int rc;

  if (((offset + len) > (MAX_QSFP_PAGE_SIZE * 2)) ||
      module > BF_MAV_SUB_PORT_CNT) {
    return BF_PLTFM_INVALID_ARG;
  }

  MAV_QSFP_LOCK;

  if (select_qsfp(hndl, module)) {
    unselect_qsfp(hndl, module); /* unselect: selection might have happened */
    MAV_QSFP_UNLOCK;
    return BF_PLTFM_COMM_FAILED;
  }

  rc = bf_pltfm_cp2112_write_byte(hndl, i2c_addr, offset, DEFAULT_TIMEOUT_MS);
  if (rc != BF_PLTFM_SUCCESS) {
    goto i2c_error_end;
  }
  if (len > 128) {
    rc = bf_pltfm_cp2112_read(hndl, i2c_addr, buf, 128, DEFAULT_TIMEOUT_MS);
    if (rc != BF_PLTFM_SUCCESS) {
      goto i2c_error_end;
    }
    buf += 128;
    len -= 128;
    rc = bf_pltfm_cp2112_read(hndl, i2c_addr, buf, len, DEFAULT_TIMEOUT_MS);
    if (rc != BF_PLTFM_SUCCESS) {
      goto i2c_error_end;
    }

  } else {
    rc = bf_pltfm_cp2112_read(hndl, i2c_addr, buf, len, DEFAULT_TIMEOUT_MS);
    if (rc != BF_PLTFM_SUCCESS) {
      goto i2c_error_end;
    }
  }

  unselect_qsfp(hndl, module);
  MAV_QSFP_UNLOCK;
  return BF_PLTFM_SUCCESS;

i2c_error_end:
  /* temporary change to remove the clutter on console. This is
   * a genuine error is th emodule was present
   */
  LOG_DEBUG("Error in qsfp read port %d\n", module);
  unselect_qsfp(hndl, module);
  MAV_QSFP_UNLOCK;
  return BF_PLTFM_COMM_FAILED;
}

/** write a qsfp module within a single cp2112 domain
 *
 *  @param module
 *   module  0 based ; cpu qsfp = 32
 *  @param i2c_addr
 *   i2c_addr of qsfp module (left bit shitef by 1)
 *  @param offset
 *   offset in qsfp memory
 *  @param len
 *   num of bytes to write
 *  @param buf
 *   buf to write from
 *  @return bf_pltfm_status_t
 *   BF_PLTFM_SUCCESS on success and other codes on error
 */
bf_pltfm_status_t bf_mav_qsfp_sub_module_write(
    bf_pltfm_cp2112_device_ctx_t *hndl,
    unsigned int module,
    uint8_t i2c_addr,
    unsigned int offset,
    int len,
    uint8_t *buf) {
  int rc;
  uint8_t out_buf[61];

  /* CP2112 can write only 61 bytes at a time, and we use up one bit for the
   * offset */
  if ((len > 60) || ((offset + len) >= (MAX_QSFP_PAGE_SIZE * 2)) ||
      module > BF_MAV_SUB_PORT_CNT) {
    return BF_PLTFM_INVALID_ARG;
  }

  MAV_QSFP_LOCK;
  if (select_qsfp(hndl, module)) {
    unselect_qsfp(hndl, module); /* unselect: selection might have happened */
    MAV_QSFP_UNLOCK;
    return BF_PLTFM_COMM_FAILED;
  }

  out_buf[0] = offset;
  memcpy(out_buf + 1, buf, len);
  rc = bf_pltfm_cp2112_write(
      hndl, i2c_addr, out_buf, len + 1, DEFAULT_TIMEOUT_MS);
  if (rc != BF_PLTFM_SUCCESS) {
    LOG_ERROR("Error in qsfp write port %d\n", module);
    unselect_qsfp(hndl, module);
    MAV_QSFP_UNLOCK;
    return BF_PLTFM_COMM_FAILED;
  }

  unselect_qsfp(hndl, module);
  MAV_QSFP_UNLOCK;
  return BF_PLTFM_SUCCESS;
}

/* port bits position on PCA9535 register, both fo rpresence and interrupt */
/* they are alternate bit swaps */
static uint16_t pca9535_qsfp_bit_adjust(uint16_t val) {
  uint16_t temp1, temp2;

  temp1 = (val >> 1) & 0x5555;
  temp2 = (val << 1) & 0xaaaa;
  return (temp1 | temp2);
}

/* get the qsfp present mask status for the group of qsfps that are
 * within a single cp2112 domain
 * bit 0: module present, 1: not-present
 */
int bf_pltfm_get_sub_module_pres(bf_pltfm_cp2112_device_ctx_t *hndl,
                                 uint32_t *pres) {
  int rc;
  uint16_t val_l, val_h;

  val_l = val_h = 0xffff; /* initialize to absent */
  MAV_QSFP_LOCK;
  rc = bf_mav_pca9535_reg_get(
      hndl, MAV_PCA9535_PR0, MAV_PCA9535_REG_INP, &val_l);
  rc |= bf_mav_pca9535_reg_get(
      hndl, MAV_PCA9535_PR1, MAV_PCA9535_REG_INP, &val_h);
  MAV_QSFP_UNLOCK;
  if (rc == BF_PLTFM_SUCCESS) {
    val_l = pca9535_qsfp_bit_adjust(val_l);
    val_h = pca9535_qsfp_bit_adjust(val_h);
    *pres = (((uint32_t)val_h << 16) | (val_l & 0xffff));
    return 0;
  } else {
    return -1;
  }
}

/* get the qsfp interrupt status for the group of qsfps that are
 * within a single cp2112 domain
 * bit 0: module interrupt active, 1: interrupt not-active
 */
int bf_pltfm_get_sub_module_int(bf_pltfm_cp2112_device_ctx_t *hndl,
                                uint32_t *intr) {
  int rc;
  uint16_t val_l, val_h;

  val_l = val_h = 0xffff; /* initialize to absent */
  MAV_QSFP_LOCK;
  rc = bf_mav_pca9535_reg_get(
      hndl, MAV_PCA9535_INT0, MAV_PCA9535_REG_INP, &val_l);
  rc |= bf_mav_pca9535_reg_get(
      hndl, MAV_PCA9535_INT1, MAV_PCA9535_REG_INP, &val_h);
  MAV_QSFP_UNLOCK;
  if (rc == BF_PLTFM_SUCCESS) {
    val_l = pca9535_qsfp_bit_adjust(val_l);
    val_h = pca9535_qsfp_bit_adjust(val_h);
    *intr = (((uint32_t)val_h << 16) | (val_l & 0xffff));
    return 0;
  } else {
    *intr = 0xFFFFFFFFL; /* error situation, mark no interrupt */
    return -1;
  }
}

/* get the qsfp present mask status for the cpu port
 * bit 0: module present, 1: not-present
 */
int bf_pltfm_get_cpu_module_pres(bf_pltfm_cp2112_device_ctx_t *hndl,
                                 uint32_t *pres) {
  int rc;
  uint16_t val_l = 0xffff; /* init with absent */

  MAV_QSFP_LOCK;
  if (select_cpu_pca9535_chn(hndl)) {
    unselect_cpu_pca9535_chn(hndl); /* unselect anyway */
    MAV_QSFP_UNLOCK;
    return -1;
  }
  rc = bf_mav_pca9535_reg_get(
      hndl, MAV_PCA9535_MISC, MAV_PCA9535_REG_INP, &val_l);
  if (rc == BF_PLTFM_SUCCESS) {
    /* check only the pres bit */
    if (val_l & 0x200) {
      *pres = 0xFFFFFFFFL; /* mark  as not present */
    } else {
      *pres = 0xFFFFFFFEL; /* mark  as present */
    }
    rc = 0;
  } else {
    rc = -1;
  }
  unselect_cpu_pca9535_chn(hndl);
  MAV_QSFP_UNLOCK;
  return rc;
}

/* get the qsfp interrupt status for the cpu port
 * bit 0: module interrupt active, 1: interrupt not-active
 */
int bf_pltfm_get_cpu_module_int(bf_pltfm_cp2112_device_ctx_t *hndl,
                                uint32_t *intr) {
  int rc;
  uint16_t val_l = 0xffff; /* init with absent */
  ;

  MAV_QSFP_LOCK;
  if (select_cpu_pca9535_chn(hndl)) {
    unselect_cpu_pca9535_chn(hndl); /* unselect anyway */
    MAV_QSFP_UNLOCK;
    return -1;
  }
  rc = bf_mav_pca9535_reg_get(
      hndl, MAV_PCA9535_MISC, MAV_PCA9535_REG_INP, &val_l);
  if (rc == BF_PLTFM_SUCCESS) {
    /* check only the intr bit */
    if (val_l & 0x400) {
      *intr = 0xFFFFFFFFL; /* mark interrupt as not present */
    } else {
      *intr = 0xFFFFFFFEL; /* mark iinterrupt  as present */
    }
    rc = 0;
  } else {
    *intr = 0xFFFFFFFFL; /* error situation, mark no interrupt */
    rc = -1;
  }
  unselect_cpu_pca9535_chn(hndl);
  MAV_QSFP_UNLOCK;
  return rc;
}

/* get the qsfp lpmode status for the cpu port
 * bit 0: no-lpmode 1: lpmode
 */
int bf_pltfm_get_cpu_module_lpmode(bf_pltfm_cp2112_device_ctx_t *hndl,
                                   uint32_t *lpmode) {
  int rc;
  uint16_t val_l = 0; /* init */

  MAV_QSFP_LOCK;
  if (select_cpu_pca9535_chn(hndl)) {
    unselect_cpu_pca9535_chn(hndl); /* unselect anyway */
    MAV_QSFP_UNLOCK;
    return -1;
  }
  rc = bf_mav_pca9535_reg_get(
      hndl, MAV_PCA9535_MISC, MAV_PCA9535_REG_INP, &val_l);
  if (rc == BF_PLTFM_SUCCESS) {
    /* check only the lpmode bit */
    if (val_l & 0x100) {
      *lpmode = 0x1UL; /* lpmode set */
    } else {
      *lpmode = 0x0UL; /* no lpmode */
    }
    rc = 0;
  } else {
    *lpmode = 0x0UL; /* error situation, mark reset default */
    rc = -1;
  }
  unselect_cpu_pca9535_chn(hndl);
  MAV_QSFP_UNLOCK;
  return rc;
}

/* set the qsfp reset for the cpu port
 */
int bf_pltfm_set_cpu_module_reset(bf_pltfm_cp2112_device_ctx_t *hndl,
                                  bool reset) {
  int rc;
  uint16_t val_l = 0xF6FF; /* init */

  MAV_QSFP_LOCK;
  if (select_cpu_pca9535_chn(hndl)) {
    unselect_cpu_pca9535_chn(hndl); /* unselect anyway */
    MAV_QSFP_UNLOCK;
    return -1;
  }
  rc = bf_mav_pca9535_reg_get(
      hndl, MAV_PCA9535_MISC, MAV_PCA9535_REG_INP, &val_l);
  if (rc == BF_PLTFM_SUCCESS) {
    if (reset) {
      val_l &= 0xF7FF; /* drive reset active */
    } else {
      val_l |= 0x0800; /* drive reset inactive */
    }
    rc = bf_mav_pca9535_reg_set(
        hndl, MAV_PCA9535_MISC, MAV_PCA9535_REG_OUT, val_l);
  }
  unselect_cpu_pca9535_chn(hndl);
  MAV_QSFP_UNLOCK;
  return rc;
}

/* set the qsfp lpmode for the cpu port
 */
int bf_pltfm_set_cpu_module_lpmode(bf_pltfm_cp2112_device_ctx_t *hndl,
                                   bool lpmode) {
  int rc;
  uint16_t val_l = 0xF6FF; /* init */

  MAV_QSFP_LOCK;
  if (select_cpu_pca9535_chn(hndl)) {
    unselect_cpu_pca9535_chn(hndl); /* unselect anyway */
    MAV_QSFP_UNLOCK;
    return -1;
  }
  rc = bf_mav_pca9535_reg_get(
      hndl, MAV_PCA9535_MISC, MAV_PCA9535_REG_INP, &val_l);
  if (rc == BF_PLTFM_SUCCESS) {
    if (!lpmode) {
      val_l &= 0xFEFF; /* drive lpmode inactive */
    } else {
      val_l |= 0x0100; /* drive lpmode active */
    }
    rc = bf_mav_pca9535_reg_set(
        hndl, MAV_PCA9535_MISC, MAV_PCA9535_REG_OUT, val_l);
  }
  unselect_cpu_pca9535_chn(hndl);
  MAV_QSFP_UNLOCK;
  return rc;
}

/* get the qsfp lpmode as set by hardware pins
 */
int bf_pltfm_get_sub_module_lpmode(bf_pltfm_cp2112_device_ctx_t *hndl,
                                   uint32_t *lpmod) {
  int rc;
  uint16_t val_l, val_h;

  val_l = val_h = 0; /* initialize to absent */
  MAV_QSFP_LOCK;
  rc = bf_mav_pca9535_reg_get(
      hndl, MAV_PCA9535_LP0, MAV_PCA9535_REG_INP, &val_l);
  rc |= bf_mav_pca9535_reg_get(
      hndl, MAV_PCA9535_LP1, MAV_PCA9535_REG_INP, &val_h);
  MAV_QSFP_UNLOCK;
  if (rc == BF_PLTFM_SUCCESS) {
    val_l = pca9535_qsfp_bit_adjust(val_l);
    val_h = pca9535_qsfp_bit_adjust(val_h);
    *lpmod = (((uint32_t)val_h << 16) | (val_l & 0xffff));
    return 0;
  } else {
    *lpmod = 0; /* error situation, mark reset default values*/
    return -1;
  }
}

/* set the qsfp lpmode as set by hardware pins
 */
int bf_pltfm_set_sub_module_lpmode(bf_pltfm_cp2112_device_ctx_t *hndl,
                                   unsigned int module,
                                   bool lpmod) {
  int rc, pca9535;
  uint16_t val, mask;

  val = 0;
  if (module < 16) {
    pca9535 = MAV_PCA9535_LP0;
    mask = (1 << module);
  } else if (module < 32) {
    pca9535 = MAV_PCA9535_LP1;
    mask = (1 << (module - 16));
  } else {
    return -1;
  }
  MAV_QSFP_LOCK;
  rc = bf_mav_pca9535_reg_get(hndl, pca9535, MAV_PCA9535_REG_INP, &val);
  if (rc != BF_PLTFM_SUCCESS) {
    MAV_QSFP_UNLOCK;
    return -1;
  }
  val = pca9535_qsfp_bit_adjust(val);
  if (lpmod) {
    if (val & mask) {
      MAV_QSFP_UNLOCK;
      return 0; /* nothing to be done */
    } else {
      val |= mask;
    }
  } else {
    if (val & mask) {
      val &= ~mask;
    } else {
      MAV_QSFP_UNLOCK;
      return 0; /* nothing to be done */
    }
  }
  val = pca9535_qsfp_bit_adjust(val);
  rc = bf_mav_pca9535_reg_set(hndl, pca9535, MAV_PCA9535_REG_OUT, val);
  MAV_QSFP_UNLOCK;
  if (rc != BF_PLTFM_SUCCESS) {
    return -1;
  }
  return 0;
}
