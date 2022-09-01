/*******************************************************************************
 * Copyright (c) 2015-2020 Barefoot Networks, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * $Id: $
 *
 ******************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <bf_pltfm_types/bf_pltfm_types.h>
#include "tofino_spi_if.h"
#include "tofino_porting_spi.h"

#if 0 /* turn this on for target platform and populate the functions functions \
       * appropriately                                                         \
       */
/* Tofino SPI Init function  */
/* @brief bf_porting_spi_init
 *   perform any subsystem specific initialization necessary to
 *   access the ASIC thru i2c or pci is done here
 * @param dev_id
 *   ASIC id (to which SPI eeprom is connected)
 *   0: on success , -1: failure
 */
int tofino_porting_spi_init(int dev_id) {
  (void)dev_id;
  return 0;
}

int tofino_porting_spi_finish(int dev_id) {
  (void)dev_id;
  return 0;
}

int tofino_porting_spi_reg_wr(int dev_id, uint32_t reg, uint32_t data) {}

int tofino_porting_spi_reg_rd(int dev_id, uint32_t reg, uint32_t *data) {}

#else /***** For barefoot newport platforms ********/

#include <unistd.h>
#include <bf_types/bf_types.h>
#include <dvm/bf_drv_intf.h>
#include <bf_pltfm_fpga.h>

#define I2C_ADDR_TOFINO (0x58)
#define I2C_ADDR_PCA9548 0x74
#define I2C_MUX_CHN_TOFINO 0x40
#define I2C_BUS_TOFINO 32
typedef enum {
  TOFINO_I2c_BAR0 = 0,    /* regular PCIe space */
  TOFINO_I2c_BAR_MSI = 1, /* msix capability register space */
  TOFINO_I2c_BAR_CFG = 2, /* 4k pcie cap + 4K config space */
} Tofino_I2C_BAR_SEG;

typedef enum {
  TOFINO_I2C_OPCODE_LOCAL_WR = (0 << 5),
  TOFINO_I2C_OPCODE_LOCAL_RD = (1 << 5),
  TOFINO_I2C_OPCODE_IMM_WR = (2 << 5),
  TOFINO_I2C_OPCODE_IMM_RD = (3 << 5),
  TOFINO_I2C_OPCODE_DIR_WR = (4 << 5),
  TOFINO_I2C_OPCODE_DIR_RD = (5 << 5),
  TOFINO_I2C_OPCODE_INDIR_WR = (6 << 5),
  TOFINO_I2C_OPCODE_INDIR_RD = (7 << 5)
} TOFINO_I2C_CMD_OPCODE;

#define TOFINO_I2C_CMD_OPCODE_MASK 0xE0
#define TOFINO_I2C_BAR_SEG_SHFT 28

static int raw_read(int fd,
                    uint8_t address,
                    uint8_t *offset,
                    uint8_t size_offset,
                    uint8_t *buf,
                    uint8_t len) {
  int ret;

  ret = bf_fpga_i2c_write_mux(fd,
                              I2C_BUS_TOFINO,
                              0,
                              I2C_ADDR_PCA9548,
                              I2C_MUX_CHN_TOFINO,
                              address,
                              offset,
                              size_offset);
  if (ret) {
    return -1;
  }
  ret = bf_fpga_i2c_read_mux(fd,
                             I2C_BUS_TOFINO,
                             0,
                             I2C_ADDR_PCA9548,
                             I2C_MUX_CHN_TOFINO,
                             address,
                             buf,
                             len);
  return ret;
}

static int raw_write(int fd, uint8_t address, uint8_t *buf, uint8_t len) {
  int ret;

  ret = bf_fpga_i2c_write_mux(fd,
                              I2C_BUS_TOFINO,
                              0,
                              I2C_ADDR_PCA9548,
                              I2C_MUX_CHN_TOFINO,
                              address,
                              buf,
                              len);
  return ret;
}
/** Tofino slave mode i2c read with "direct read" command
 *
 *  @param chip_id
 *    chip_id
 *  @param fd
 *    fpga_id
 *  @param seg
 *    BAR segment (0: regular PCIe space, 1: MSIX cap space,
 *                 2: 4K pcie cap/cfg space)
 *  @param dir_reg
 *     direct address space register to access
 *  @param data
 *    data to read from dir_addr
 *  @return
 *    0 on success and any other value on error
 */
int bf_tofino_dir_i2c_rd(int chip_id,
                         int fd,
                         Tofino_I2C_BAR_SEG seg,
                         uint32_t dir_addr,
                         uint32_t *data) {
  uint8_t out[7];

  out[0] = TOFINO_I2C_OPCODE_DIR_RD;
  dir_addr |= (seg << TOFINO_I2C_BAR_SEG_SHFT);
  /* TBD : check endianness with tofino specs */
  memcpy(&out[1], &dir_addr, 4);
  return (raw_read(fd, I2C_ADDR_TOFINO, &out[0], 5, (uint8_t *)data, 4));
}

/** Tofino slave mode i2c write with "direct write" command
 *
 *  @param chip_id
 *    chip_id
 *  @param fd
 *    fpga_id
 *  @param seg
 *    BAR segment (0: regular PCIe space, 1: MSIX cap space,
 *                 2: 4K pcie cap/cfg space)
 *  @param dir_reg
 *     direct address space register to access
 *  @param data
 *    data to write to dir_addr
 *  @return
 *    0 on success and any other value on error
 */
int bf_tofino_dir_i2c_wr(int chip_id,
                         int fd,
                         Tofino_I2C_BAR_SEG seg,
                         uint32_t dir_addr,
                         uint32_t data) {
  uint8_t out[16];

  out[0] = TOFINO_I2C_OPCODE_DIR_WR;
  dir_addr |= (seg << TOFINO_I2C_BAR_SEG_SHFT);
  /* TBD : check endianness with tofino specs */
  memcpy(&out[1], &dir_addr, 4);
  memcpy(&out[5], &data, 4);

  return (raw_write(fd, I2C_ADDR_TOFINO, &out[0], 9));
}

int tofino_porting_spi_init(int dev_id, void *arg) {
  uint8_t board_id = (uint8_t)strtol((char *)arg, NULL, 0);
  if (board_id != 0) {
    TOFINO_PORTING_LOG_ERR("Error: invalid platform\n");
    return -1;
  }
  /* init fpga library module */
  if (bf_pltfm_fpga_init((void *)0UL)) {
    TOFINO_PORTING_LOG_ERR("Error: fpga initialization\n");
    return -1;
  }
  return 0;
}

int tofino_porting_spi_finish(int dev_id) {
  (void)dev_id;
  bf_pltfm_fpga_deinit((void *)0UL);
  return 0;
}

int tofino_porting_spi_reg_wr(int dev_id, uint32_t reg, uint32_t data) {
  return (bf_tofino_dir_i2c_wr(dev_id, 0, 0, reg, data));
}

int tofino_porting_spi_reg_rd(int dev_id, uint32_t reg, uint32_t *data) {
  return (bf_tofino_dir_i2c_rd(dev_id, 0, 0, reg, data));
}

/* stub functions to make tofino_spi_util build */
int platform_num_ports_get(void) { return 0; }

struct pltfm_bd_map_t *platform_pltfm_bd_map_get(int *rows) {
  if (rows) {
    *rows = 0;
  }
  return NULL;
}

#endif
