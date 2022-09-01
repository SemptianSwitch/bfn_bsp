#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>  //strlen
#include <errno.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>

bool dbg_print = false;
#define BF_FPGA_DEV_BASE_NAME "/dev/bf_fpga_"
#define BF_FPGA_BAR_SIZE(i) ((i == 0) ? (256 * 1024) : 0)

static char usage[] = {"bf_fpga_update <fpga_id> <binfile>"};

static uint8_t *fpga_base_addr = NULL;

/* FPGA update specific register offsets */
#define FPGA_UPDATE_CTRL 0x26000
#define FPGA_UPDATE_STATUS 0x26004
#define FPGA_FIFO_STATUS 0x26008
#define FPGA_FIFO_DATA 0x2600C
#define BF_FPGA_VER_REG 0x3F000
#define BF_FPGA_BUILD_DATE 0x3F004

/* FPGA update register macros */
#define FPGA_PROG_START (1 << 0)
#define FPGA_PROG_ID_CHCK_ONLY (1 << 1)
#define FPGA_PROG_VERIFY_ONLY (1 << 2)
#define FPGA_FIFO_RESET (1 << 3)

#define FPGA_PROG_DONE (1 << 0)
#define FPGA_PROG_ERR (1 << 1)
#define FPGA_ID_ERR (1 << 2)
#define FPGA_ERASE_ERR (1 << 3)
#define FPGA_PROG_INT_ERR (1 << 4)
#define FPGA_TOUT_ERR (1 << 5)
#define FPGA_CRC_ERR (1 << 6)
#define FPGA_PROG_STARTED (1 << 7)
#define FPGA_INIT_OK (1 << 8)
#define FPGA_ID_OK (1 << 9)
#define FPGA_ERASE_SW_OK (1 << 10)
#define FPGA_ERASE_OK (1 << 11)
#define FPGA_PROG_OK (1 << 12)
#define FPGA_VERIFY_OK (1 << 13)
#define FPGA_PROG_SW_OK (1 << 14)

#define FPGA_PROG_ERR_MASK (0x7E)
#define FPGA_PROG_OK_MASK (0x7F01)

#define FPGA_FIFO_EMPTY (1 << 0)
#define FPGA_FIFO_FULL (1 << 1)

static uint32_t swap_32(uint32_t num) {
  uint32_t swapped = ((num >> 24) & 0xff) |       // move byte 3 to byte 0
                     ((num << 8) & 0xff0000) |    // move byte 1 to byte 2
                     ((num >> 8) & 0xff00) |      // move byte 2 to byte 1
                     ((num << 24) & 0xff000000);  // byte 0 to byte 3
  return swapped;
}

static uint32_t fpga_read(uint32_t offset) {
  uint32_t val;
  val = *(volatile uint32_t *)(fpga_base_addr + offset);
  return val;
}

static void fpga_write(uint32_t offset, uint32_t val) {
  *(volatile uint32_t *)(fpga_base_addr + offset) = val;
}

static bool is_fifo_full() {
  uint32_t val = fpga_read(FPGA_FIFO_STATUS);

  if ((val & FPGA_FIFO_FULL) == 0) {
    return false;
  } else {
    return true;
  }
}

static bool is_fifo_empty() {
  uint32_t val = fpga_read(FPGA_FIFO_STATUS);

  if ((val & FPGA_FIFO_EMPTY) == 0) {
    return false;
  } else {
    return true;
  }
}

static bool is_spi_program_started() {
  uint32_t ctrl = fpga_read(FPGA_UPDATE_CTRL);

  if ((ctrl & FPGA_PROG_START) == 1) {
    return true;
  } else {
    return false;
  }
}

static int process_update(uint8_t *buf, size_t size) {
  int wait_count, cnt, status;
  uint32_t rd_sts, fifo_data;
  char user_input[8];

  if (is_spi_program_started()) {
    printf("spi programming is currently going on. Try later!\n");
    return -1;
  }
  /* start with fifo empty, wait for maximum 10 seconds */
  wait_count = 1000;
  status = 0;
  printf("checking for empty fifo ");
  while (!is_fifo_empty()) {
    if (wait_count-- <= 0) {
      status = -1;
      break;
    }
    fpga_write(FPGA_UPDATE_CTRL, FPGA_FIFO_RESET);
    if (wait_count % 100 == 0) {
      printf(".");
      fflush(stdout);
    }
    usleep(10000);
  }
  if (status != 0) {
    printf("\nError fifo not empty at initialization\n");
    return -1;
  }
  memset(user_input, 0, sizeof(user_input));
  wait_count = 4;
  while (user_input[0] != 'Y' && wait_count-- > 0) {
    printf("\nAre you sure to erase/program the fpga [Y/n] ? ");
    if (!fgets(user_input, sizeof(user_input), stdin)) {
      printf("exiting without updating\n");
      return -1;
    }
    if (user_input[0] == 'n' || user_input[0] == 'N') {
      printf("exiting without updating\n");
      return -1;
    }
  }
  if (wait_count <= 0) {
    printf("exiting without updating\n");
    return -1;
  }
  /* turn on SPI start control */
  printf("starting to update fpga.. ");
  fflush(stdout);
  fpga_write(FPGA_UPDATE_CTRL, FPGA_PROG_START);
  /* start writing data until the fifo is not full or timeout */
  cnt = (int)size;
  while (cnt > 0) {
    wait_count = 1000;
    status = 0;
    while (is_fifo_full()) {
      if (wait_count-- <= 0) {
        status = -1;
        break;
      }
      usleep(100000);
    }
    if (status != 0) {
      printf("\nError fifo not emptying during programming\n");
      goto update_end;
    }
    fifo_data = swap_32(*(uint32_t *)buf);
    fpga_write(FPGA_FIFO_DATA, fifo_data);
    if (cnt % (100 * 1024) == 0) {
      printf(".");
      fflush(stdout);
    }
    buf += 4;
    cnt -= 4;
  }
  /* check for programing status */
  printf("\nchecking the program status..\n");
  wait_count = 200;
  status = 0;
  rd_sts = 0;
  while (!(rd_sts & FPGA_PROG_DONE)) {
    usleep(100000);
    if (wait_count-- <= 0) {
      status = -1;
      break;
    }
    rd_sts = fpga_read(FPGA_UPDATE_STATUS);
  }
  if (status != 0) {
    printf("Error timeout during programming\n");
    goto update_end;
  }
  if (rd_sts & FPGA_PROG_ERR_MASK) {
    printf("Error in fpga_update status 0x%x\n", rd_sts);
    status = (int)rd_sts;
  } else {
    printf("fpga_update ok status 0x%x\n", rd_sts);
    status = 0;
  }

update_end:
  /* reset the fifo, anyway */
  fpga_write(FPGA_UPDATE_CTRL, FPGA_FIFO_RESET);
  /* turn off SPI start control */
  fpga_write(FPGA_UPDATE_CTRL, 0);
  return status;
}

void print_version(void) {
  char day, month, year, hr, min, sec;
  uint32_t build_ver, build_date;
  int f, sz;

  build_ver = fpga_read(BF_FPGA_VER_REG);
  build_date = fpga_read(BF_FPGA_BUILD_DATE);
  printf("fpga version %hu.%hu\n", (build_ver >> 16), build_ver & 0xFFFF);

  sec = (char)(build_date & 0x3f);
  build_date >>= 6;
  min = (char)(build_date & 0x3f);
  build_date >>= 6;
  hr = (char)(build_date & 0x1f);
  build_date >>= 5;
  year = (char)(build_date & 0x3f);
  build_date >>= 6;
  month = (char)(build_date & 0x0f);
  build_date >>= 4;
  day = (char)(build_date & 0x1f);
  printf("fpga build %02d/%02d/%2d %02d:%02d:%02d\n",
         month,
         day,
         year,
         hr,
         min,
         sec);
}

int main(int argc, char *argv[]) {
  char fname[32];
  int ret = 1, dev_fd, fpga_id;
  FILE *binfile;
  size_t f_size;
  uint8_t *f_buf;

  if (argc < 3) {
    printf("%s\n", usage);
    return 1;
  }
  fpga_id = atoi(argv[1]);

  snprintf(fname, sizeof(fname), "%s%d", BF_FPGA_DEV_BASE_NAME, fpga_id);
  dev_fd = open(fname, O_RDWR);
  if (dev_fd < 0) {
    printf("Error in opening %s\n", fname);
    return -1;
  }
  fpga_base_addr = mmap(
      NULL, BF_FPGA_BAR_SIZE(0), PROT_READ | PROT_WRITE, MAP_SHARED, dev_fd, 0);
  if (fpga_base_addr == (uint8_t *)-1) {
    printf("Error in mmapping %s\n", fname);
    close(dev_fd);
    return 1;
  }
  if (strcmp(argv[2], "-v") == 0) {
    print_version();
    close(dev_fd);
    return 0;
  }

  /* read the bin file */
  binfile = fopen(argv[2], "rb");
  if (!binfile) {
    printf("error opening %s\n", argv[2]);
    goto fun_end;
  }
  /* file the file size */
  fseek(binfile, 0, SEEK_END);
  f_size = ftell(binfile);
  fseek(binfile, 0, SEEK_SET);

  /* read the file */
  f_buf = malloc(f_size);
  if (!f_buf) {
    printf("error allocating memory\n");
    goto fun_end;
  }
  memset(f_buf, 0xff, f_size);  // default erased value
  if (fread(f_buf, 1, f_size, binfile) != f_size) {
    printf("error %d reading from %s\n", errno, argv[2]);
    goto fun_end;
  }
  ret = process_update(f_buf, f_size);
  if (ret == 0) {
    printf("fpga_update ok\n");
  }

fun_end:
  if (fpga_base_addr != (uint8_t *)-1) {
    munmap(fpga_base_addr, BF_FPGA_BAR_SIZE(0));
  }
  close(dev_fd);
  if (!f_buf) {
    free(f_buf);
  }
  if (binfile) {
    fclose(binfile);
  }
  return ret;
}
