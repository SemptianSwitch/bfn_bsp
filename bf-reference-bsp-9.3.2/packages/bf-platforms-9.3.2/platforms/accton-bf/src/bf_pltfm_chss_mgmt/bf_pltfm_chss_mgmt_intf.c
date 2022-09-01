#include <stdio.h>
#include "sys/types.h"
#include "ifaddrs.h"
#include <arpa/inet.h>
#include <dirent.h>
#include <string.h>
#include <bf_pltfm_chss_mgmt_intf.h>
#include <bf_bd_cfg/bf_bd_cfg_intf.h>
#include <bf_pltfm_cp2112_intf.h>
#include <bf_pltfm_syscpld.h>

// Local header files
#include "bf_pltfm_bd_eeprom.h"

/* APIs to derive the cdc-ethernet port name and its ipv6 link local address */
static char pltfm_cdc_ethernet_port_name[32] = {0};
static char pltfm_cdc_eth_server_ipv6_lladdr[128] = {0};

const char *bf_pltfm_bmc_server_addr_get(void) {
  return pltfm_cdc_eth_server_ipv6_lladdr;
}

const char *bf_pltfm_bmc_server_port_get(void) {
  return pltfm_cdc_ethernet_port_name;
}

#ifdef DEVICE_IS_ASIC
static int get_link_local_addr(char *if_name,
                               int if_name_length,
                               struct sockaddr_in6 *ip,
                               size_t size_ip) {
  struct ifaddrs *ifaddr, *ifa;
  int ret = -1;

  if (getifaddrs(&ifaddr) == -1) {
    perror("getifaddrs");
    ret = -1;
    freeifaddrs(ifaddr);
    return ret;
  }

  for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next) {
    if (!ifa->ifa_addr || ifa->ifa_addr->sa_family != AF_INET6) {
      continue;
    }

    if (strncmp(ifa->ifa_name, if_name, if_name_length)) {
      continue;
    }

    struct sockaddr_in6 *current_addr = (struct sockaddr_in6 *)ifa->ifa_addr;

    if (!IN6_IS_ADDR_LINKLOCAL(&(current_addr->sin6_addr))) {
      continue;
    }

    memcpy(ip, current_addr, size_ip);
    ret = 0;
  }

  freeifaddrs(ifaddr);
  return ret;
}

static int find_cdc_ether_device(char *path,
                                 char *intf_name,
                                 size_t intf_size) {
  struct dirent *dir;     /* for the directory entries */
  struct dirent *dir_net; /* for the directory entries at /net level */
  char d_path[255];
  DIR *d_net;
  DIR *d = opendir(path);
  int ret;

  *intf_name = '0'; /* initialize */
  if (d == NULL) {
    return -1;
  }
  while ((dir = readdir(d)) != NULL) {
    if (strcmp(dir->d_name, ".") != 0 && strcmp(dir->d_name, "..") != 0) {
      ret = snprintf(d_path, sizeof(d_path), "%s/%s/net", path, dir->d_name);
      if (ret < 0 || ret >= (int)sizeof(d_path)) {
        LOG_ERROR("error parsing cdc_ether directory name\n");
        closedir(d);
        return -1;
      }
      d_net = opendir(d_path);
      if (d_net == NULL) {
        continue;
      }
      while ((dir_net = readdir(d_net)) != NULL) {
        if (strcmp(dir_net->d_name, ".") != 0 &&
            strcmp(dir_net->d_name, "..") != 0) {
          /* found the cdc_ether device */
          memcpy(intf_name, dir_net->d_name, intf_size - 1);
          intf_name[intf_size - 1] = '\0';
          closedir(d);
          return 0;
        }
      }
    }
  }
  closedir(d);
  return -1;
}

static int cdc_ethernet_init(void) {
  char net_intf[32];
  struct sockaddr_in6 ip;
  char ipv6_str[64];
  size_t str_len;

  if (find_cdc_ether_device(
          "/sys/bus/usb/drivers/cdc_ether", net_intf, sizeof(net_intf))) {
    LOG_ERROR("Error unable to find cdc_ethernet port\n");
    return -1;
  } else {
    strncpy(pltfm_cdc_ethernet_port_name,
            net_intf,
            sizeof(pltfm_cdc_ethernet_port_name));
    pltfm_cdc_ethernet_port_name[sizeof(pltfm_cdc_ethernet_port_name) - 1] =
        '\0';
    LOG_DEBUG("cdc_ethernet port is %s\n", net_intf);
  }
  if (get_link_local_addr(net_intf, sizeof(net_intf), &ip, sizeof(ip))) {
    LOG_ERROR("Error getting ipv6 address for interface %s\n", net_intf);
    return -1;
  }
  if (inet_ntop(AF_INET6, &ip.sin6_addr, ipv6_str, sizeof(ipv6_str))) {
    str_len = strlen(ipv6_str);
    if (str_len > 0) {
      ipv6_str[str_len - 1] = '1'; /* force change the ip address */
      snprintf(pltfm_cdc_eth_server_ipv6_lladdr,
               sizeof(pltfm_cdc_eth_server_ipv6_lladdr),
               "%s%%25%s",
               ipv6_str,
               net_intf);
    } else {
      LOG_ERROR("Error bad ipv6 address for interface %s\n", ipv6_str);
      return -1;
    }
  } else {
    LOG_ERROR("Error bad ipv6 address for interface %s\n", net_intf);
    return -1;
  }
  return 0;
}
#endif /* DEVICE_IS_ASIC */

/* temporary API. functionality to move to syscpld hardware */
bf_pltfm_status_t bf_pltfm_test_core_set() {
  bf_pltfm_cp2112_device_ctx_t *hndl;
  bf_pltfm_board_id_t board_id;
  uint8_t buf[2];

  bf_pltfm_chss_mgmt_bd_type_get(&board_id);
  if ((board_id == BF_PLTFM_BD_ID_MAVERICKS_P0B) ||
      (board_id == BF_PLTFM_BD_ID_MAVERICKS_P0A) ||
      (board_id == BF_PLTFM_BD_ID_MAVERICKS_P0C)) {
    hndl = bf_pltfm_cp2112_get_handle(CP2112_ID_2);
  } else if ((board_id == BF_PLTFM_BD_ID_MONTARA_P0B) ||
             (board_id == BF_PLTFM_BD_ID_MONTARA_P0C) ||
             (board_id == BF_PLTFM_BD_ID_MONTARA_P0A)) {
    hndl = bf_pltfm_cp2112_get_handle(CP2112_ID_1);
  } else {
    return BF_PLTFM_COMM_FAILED;
  }
  buf[0] = 0x41;
  buf[1] = 0x00;
  return (bf_pltfm_cp2112_write(hndl, BF_MAV_SYSCPLD_I2C_ADDR, buf, 2, 100));
}

bf_pltfm_status_t bf_pltfm_chss_mgmt_init() {
  // Initialize all the sub modules
  bf_pltfm_status_t sts;

#ifdef DEVICE_IS_ASIC
  if (cdc_ethernet_init()) {
    printf("CHSS MGMT ERROR: Failed to configure cdc_eth ipv6\n");
    /* do not return error for now as the code has to intialize the board id */
  }
#endif

  sts = bf_pltfm_bd_type_init();
  if (sts != BF_PLTFM_SUCCESS) {
    printf("CHSS MGMT ERROR: Failed to initialize EEPROM library\n");
    return sts;
  }

  // Other initializations(Fan, etc.) go here

  return BF_PLTFM_SUCCESS;
}

bf_pltfm_status_t bf_pltfm_chss_mgmt_bd_type_get(
    bf_pltfm_board_id_t *board_id) {
  return bf_pltfm_bd_type_get(board_id);
}

bf_pltfm_status_t bf_pltfm_device_type_get(bf_dev_id_t dev_id,
                                           bool *is_sw_model) {
/* This func returns the device type based on compile time flags */
#ifdef DEVICE_IS_SW_MODEL
  *is_sw_model = true;
#else
  *is_sw_model = false;
#endif

  return BF_PLTFM_SUCCESS;
}
