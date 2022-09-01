#include <sched.h>
#include <linux/tcp.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>  //strlen
#include <stddef.h>
#include <pthread.h>
#include <inttypes.h>  //strlen
#include <sys/socket.h>
#include <arpa/inet.h>  //inet_addr
#include <unistd.h>     //write
#include <errno.h>
#include <bfsys/bf_sal/bf_sys_intf.h>
#include <bf_pltfm_types/bf_pltfm_types.h>
#include "diag_server.h"
#include "diag_handler.h"
#include "bf_pltfm_diag_ver.h"

/* global */
newport_diag_info_t newport_diag_info;
extern void newport_diag_register_ucli_node();

const char *bf_pltfm_diag_get_version() { return BF_PLTFM_DIAG_VER; }
extern void bf_pltfm_temperature_monitor_enable(bool enable);

const char *bf_pltfm_diag_get_internal_version() {
  return BF_PLTFM_DIAG_INTERNAL_VER;
}

/* Create diag server docket */
bf_status_t newport_diag_create_server(int listen_port) {
  struct sockaddr_in server;
  int server_sock, so_reuseaddr = 1;

  // Create socket
  server_sock = socket(AF_INET, SOCK_STREAM, 0);
  if (server_sock == -1) {
    NEWPORT_DIAG_PRINT("Could not create socket");
    return BF_NO_SYS_RESOURCES;
  }
  NEWPORT_DIAG_PRINT("Diag: Listen socket created \n");

  // Prepare the sockaddr_in structure
  memset((char *)&server, 0, sizeof(server));
  server.sin_family = AF_INET;
  server.sin_addr.s_addr = INADDR_ANY;
  server.sin_port = htons(listen_port);

  setsockopt(server_sock,
             SOL_SOCKET,
             SO_REUSEADDR,
             &so_reuseaddr,
             sizeof so_reuseaddr);

  // Bind
  if (bind(server_sock, (struct sockaddr *)&server, sizeof(server)) < 0) {
    // print the error message
    perror("bind failed. Error");
    return BF_INVALID_ARG;
  }
  NEWPORT_DIAG_PRINT("Diag: bind done on port %d. Listening now..\n",
                     listen_port);
  // Listen
  if (listen(server_sock, 1) == -1) {
    perror("Listen failed: Error");
    return BF_INVALID_ARG;
  }

  // Accept an incoming connection
  puts("Diag: Waiting for incoming connections...");
  return server_sock;
}

/* Accept client connection */
int get_newport_diags_client_conn(int server_sock) {
  struct sockaddr_in client;
  int client_sock, sock_size = 0;

  memset(&client, 0, sizeof(client));
  sock_size = sizeof(client);
  // accept connection from an incoming client
  client_sock =
      accept(server_sock, (struct sockaddr *)&client, (socklen_t *)&sock_size);
  if (client_sock < 0) {
    perror("accept failed");
    return BF_NO_SYS_RESOURCES;
  }
  // NEWPORT_DIAG_PRINT("Diag Client Connection accepted\n");

  return client_sock;
}

/* Read from diag socket */
static bf_status_t newport_diag_server_read_from_socket(int sock,
                                                        char *buf,
                                                        int len) {
  int n_read = 0, n_read_this_time = 0, i = 1;

  // Receive a message from client
  do {
    n_read_this_time =
        recv(sock, (buf + n_read), (len - n_read), 0 /*MSG_WAITALL*/);
    if (n_read_this_time > 0) {
      n_read += n_read_this_time;
      if (n_read < len) {
        NEWPORT_DIAG_PRINT("Partial recv: %d of %d so far..\n", n_read, len);
      }
    }
    setsockopt(sock, IPPROTO_TCP, TCP_QUICKACK, (void *)&i, sizeof(i));

  } while (n_read < len);
  return n_read;
}

/* Write to socket */
static void newport_diag_write_to_socket(int sock, char *buf, int len) {
  if (send(sock, buf, len, 0) < 0) {
    puts("Send failed");
    exit(3);
  }
}

/* Send back response to client socket   */
static bf_status_t newport_diag_send_client_resp(uint8_t exit_code, char *dat) {
  char resp_str[NEWPORT_DIAG_CLIENT_CMD_MAX_SIZE];
  int size = strlen(NEWPORT_DIAG_CMD_OUT_FILENAME) + 1;

  newport_diag_info.resp_filep = fopen(NEWPORT_DIAG_CMD_OUT_FILENAME, "w+");
  if (!newport_diag_info.resp_filep) {
    NEWPORT_DIAG_PRINT("error opening diag result file %s\n",
                       NEWPORT_DIAG_CMD_OUT_FILENAME);
    return BF_NO_SYS_RESOURCES;
  }

  /* write output to a file */
  int ret = truncate(NEWPORT_DIAG_CMD_OUT_FILENAME, 0);
  if (ret != 0) {
    puts("Truncate failed");
  }

  resp_str[0] = exit_code;
  if ((dat) && (strlen(dat) > 0)) {
    fseek(newport_diag_info.resp_filep, 0, SEEK_SET);
    fprintf(newport_diag_info.resp_filep, "%s\n", dat);
    fflush(newport_diag_info.resp_filep);
  }
  /* send file name to client */
  strncpy(resp_str + 1, NEWPORT_DIAG_CMD_OUT_FILENAME, size);
  resp_str[size + 1] = 0; /* force null terminate */
  newport_diag_write_to_socket(
      newport_diag_info.client_sock, resp_str, size + 1);
  fclose(newport_diag_info.resp_filep);

  /* Zero out the diag response string */
  newport_diag_info.file_resp_str[0] = '\0';
  return 0;
}

/* Parse key=value string (value is integer) and return value */
bf_status_t newport_diag_parse_key_value_int(char *string) {
  const char delimiters[] = "=";
  char *key, *value;
  int ret_value = -1;

  key = strtok(string, delimiters);
  (void)key;
  value = strtok(NULL, delimiters);

  if (value != NULL) {
    ret_value = atoi(value);
  }

  return ret_value;
}

/* Parse key=value string (value is string) and return value */
char *newport_diag_parse_key_value_string(char *string,
                                          char *buffer,
                                          int buf_len) {
  const char delimiters[] = "=";
  char *key, *value;

  buffer[0] = '\0';
  key = strtok(string, delimiters);
  (void)key;
  value = strtok(NULL, delimiters);

  if ((value != NULL) && ((int)strlen(value) < buf_len)) {
    strncpy(buffer, value, strlen(value));
  }

  return buffer;
}

/* Parse port list in this format: 1-4,6,11-13,14  */
void newport_diag_parse_port_list(char *ports_bm,
                                  int *port_arr,
                                  int *num_ports) {
  int j = 0, first_port = 0, last_port = 0, temp_port = 0;
  char first_port_str[200];
  const char delimiters_ports[] = ",";
  char *ports = NULL, *range_sep;
  int port_arr_cntr = 0;

  if (ports_bm != NULL) {
    ports = strtok(ports_bm, delimiters_ports);
  }

  while (ports != NULL) {
    /* Start of range */
    range_sep = strchr(ports, '-');
    if (range_sep != NULL) {
      memset(first_port_str, 0, sizeof(first_port_str));
      // NEWPORT_DIAG_PRINT("Range sep at %ld \n", (range_sep - ports));
      strncpy(first_port_str, ports, (range_sep - ports));
      // NEWPORT_DIAG_PRINT("first_port_str %s \n", first_port_str);
      first_port = atoi(first_port_str);
      last_port = atoi(range_sep + 1);
      /* Put ports from range */
      for (j = first_port; j <= last_port; j++) {
        if (j < BF_DIAG_MAX_PORTS) {
          port_arr[port_arr_cntr] = j;
          port_arr_cntr = port_arr_cntr + 1;
        }
      }
    } else {
      temp_port = atoi(ports);
      if (temp_port < BF_DIAG_MAX_PORTS) {
        port_arr[port_arr_cntr] = temp_port;
        port_arr_cntr = port_arr_cntr + 1;
      }
    }
    ports = strtok(NULL, delimiters_ports);
  }
  *num_ports = port_arr_cntr;
}

/* Parse port list in this format: ubm=1-4,6,11-13,14  */
void newport_diag_parse_port_list_string(char *port_string,
                                         int *port_arr,
                                         int *num_ports) {
  const char delimiters_ubm_pbm[] = "=";
  char *ports_bm;

  ports_bm = strtok(port_string, delimiters_ubm_pbm);
  // NEWPORT_DIAG_PRINT("Ports_bm %s \n", ports_bm);
  /* Get the port string */
  ports_bm = strtok(NULL, delimiters_ubm_pbm);
  newport_diag_parse_port_list(ports_bm, port_arr, num_ports);
}

/* Send invalid command to client */
static void newport_diag_process_invalid_cmd() {
  newport_diag_send_client_resp(1, "invalid cmd");
}

/* Send diag command response */
static void newport_diag_send_cmd_response(int ret) {
  if (ret == 0) {
    newport_diag_send_client_resp(ret, "Success");
  } else {
    newport_diag_send_client_resp(ret, "Failed");
  }
}

/* Process linespeed command  */
static void newport_diag_process_linespeed_cmd(
    char subtokens[][NEWPORT_DIAG_SUBTOKEN_STR_SIZE], int num_subtokens) {
  int ret = 0;
  if (num_subtokens < 1) {
    newport_diag_process_invalid_cmd();
    return;
  }
  if (!strncmp(subtokens[0], "init", strlen("init"))) {
    if (num_subtokens != 4) {
      newport_diag_process_invalid_cmd();
      return;
    }
    int start_port = atoi(subtokens[1]);
    int size = atoi(subtokens[2]);
    char *type = subtokens[3];
    /* Call the handler */
    ret = newport_diag_service_linespeed_init(
        NEWPORT_DIAG_DEF_DEV_ID, start_port, size, type);
    newport_diag_send_cmd_response(ret);
  } else if (!strncmp(subtokens[0], "run", strlen("run"))) {
    if (num_subtokens != 4) {
      newport_diag_process_invalid_cmd();
      return;
    }
    int time = atoi(subtokens[1]);
    int pkt_size = atoi(subtokens[2]);
    int num_packet = atoi(subtokens[3]);
    /* Call the handler */
    ret = newport_diag_service_linespeed_run(time, pkt_size, num_packet);
    newport_diag_send_cmd_response(ret);
  } else if (!strncmp(subtokens[0], "stop", strlen("stop"))) {
    /* Call the handler */
    ret = newport_diag_service_linespeed_stop();
    newport_diag_send_cmd_response(ret);
  } else if (!strncmp(subtokens[0], "end", strlen("end"))) {
    /* Call the handler */
    ret = newport_diag_service_linespeed_end();
    newport_diag_send_cmd_response(ret);
  } else if (!strncmp(subtokens[0], "show", strlen("show"))) {
    /* Call the handler */
    ret = newport_diag_service_linespeed_show(newport_diag_info.file_resp_str,
                                              NEWPORT_DIAG_FILE_RESP_MAX_SIZE);
    newport_diag_send_client_resp(ret, newport_diag_info.file_resp_str);
  } else {
    newport_diag_send_client_resp(1, "bad cmd param");
  }
}

/* Process port-status command  */
static void newport_diag_process_ps_cmd(
    char subtokens[][NEWPORT_DIAG_SUBTOKEN_STR_SIZE], int num_subtokens) {
  char *port_name = NULL;
  diag_front_port_t front_port = -1;
  if (num_subtokens >= 1) {
    port_name = subtokens[0];
    front_port = atoi(port_name);
  }
  /* Call the handler */
  newport_diag_service_port_status_show(NEWPORT_DIAG_DEF_DEV_ID,
                                        front_port,
                                        newport_diag_info.file_resp_str,
                                        NEWPORT_DIAG_FILE_RESP_MAX_SIZE);
  newport_diag_send_client_resp(0, newport_diag_info.file_resp_str);
}

/* Process version command  */
static void newport_diag_process_version_cmd() {
  snprintf(newport_diag_info.file_resp_str,
           NEWPORT_DIAG_FILE_RESP_MAX_SIZE - 1,
           "%s",
           bf_pltfm_diag_get_version());
  newport_diag_send_client_resp(0, newport_diag_info.file_resp_str);
}

/* Process phy command  */
static void newport_diag_process_phy_cmd(
    char subtokens[][NEWPORT_DIAG_SUBTOKEN_STR_SIZE], int num_subtokens) {
  char *port_name;
  int ret = 0;
  const char delim[2] = "=";

  if (num_subtokens < 1) {
    newport_diag_process_invalid_cmd();
    return;
  }
  if (!strncmp(subtokens[0], "info", strlen("info"))) {
    newport_diag_service_phy_info_show(NEWPORT_DIAG_DEF_DEV_ID,
                                       newport_diag_info.file_resp_str,
                                       NEWPORT_DIAG_FILE_RESP_MAX_SIZE);
    newport_diag_send_client_resp(0, newport_diag_info.file_resp_str);
  } else if (!strncmp(subtokens[0], "diag", strlen("diag"))) {
    if (num_subtokens < 3) {
      newport_diag_process_invalid_cmd();
      return;
    }
    port_name = subtokens[1];
    if (!strncmp(subtokens[2], "eyescan", strlen("eyescan"))) {
      /* Call handler */
      /* Assume port name is numeric for now */
      newport_diag_service_eyescan(NEWPORT_DIAG_DEF_DEV_ID,
                                   atoi(port_name),
                                   newport_diag_info.file_resp_str,
                                   NEWPORT_DIAG_FILE_RESP_MAX_SIZE);
      newport_diag_send_client_resp(0, newport_diag_info.file_resp_str);
    } else if (!strncmp(subtokens[2], "prbs", strlen("prbs"))) {
      if (num_subtokens < 4) {
        newport_diag_process_invalid_cmd();
        return;
      }
      if (!strncmp(subtokens[3], "set", strlen("set"))) {
        uint32_t p_value = 0;
        if (num_subtokens < 5) {
          newport_diag_process_invalid_cmd();
          return;
        }
        if (!strncmp(subtokens[4], "p", strlen("p"))) {
          char *value_str = strtok(subtokens[4], delim);
          value_str = strtok(NULL, delim);
          if (!value_str) {
            newport_diag_process_invalid_cmd();
            return;
          }
          p_value = atoi(value_str);
        } else {
          newport_diag_process_invalid_cmd();
          return;
        }
        ret = newport_diag_service_prbs_set(
            NEWPORT_DIAG_DEF_DEV_ID, atoi(port_name), p_value);
        newport_diag_send_cmd_response(ret);
      } else if (!strncmp(subtokens[3], "clean", strlen("clean"))) {
        ret = newport_diag_service_prbs_clean(NEWPORT_DIAG_DEF_DEV_ID,
                                              atoi(port_name));
        newport_diag_send_cmd_response(ret);
      } else if (!strncmp(subtokens[3], "show", strlen("show"))) {
        newport_diag_service_prbs_show(NEWPORT_DIAG_DEF_DEV_ID,
                                       atoi(port_name),
                                       newport_diag_info.file_resp_str,
                                       NEWPORT_DIAG_FILE_RESP_MAX_SIZE);
        newport_diag_send_client_resp(0, newport_diag_info.file_resp_str);
      } else {
        newport_diag_process_invalid_cmd();
        return;
      }
    } else {
      newport_diag_process_invalid_cmd();
      return;
    }
  } else if (!strncmp(subtokens[0], "ber", strlen("ber"))) {
    if (num_subtokens < 2) {
      newport_diag_process_invalid_cmd();
      return;
    }
    int port = atoi(subtokens[1]);
    newport_diag_service_ber(NEWPORT_DIAG_DEF_DEV_ID,
                             port,
                             newport_diag_info.file_resp_str,
                             NEWPORT_DIAG_FILE_RESP_MAX_SIZE);
    newport_diag_send_client_resp(0, newport_diag_info.file_resp_str);
  } else {
    /* Port number has been specified */
    bool valid_cmd = false;
    bool tx_pre2 = false, tx_pre1 = false;
    bool tx_main = false, tx_post1 = false, tx_post2 = false;
    int emphasis_value = 0;
    if (num_subtokens < 3) {
      newport_diag_process_invalid_cmd();
      return;
    }
    port_name = subtokens[0];
    /* Check for pre-emphasis */
    if (!strncmp(
            subtokens[1], "CL93N72_UT_CTL2r.1", strlen("CL93N72_UT_CTL2r.1"))) {
      if (!strncmp(subtokens[2],
                   "CL93N72_TXFIR_PRE2",
                   strlen("CL93N72_TXFIR_PRE2"))) {
        valid_cmd = true;
        tx_pre2 = true;
      } else if (!strncmp(subtokens[2],
                          "CL93N72_TXFIR_PRE1",
                          strlen("CL93N72_TXFIR_PRE1"))) {
        valid_cmd = true;
        tx_pre1 = true;
      }
    } else if (!strncmp(subtokens[1],
                        "CL93N72_UT_CTL3r.1",
                        strlen("CL93N72_UT_CTL3r.1"))) {
      if (!strncmp(subtokens[2],
                   "CL93N72_TXFIR_MAIN",
                   strlen("CL93N72_TXFIR_MAIN"))) {
        valid_cmd = true;
        tx_main = true;
      }
    } else if (!strncmp(subtokens[1],
                        "CL93N72_UT_CTL1r.1",
                        strlen("CL93N72_UT_CTL1r.1"))) {
      if (!strncmp(subtokens[2],
                   "CL93N72_TXFIR_POST1",
                   strlen("CL93N72_TXFIR_POST1"))) {
        valid_cmd = true;
        tx_post1 = true;
      } else if (!strncmp(subtokens[2],
                          "CL93N72_TXFIR_POST2",
                          strlen("CL93N72_TXFIR_POST2"))) {
        valid_cmd = true;
        tx_post2 = true;
      }
    }
    if (!valid_cmd) {
      newport_diag_process_invalid_cmd();
      return;
    }
    char *type_str = strtok(subtokens[2], delim);
    type_str = strtok(NULL, delim);
    if (!type_str) {
      newport_diag_process_invalid_cmd();
      return;
    }
    emphasis_value = atoi(type_str);
    ret = newport_diag_service_pre_emphasis(NEWPORT_DIAG_DEF_DEV_ID,
                                            atoi(port_name),
                                            emphasis_value,
                                            tx_pre2,
                                            tx_pre1,
                                            tx_main,
                                            tx_post1,
                                            tx_post2);
    newport_diag_send_cmd_response(ret);
  }
}

/* Process show command  */
static void newport_diag_process_show_cmd(
    char subtokens[][NEWPORT_DIAG_SUBTOKEN_STR_SIZE], int num_subtokens) {
  if (num_subtokens < 1) {
    newport_diag_process_invalid_cmd();
    return;
  }
  if (!strncmp(subtokens[0], "pmap", strlen("pmap"))) {
    newport_diag_service_show_pmap(NEWPORT_DIAG_DEF_DEV_ID,
                                   newport_diag_info.file_resp_str,
                                   NEWPORT_DIAG_FILE_RESP_MAX_SIZE);
    newport_diag_send_client_resp(0, newport_diag_info.file_resp_str);
  } else if (!strncmp(subtokens[0], "temp", strlen("temp"))) {
    newport_diag_service_show_temp(NEWPORT_DIAG_DEF_DEV_ID,
                                   newport_diag_info.file_resp_str,
                                   NEWPORT_DIAG_FILE_RESP_MAX_SIZE);
    newport_diag_send_client_resp(0, newport_diag_info.file_resp_str);
  } else {
    newport_diag_process_invalid_cmd();
  }
}

/* Process vlan command  */
static void newport_diag_process_vlan_cmd(
    char subtokens[][NEWPORT_DIAG_SUBTOKEN_STR_SIZE], int num_subtokens) {
  int i, ret = 0;
  int vlan_id = 0;
  int num_pbm_ports = 0, num_ubm_ports = 0;
  int pbm[BF_DIAG_MAX_PORTS], ubm[BF_DIAG_MAX_PORTS];

  memset(&pbm, 0, sizeof(pbm));
  memset(&ubm, 0, sizeof(ubm));

  if (num_subtokens < 1) {
    newport_diag_process_invalid_cmd();
    return;
  }
  if (!strncmp(subtokens[0], "create", strlen("create"))) {
    if (num_subtokens < 2) {
      newport_diag_process_invalid_cmd();
      return;
    }
    vlan_id = atoi(subtokens[1]);
    for (i = 2; i < num_subtokens; i++) {
      if (!strncmp(subtokens[i], "pbm", strlen("pbm"))) {
        newport_diag_parse_port_list_string(
            subtokens[i], &pbm[0], &num_pbm_ports);
      } else if (!strncmp(subtokens[i], "ubm", strlen("ubm"))) {
        newport_diag_parse_port_list_string(
            subtokens[i], &ubm[0], &num_ubm_ports);
      }
    }
    /* Call handler */
    ret = newport_diag_service_vlan_create(NEWPORT_DIAG_DEF_DEV_ID,
                                           vlan_id,
                                           pbm,
                                           num_pbm_ports,
                                           ubm,
                                           num_ubm_ports);
    newport_diag_send_cmd_response(ret);
  } else if (!strncmp(subtokens[0], "add", strlen("add"))) {
    if (num_subtokens < 2) {
      newport_diag_process_invalid_cmd();
      return;
    }
    vlan_id = atoi(subtokens[1]);
    for (i = 2; i < num_subtokens; i++) {
      if (!strncmp(subtokens[i], "pbm", strlen("pbm"))) {
        newport_diag_parse_port_list_string(
            subtokens[i], &pbm[0], &num_pbm_ports);
      } else if (!strncmp(subtokens[i], "ubm", strlen("ubm"))) {
        newport_diag_parse_port_list_string(
            subtokens[i], &ubm[0], &num_ubm_ports);
      }
    }
    /* Call handler */
    ret = newport_diag_service_vlan_add(NEWPORT_DIAG_DEF_DEV_ID,
                                        vlan_id,
                                        pbm,
                                        num_pbm_ports,
                                        ubm,
                                        num_ubm_ports);
    newport_diag_send_cmd_response(ret);
  } else if (!strncmp(subtokens[0], "destroy", strlen("destroy"))) {
    if (num_subtokens < 2) {
      newport_diag_process_invalid_cmd();
      return;
    }
    vlan_id = atoi(subtokens[1]);
    /* Call handler */
    ret = newport_diag_service_vlan_destroy(NEWPORT_DIAG_DEF_DEV_ID, vlan_id);
    newport_diag_send_cmd_response(ret);
  } else if (!strncmp(subtokens[0], "show", strlen("show"))) {
    if (num_subtokens >= 2) {
      vlan_id = atoi(subtokens[1]);
    } else {
      vlan_id = -1;
    }
    /* Call handler */
    ret = newport_diag_service_vlan_show(NEWPORT_DIAG_DEF_DEV_ID,
                                         vlan_id,
                                         newport_diag_info.file_resp_str,
                                         NEWPORT_DIAG_FILE_RESP_MAX_SIZE);
    newport_diag_send_client_resp(0, newport_diag_info.file_resp_str);
  } else {
    newport_diag_process_invalid_cmd();
  }
}

/* Process led command  */
static void newport_diag_process_led_cmd(
    char subtokens[][NEWPORT_DIAG_SUBTOKEN_STR_SIZE],
    int num_subtokens,
    bool led_port) {
  int ret = 0;
  char *port_name = NULL;
  diag_front_port_t front_port = -1;
  char def_color[20] = "blue";

  /* Is it led_port command */
  if (led_port) {
    if (num_subtokens >= 1) {
      port_name = subtokens[0];
      front_port = atoi(port_name);
    }
    ret = newport_diag_service_led_set(
        NEWPORT_DIAG_DEF_DEV_ID, front_port, def_color);
    newport_diag_send_cmd_response(ret);
    return;
  }

  if (num_subtokens < 1) {
    newport_diag_process_invalid_cmd();
    return;
  }
  if (num_subtokens >= 2) {
    port_name = subtokens[1];
    front_port = atoi(port_name);
  }
  ret = newport_diag_service_led_set(
      NEWPORT_DIAG_DEF_DEV_ID, front_port, subtokens[0]);
  if (ret == BF_INVALID_ARG) {
    newport_diag_process_invalid_cmd();
  } else {
    newport_diag_send_cmd_response(ret);
  }
}

/* Process snake-test command  */
static void newport_diag_process_snake_cmd(
    char subtokens[][NEWPORT_DIAG_SUBTOKEN_STR_SIZE], int num_subtokens) {
  int ret = 0;
  if (num_subtokens < 1) {
    newport_diag_process_invalid_cmd();
    return;
  }
  if (!strncmp(subtokens[0], "init", strlen("init"))) {
    if (num_subtokens != 4) {
      newport_diag_process_invalid_cmd();
      return;
    }
    int start_port = atoi(subtokens[1]);
    int size = atoi(subtokens[2]);
    char *type = subtokens[3];
    /* Call the handler */
    ret = newport_diag_service_snake_init(
        NEWPORT_DIAG_DEF_DEV_ID, start_port, size, type);
    newport_diag_send_cmd_response(ret);
  } else if (!strncmp(subtokens[0], "run", strlen("run"))) {
    if (num_subtokens != 3) {
      newport_diag_process_invalid_cmd();
      return;
    }
    int pkt_size = atoi(subtokens[1]);
    int num_packet = atoi(subtokens[2]);
    /* Call the handler */
    ret = newport_diag_service_snake_run(pkt_size, num_packet);
    newport_diag_send_cmd_response(ret);
  } else if (!strncmp(subtokens[0], "stop", strlen("stop"))) {
    if (num_subtokens != 1) {
      newport_diag_process_invalid_cmd();
      return;
    }
    /* Call the handler */
    ret = newport_diag_service_snake_stop();
    newport_diag_send_cmd_response(ret);
  } else if (!strncmp(subtokens[0], "end", strlen("end"))) {
    /* Call the handler */
    ret = newport_diag_service_snake_end();
    newport_diag_send_cmd_response(ret);
  } else if (!strncmp(subtokens[0], "status", strlen("status"))) {
    /* Call the handler */
    ret = newport_diag_service_snake_status();
    newport_diag_send_cmd_response(ret);
  } else {
    newport_diag_send_client_resp(1, "bad cmd param");
  }
}

/* Process port command  */
static void newport_diag_process_port_cmd(
    char subtokens[][NEWPORT_DIAG_SUBTOKEN_STR_SIZE], int num_subtokens) {
  int ret = 0;
  const char delim[2] = "=";

  if (num_subtokens < 2) {
    newport_diag_process_invalid_cmd();
    return;
  }
  int port = atoi(subtokens[0]);
  if (!strncmp(subtokens[1], "fec", strlen("fec"))) {
    if (num_subtokens != 3) {
      newport_diag_process_invalid_cmd();
      return;
    }
    char *fec = subtokens[2];
    /* Call the handler */
    ret = newport_diag_service_fec(NEWPORT_DIAG_DEF_DEV_ID, port, fec);
    newport_diag_send_cmd_response(ret);
  } else if (!strncmp(subtokens[1], "autoneg", strlen("autoneg"))) {
    if (num_subtokens != 3) {
      newport_diag_process_invalid_cmd();
      return;
    }
    char *autoneg = subtokens[2];
    /* Call the handler */
    ret = newport_diag_service_port_autoneg(
        NEWPORT_DIAG_DEF_DEV_ID, port, autoneg);
    newport_diag_send_cmd_response(ret);
  } else if (!strncmp(subtokens[1], "speed", strlen("speed"))) {
    char *speed = strtok(subtokens[1], delim);
    speed = strtok(NULL, delim);
    if (!speed) {
      newport_diag_process_invalid_cmd();
      return;
    }
    /* Call the handler */
    ret = newport_diag_service_port_speed(NEWPORT_DIAG_DEF_DEV_ID, port, speed);
    newport_diag_send_cmd_response(ret);
  } else if (!strncmp(subtokens[1], "if", strlen("if"))) {
    char *intf = strtok(subtokens[1], delim);
    intf = strtok(NULL, delim);
    if (!intf) {
      newport_diag_process_invalid_cmd();
      return;
    }
    /* Call the handler */
    ret = newport_diag_service_port_intf(NEWPORT_DIAG_DEF_DEV_ID, port, intf);
    newport_diag_send_cmd_response(ret);
  } else {
    newport_diag_send_client_resp(1, "bad cmd param");
  }
}

/* Process client request and reply to client */
static void newport_diag_server_process_client_cmd(char *client_buf, int len) {
  const char delimiters[] = " ";
  char *cmd_token, cmd_str[NEWPORT_DIAG_CLIENT_CMD_MAX_SIZE];
  char *subtoken, subtokens[NEWPORT_DIAG_SUBTOKEN_CNT_LIMIT]
                           [NEWPORT_DIAG_SUBTOKEN_STR_SIZE];
  int num_subtokens = 0;

  (void)len;
  /* keep a copy of original command */
  strncpy(cmd_str, client_buf, NEWPORT_DIAG_CLIENT_CMD_MAX_SIZE);
  cmd_token = strtok(client_buf, delimiters);
  subtoken = strtok(NULL, delimiters);
  while (subtoken != NULL) {
    strncpy(subtokens[num_subtokens], subtoken, NEWPORT_DIAG_SUBTOKEN_STR_SIZE);
    num_subtokens++;
    if (num_subtokens >= NEWPORT_DIAG_SUBTOKEN_CNT_LIMIT) {
      break;
    }
    subtoken = strtok(NULL, delimiters);
  }
  if (!strncmp(cmd_token, "linespeed", strlen("linespeed"))) {
    newport_diag_process_linespeed_cmd(subtokens, num_subtokens);
  } else if (!strncmp(cmd_token, "ps", strlen("ps"))) {
    newport_diag_process_ps_cmd(subtokens, num_subtokens);
  } else if (!strncmp(cmd_token, "phy", strlen("phy"))) {
    newport_diag_process_phy_cmd(subtokens, num_subtokens);
  } else if (!strncmp(cmd_token, "show", strlen("show"))) {
    newport_diag_process_show_cmd(subtokens, num_subtokens);
  } else if (!strncmp(cmd_token, "vlan", strlen("vlan"))) {
    newport_diag_process_vlan_cmd(subtokens, num_subtokens);
  } else if (!strncmp(cmd_token, "led_port", strlen("led_port"))) {
    newport_diag_process_led_cmd(subtokens, num_subtokens, true);
  } else if (!strncmp(cmd_token, "led", strlen("led"))) {
    newport_diag_process_led_cmd(subtokens, num_subtokens, false);
  } else if (!strncmp(cmd_token, "snake", strlen("snake"))) {
    newport_diag_process_snake_cmd(subtokens, num_subtokens);
  } else if (!strncmp(cmd_token, "port", strlen("port"))) {
    newport_diag_process_port_cmd(subtokens, num_subtokens);
  } else if (!strncmp(cmd_token, "version", strlen("version"))) {
    newport_diag_process_version_cmd();
  } else {
    newport_diag_process_invalid_cmd();
  }
}

/* Initialize and start diags server */
bf_status_t newport_diag_server_init(void) {
  /* Set up sockets to the model for DMA messages */
  int server_sock, client_sock;
  int rd_len;
  char client_buf[NEWPORT_DIAG_CLIENT_CMD_MAX_SIZE];
  uint32_t cmd_len, network_cmd_len;

  LOG_DEBUG("Loading Accton Diag version %s ", bf_pltfm_diag_get_version());
  printf("Loading Accton Diag version %s \n", bf_pltfm_diag_get_version());
  memset(&newport_diag_info, 0, sizeof(newport_diag_info));

  server_sock = newport_diag_create_server(NEWPORT_DIAG_TCP_PORT_BASE_DEFAULT);
  newport_diag_info.server_sock = server_sock;

  /* Register ucli node */
  newport_diag_register_ucli_node();
  /* Add ports */
  newport_diag_ports_add(NEWPORT_DIAG_DEF_DEV_ID);
  /* Enable temp monitoring */
  bf_pltfm_temperature_monitor_enable(true);

  /* listen to client commands and execute them */
  while (1) {
    client_sock = get_newport_diags_client_conn(server_sock);
    newport_diag_info.client_sock = client_sock;
    /* first read the four bytes of length */
    rd_len = newport_diag_server_read_from_socket(
        client_sock, (char *)&network_cmd_len, sizeof(uint32_t));
    /* now read the expected number of command bytes */
    cmd_len = ntohl(network_cmd_len);
    rd_len =
        newport_diag_server_read_from_socket(client_sock, client_buf, cmd_len);
    newport_diag_server_process_client_cmd(client_buf, rd_len);
    bf_sys_usleep(10000);
  }

  return 0;
}

/* newport_diag thread exit callback API */
static void newport_diag_exit_cleanup(void *arg) {
  (void)arg;
  if (newport_diag_info.server_sock) {
    close(newport_diag_info.server_sock);
    newport_diag_info.server_sock = 0;
  }
  if (newport_diag_info.client_sock) {
    close(newport_diag_info.client_sock);
    newport_diag_info.client_sock = 0;
  }
  if (newport_diag_info.resp_filep) {
    fclose(newport_diag_info.resp_filep);
    newport_diag_info.resp_filep = NULL;
  }
  bf_sys_usleep(10000); /* give sockets a chance to close */
}

/* Init function  */
void *bf_switchd_agent_init(void *arg) {
  (void)arg;
  /* Register cleanup handler */
  pthread_cleanup_push(newport_diag_exit_cleanup,
                       "newport Diag Cleanup handler");
  newport_diag_server_init();
  pthread_cleanup_pop(0);
  return NULL;
}
