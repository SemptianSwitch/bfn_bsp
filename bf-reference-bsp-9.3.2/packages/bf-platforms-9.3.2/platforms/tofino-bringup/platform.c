/*******************************************************************************
 * Copyright (c) 2015-2020 Barefoot Networks, Inc.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * $Id: $
 *
 ******************************************************************************/
#include <stdio.h>
#define __USE_GNU /* See feature_test_macros(7) */
#include <pthread.h>

#include <bfsys/bf_sal/bf_sys_intf.h>
#include <bf_pltfm_types/bf_pltfm_types.h>
#include <bf_switchd/bf_switchd.h>
#include <bf_bd_cfg/bf_bd_cfg_intf.h>
#include <bf_bd_cfg/bf_bd_cfg_porting.h>

#include <bf_led/bf_led.h>
#include <bf_pltfm.h>
#include <bf_qsfp/bf_qsfp.h>
#include <bf_pltfm_bd_cfg.h>
#include <tcl_server.h>

#include <bfutils/uCli/ucli.h>
#include <bfutils/uCli/ucli_argparse.h>
#include <bfutils/uCli/ucli_handler_macros.h>

#if 1
#include "semp_types.h"
#include "sys_led.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>


#endif

extern int pltfm_create_bd_map(char *json_file);
extern void pltfm_clear_bd_map(void);

/* global */
static pltfm_mgr_info_t pltfm_mgr_info;
static ucli_node_t *bf_tof_bringup_plat_ucli_node;

#ifdef THRIFT_ENABLED
int bf_pltfm_agent_rpc_server_thrift_service_add(void *processor) {
  (void)processor;
  return 0;
}

int bf_pltfm_agent_rpc_server_thrift_service_rmv(void *processor) {
  (void)processor;
  return 0;
}
#endif

static ucli_command_handler_f bf_pltfm_ucli_ucli_handlers__[] = {NULL};

static ucli_module_t bf_pltfm_ucli_module__ = {
    "bf_pltfm_ucli", NULL, bf_pltfm_ucli_ucli_handlers__, NULL, NULL};

ucli_node_t *bf_tof_bringup_plat_ucli_node_create(void) {
  ucli_node_t *n;
  ucli_module_init(&bf_pltfm_ucli_module__);
  n = ucli_node_create("bf_pltfm", NULL, NULL);
  ucli_node_subnode_add(n, ucli_module_log_node_create("bf_pltfm"));
  bf_bd_cfg_ucli_node_create(n);
  return n;
}

/* platform-mgr thread exit callback API */
void bf_pltfm_platform_exit(void *arg) {
  (void)arg;

  pltfm_clear_bd_map();
  //  pltfm_mgr_stop_tcl_server();
}

bf_pltfm_status_t bf_pltfm_device_type_get(bf_dev_id_t dev_id,
                                           bool *is_sw_model) {
  *is_sw_model = false;
  return BF_PLTFM_SUCCESS;
}

#if 1
#define I2C_DEVICE_PATH_MAX_LEN 128

SERV watch_dog_base_path_get(char *path)
{
    SERV ret = SE_SUCCESS;
	char path_tmp[I2C_DEVICE_PATH_MAX_LEN] = {0};

	snprintf(path_tmp, sizeof(path_tmp), "/sys/class/hwmon/hwmon4"); 

    memcpy(path, path_tmp, sizeof(path_tmp));
    return ret;
}




SERV watch_dog_feed_dog()
{
    SERV ret = SE_SUCCESS;
	char watch_dog_base_path[I2C_DEVICE_PATH_MAX_LEN] = {0};
	char watch_dog_path[256] = {0};
	
	watch_dog_base_path_get(watch_dog_base_path);

	snprintf(watch_dog_path, sizeof(watch_dog_path), "%s/wdt_signal", watch_dog_base_path); 
	
	int fd=open(watch_dog_path, O_WRONLY, 0664);
	if (fd < 0)
	{ 
	  printf("open sys watch_dog_path[%s]\r\n", watch_dog_path);
	  return -1;
	}	
	
	ret = write(fd, "1", strlen("1"));
	
	close(fd);
	
	if (ret < 0)
	{
		printf("write watch_dog_path[%s] error, script exit code: %d!", watch_dog_path, WEXITSTATUS(ret));
	}

    return ret;
}

SERV watch_dog_enable_set(int state)
{
    SERV ret = SE_SUCCESS;	
	char watch_dog_base_path[I2C_DEVICE_PATH_MAX_LEN] = {0};
	char watch_dog_path[256] = {0};
	
	watch_dog_base_path_get(watch_dog_base_path);

	snprintf(watch_dog_path, sizeof(watch_dog_path), "%s/wdt_enb", watch_dog_base_path); 
	
	int fd=open(watch_dog_path, O_WRONLY, 0664);
	if (fd < 0)
	{ 
	  printf("open sys watch_dog_path[%s]\r\n", watch_dog_path);
	  return -1;
	}	

	if (1 == state)
	{
	    ret = write(fd, "1", strlen("1"));
	}
	else
	{
        ret = write(fd, "0", strlen("0"));
	}
	
	close(fd);
	
	if (ret < 0)
	{
		printf("write watch_dog_path[%s] error, script exit code: %d!", watch_dog_path, WEXITSTATUS(ret));
	}

    return ret;
}


SERV watch_dog_time_get(uint32_t *time)
{
    SERV ret = SE_SUCCESS;
	uint8_t buf[16] = {0};
	
	LOG_DEBUG("[watchdog] func[%s] line[%d]\r\n", __func__, __LINE__);	
	char watch_dog_base_path[I2C_DEVICE_PATH_MAX_LEN] = {0};
	char watch_dog_path[256] = {0};
	
	watch_dog_base_path_get(watch_dog_base_path);

	snprintf(watch_dog_path, sizeof(watch_dog_path), "%s/wdt_time", watch_dog_base_path); 	
	
	int fd=open(watch_dog_path, O_RDONLY, 0664);
	if (fd < 0)
	{ 
	  printf("open sys watch_dog_path[%s]\r\n", watch_dog_path);
	  return -1;
	}	
	
	ret = read(fd, buf, sizeof(buf));
	
	close(fd);
	
	if (ret < 0)
	{
		printf("write watch_dog_path[%s] error, script exit code: %d!", watch_dog_path, WEXITSTATUS(ret));
		return ret;
	}
	
    LOG_DEBUG("[watchdog] func[%s] line[%d] buf:%s *time:%d ret:%d\r\n", __func__, __LINE__, buf, *time, ret);
	
	*time = (uint32_t)strtol((char *)buf, NULL, 10);

	LOG_DEBUG("[watchdog] func[%s] line[%d] buf:%s *time:%d\r\n", __func__, __LINE__, buf, *time);

    return ret;
}


SERV watch_dog_reboot_counter_get(uint32_t *time)
{
    SERV ret = SE_SUCCESS;
	char buf[16] = {0};
	char watch_dog_base_path[I2C_DEVICE_PATH_MAX_LEN] = {0};
	char watch_dog_path[256] = {0};
	
	watch_dog_base_path_get(watch_dog_base_path);

	snprintf(watch_dog_path, sizeof(watch_dog_path), "%s/wdt_valid", watch_dog_base_path); 			
	
	int fd=open(watch_dog_path, O_RDONLY, 0664);
	if (fd < 0)
	{ 
	  printf("open sys watch_dog_path[%s]\r\n", watch_dog_path);
	  return -1;
	}	
	
	ret = read(fd, buf, sizeof(buf));
	
	close(fd);
	
	if (ret < 0)
	{
		printf("read watch_dog_path[%s] error, script exit code: %d!", watch_dog_path, WEXITSTATUS(ret));
		return ret;
	}

	*time = (uint32_t)strtol((char *)buf, NULL, 10);

    return ret;
}

SERV watch_dog_reboot_counter_clear()
{
    SERV ret = SE_SUCCESS;
	char watch_dog_base_path[I2C_DEVICE_PATH_MAX_LEN] = {0};
	char watch_dog_path[256] = {0};
	
	watch_dog_base_path_get(watch_dog_base_path);
	snprintf(watch_dog_path, sizeof(watch_dog_path), "%s/wdt_valid", watch_dog_base_path); 	
	
	int fd=open(watch_dog_path, O_WRONLY, 0664);
	if (fd < 0)
	{ 
	  printf("open sys watch_dog_path[%s]\r\n", watch_dog_path);
	  return -1;
	}	
	
	ret = write(fd, "0", strlen("0"));
	
	close(fd);
	
	if (ret < 0)
	{
		printf("write watch_dog_path[%s] error, script exit code: %d!", watch_dog_path, WEXITSTATUS(ret));
	}

    return ret;
}


SERV watch_dog_soft_enb_stat_get(int* stat)
{
    SERV ret = SE_SUCCESS;
	char watch_dog_enb_stat_path[128] = {0};
	char buf[16] = {0};
	

    snprintf(watch_dog_enb_stat_path, sizeof(watch_dog_enb_stat_path), "/opt/bfn/install/lib/platform/x86_64-semptian_ps7350_32x-r0/wdt_enb"); 


	int fd=open(watch_dog_enb_stat_path, O_RDONLY, 0664);
	if (fd < 0)
	{ 
	  printf("open sys watch_dog_enb_stat_path[%s]\r\n", watch_dog_enb_stat_path);
	  return -1;
	}	
	
	ret = read(fd, buf, sizeof(buf));
	
	close(fd);
	
	if (ret < 0)
	{
		printf("read watch_dog_path[%s] error, script exit code: %d!", watch_dog_enb_stat_path, WEXITSTATUS(ret));
		return ret;
	}

	*stat = (int)strtol((char *)buf, NULL, 10);

    return ret;
}

SERV watch_dog_enb_init()
{
	int stat = 0;
	SERV ret = SE_SUCCESS;

	LOG_WARNING("[watchdog] func[%s] line[%d] init watch_dog enb stat\r\n", __func__, __LINE__);

	ret = watch_dog_soft_enb_stat_get(&stat);

	printf("[watchdog] func[%s] line[%d] get watch_dog soft enb stat[%d]\r\n", __func__, __LINE__, stat);
	
    if (1 == stat)
    {
	    watch_dog_enable_set(1);
	}else if (0 == stat)
	{
	    watch_dog_enable_set(0);
	}
	
	return ret;
}

SERV watch_dog_timer_get(int* time)
{
    SERV ret = SE_SUCCESS;
	char watch_dog_time_path[128] = {0};
	char buf[16] = {0};

	snprintf(watch_dog_time_path, sizeof(watch_dog_time_path), "/opt/bfn/install/lib/platform/x86_64-semptian_ps7350_32x-r0/wdt_timer"); 

	int fd=open(watch_dog_time_path, O_RDONLY, 0664);
	if (fd < 0)
	{ 
	  printf("open sys watch_dog_time_path[%s]\r\n", watch_dog_time_path);
	  return -1;
	}	
	
	ret = read(fd, buf, sizeof(buf));
	
	close(fd);
	
	if (ret < 0)
	{
		printf("read watch_dog_path[%s] error, script exit code: %d!", watch_dog_time_path, WEXITSTATUS(ret));
		return ret;
	}

	*time = (int)strtol((char *)buf, NULL, 10);

    return ret;
}


SERV watch_dog_timer_init()
{
	int timer = 0;
	SERV ret = SE_SUCCESS;

	ret = watch_dog_timer_get(&timer);
	
	LOG_DEBUG("[watchdog] func[%s] line[%d] init watch_dog timer[%d]\r\n", __func__, __LINE__, timer);
	

	
	return ret;
}


static void * watch_dog_pthread(void *arg) 
{

	printf("[watchdog] watch_dog_pthread start \r\n");
	LOG_WARNING("[watchdog] watch_dog_pthread start \r\n");
	
	uint32_t feed_dog_time = 6;
	
	watch_dog_enb_init();

	while(1)
	{
	    watch_dog_time_get(&feed_dog_time);
		
		printf("[watchdog] func[%s] line[%d] feed_dog_time[0x%x]\r\n", 
			__func__, __LINE__, feed_dog_time);
		LOG_WARNING("[watchdog] func[%s] line[%d] feed_dog_time[0x%x]\r\n", 
			__func__, __LINE__, feed_dog_time);
		
		feed_dog_time = feed_dog_time/6;		
		
		watch_dog_feed_dog();
		
		if (feed_dog_time > 0)
		{
		    sleep(feed_dog_time);
		}
		else
		{
		    sleep(10);
		}
	}
	
	return NULL;
}

int watch_dog_init()
{
	pthread_t tidp;
	char *data = NULL;
	
	printf("func[%s] line[%d]\r\n", __func__, __LINE__);
	LOG_WARNING("[watchdog] func[%s] line[%d]\r\n", __func__, __LINE__);
	
	/* ¡ä¡ä?¡§??3¨¬pthread */
	if ((pthread_create(&tidp, NULL, watch_dog_pthread, (void*)data)) == -1)
	{
		printf("platform_monitor_pthread create error!\n");
		return 1;
	}	
	
	return 0;	
}




#endif


#if 1




#define SYS_LED_BASE_I2C_DEVICE_PATH_MAX_LEN 128
#define SYS_LED_I2C_DEVICE_PATH_MAX_LEN 256
int sys_led_base_path_get(char *path)
{
    SERV ret = SE_SUCCESS;

	char path_tmp[SYS_LED_BASE_I2C_DEVICE_PATH_MAX_LEN] = {0};
	

	snprintf(path_tmp, sizeof(path_tmp), "/sys/class/hwmon/hwmon4"); 


    memcpy(path, path_tmp, sizeof(path_tmp));
    return ret;
}

int bf_pltfm_ps7350_sys_led_set(bdd_sys_led_type_e led_type, bdd_sys_led_e led_stat) 
{
  char led_path[SYS_LED_I2C_DEVICE_PATH_MAX_LEN] = {0};
  char path_base[SYS_LED_BASE_I2C_DEVICE_PATH_MAX_LEN] = {0};
  int ret = 0;
  sys_led_base_path_get(path_base);
  
  if (SYS_LED_TYPE_STATE == led_type)
  {
	  
	snprintf(led_path, sizeof(led_path), "%s/system_led", path_base); 
  }else if (SYS_LED_TYPE_ALARM == led_type)
  {
    snprintf(led_path, sizeof(led_path), "%s/alarm_led", path_base);
  }
  

  int fd=open(led_path, O_WRONLY, 0664);
  if (fd < 0)
  {	
    printf("open sys led path err led_path[%s]\r\n", led_path);
	return -1;
  }

  if (SYS_LED_ON == led_stat)
  {
	 ret = write(fd, SYS_LED_STR_ON, strlen(SYS_LED_STR_ON));
  }else if (SYS_LED_BLINK_1S == led_stat)
  {
     ret = write(fd, SYS_LED_STR_BLINK_1S, strlen(SYS_LED_STR_BLINK_1S));
  }else if (SYS_LED_BLINK_500MS == led_stat)
  {
	  ret = write(fd, SYS_LED_STR_BLINK_500MS, strlen(SYS_LED_STR_BLINK_500MS));
  }else if (SYS_LED_BLINK_125MS == led_stat)
  {
	  ret = write(fd, SYS_LED_STR_BLINK_125MS, strlen(SYS_LED_STR_BLINK_125MS));
  }else if (SYS_LED_OFF == led_stat)
  {
      ret = write(fd, SYS_LED_STR_OFF, strlen(SYS_LED_STR_OFF)); 
  }
  
  close(fd);
  
  if (ret < 0)
  {
	  printf("write led_path[%s] error, script exit code: %d!", led_path, WEXITSTATUS(ret));
  }

  return ret;
}


int bf_pltfm_sys_led_init() 
{
  bf_pltfm_ps7350_sys_led_set(SYS_LED_TYPE_STATE, SYS_LED_BLINK_1S);

  return 0;
}

int bf_pltfm_sys_led_set(bdd_pltfm_type_e pltfm_type, bdd_sys_led_type_e led_type, bdd_sys_led_e led_stat) 
{
  int ret = 0;

  ret = bf_pltfm_ps7350_sys_led_set(led_type, led_stat);

  return ret;
}



SERV bmc_alarm_info_check(uint32_t *alarm_num)
{
    SERV ret = SE_SUCCESS;
    FILE *fp = NULL;
	
    char cmd[256] = {0};
    char line[128] = {0};
	
    char *saveptr_tmp = NULL;
    char *saveptr = NULL;
	
	char *strRole0 = NULL;
	char *strRole1 = NULL;
	char *strRole2 = NULL;
	char *value_str = NULL;
	uint32_t alarm_state = 0;

	
    snprintf(cmd, sizeof(cmd), "/opt/bfn/install/lib/platform/x86_64-semptian_ps7350_32x-r0/ipmitool sdr");
	

    if ((fp = popen(cmd,"r")) == NULL)  
    {
        printf("Popen error :%s.\n", cmd);
		LOG_ERROR("Popen error :%s.\n", cmd);
        return SE_OPEN;
    }

    while (fgets(line, sizeof(line), fp))
    {
		//printf("1111--------------------------------------- line:%s \r\n", line);
		strRole0 = strtok_r(line, "|", &saveptr);
		
		strRole1 = strtok_r(NULL, "|", &saveptr);
		
		strRole1 = strtok_r(strRole1, " ", &saveptr_tmp);
		
		//printf("2222 strRole:%s strlen(strRole):%d\r\n", strRole, strlen(strRole));
		if (!strRole1 || !strncmp(strRole1,"disable",7))
		{
			
			//printf("not care strRole0:%s!!!!!!!!!!!!!!!!\r\n", strRole0);
			continue;
		}
		
		strRole2 = strtok_r(NULL, "|", &saveptr);
		strRole2 = strtok_r(strRole2, " ", &value_str);
	
		//printf("3333 strRole:%s strlen(strRole):%d\r\n", strRole, strlen(strRole));
		if (!strRole2 || !strncmp(strRole2, "nr", 2) || !strncmp(strRole2, "sr", 2))
		{
			
			printf("warning  strRole0:%s %s!!!!!!!!!!!!!!!!\r\n", strRole0, strRole2);
			LOG_WARNING("warning  strRole0:%s %s!!!!!!!!!!!!!!!!\r\n", strRole0, strRole2);
			alarm_state++;
		}		
    }
	
	*alarm_num = alarm_state;
	
    pclose(fp);

    return ret;
}

uint32_t g_alarm = 0;

static void * platform_monitor_pthread(void *arg) 
{
	uint32_t alarm_num = 0;
	int ret = 0;
	
	bdd_pltfm_type_e pltfm_type = PLTFM_TYPE_PS8550;
	//bf_pltfm_board_id_t board_id;

    pltfm_type = PLTFM_TYPE_PS7350;
	

	LOG_DEBUG("platform_monitor_pthread start pltfm_type:%d\r\n", pltfm_type);
	while(1)
	{
		bmc_alarm_info_check(&alarm_num);
		if (g_alarm != alarm_num)
		{
			
			LOG_WARNING("warning  g_alarm:%d changed to alarm_num:%d !!!!!!!!!!!!!!!!\r\n", g_alarm, alarm_num);
			g_alarm = alarm_num;
			if (alarm_num > 4)
			{
				LOG_WARNING("warning  alarm_num:%d !!!!!!!!!!!!!!!!\r\n", alarm_num);
				ret = bf_pltfm_sys_led_set(pltfm_type, SYS_LED_TYPE_ALARM, SYS_LED_BLINK_125MS);
			}else if (alarm_num > 2)
			{
				LOG_WARNING("warning  alarm_num:%d !!!!!!!!!!!!!!!!\r\n", alarm_num);
				ret = bf_pltfm_sys_led_set(pltfm_type, SYS_LED_TYPE_ALARM, SYS_LED_BLINK_500MS);
			}else if (alarm_num > 0)
			{
				LOG_WARNING("warning  alarm_num:%d !!!!!!!!!!!!!!!!\r\n", alarm_num);
				ret = bf_pltfm_sys_led_set(pltfm_type, SYS_LED_TYPE_ALARM, SYS_LED_BLINK_1S);
			}
			else if (0 == alarm_num)
			{
				ret = bf_pltfm_sys_led_set(pltfm_type, SYS_LED_TYPE_ALARM, SYS_LED_OFF);
			}	

			if (ret < 0)
			{
               g_alarm = 0;
			   LOG_ERROR("[monitor] pltfm_type:%d set sys led err[%d]\r\n", pltfm_type, ret);
			}
		}
		
		sleep(5);
	}
	
	return NULL;
}

int platform_monitor_init()
{
	pthread_t tidp;
	char *data = NULL;
	
	printf("%s %d\r\n", __func__, __LINE__);
	
	/* create pthread*/
	if ((pthread_create(&tidp, NULL, platform_monitor_pthread, (void*)data)) == -1)
	{
		printf("platform_monitor_pthread create error!\n");
		return 1;
	}	
	
	return 0;	
}


#endif

/* Initialize and start pltfm_mgrs server */
bf_status_t bf_pltfm_platform_init(bf_switchd_context_t *switchd_ctx) {
  int ret = 0;
  char fname[128];

  memset(&pltfm_mgr_info, 0, sizeof(pltfm_mgr_info));
  //  pltfm_mgr_start_tcl_server();
  (void)ret;

  if (switchd_ctx->install_dir)
  {
      snprintf(fname,
           sizeof(fname),
           "%s/share/platforms/tofino-bringup/board_lane_map.json",
           switchd_ctx->install_dir);
  }
  else
  {
      switchd_ctx->install_dir = "/opt/bfn/install";
      printf("%s %d not parse switch install dir set default to /opt/bfn/install\r\n", __func__, __LINE__);
      snprintf(fname,
           sizeof(fname),
           "%s/share/platforms/tofino-bringup/board_lane_map.json",
           switchd_ctx->install_dir);
  
  }

  if (pltfm_create_bd_map(fname) != 0) {
    LOG_ERROR("pltfm_mgr: bd_lane_map init failed\n");
    return -1;
  }
  bf_qsfp_set_num(platform_num_ports_get());

  bf_tof_bringup_plat_ucli_node = bf_tof_bringup_plat_ucli_node_create();
  bf_drv_shell_register_ucli(bf_tof_bringup_plat_ucli_node);

  //bf_pltfm_sys_led_init();
  
  //watch_dog_init();

  //platform_monitor_init();  

  return BF_PLTFM_SUCCESS;
}
