#ifndef __SYS_LED_H__
#define __SYS_LED_H__

#define SYS_LED_STR_ON "0"
#define SYS_LED_STR_BLINK_1S "1" 
#define SYS_LED_STR_BLINK_500MS "2"
#define SYS_LED_STR_BLINK_125MS "3"
#define SYS_LED_STR_OFF "4"

typedef enum
{
    SYS_LED_ON = 0,
    SYS_LED_BLINK_1S = 1,
    SYS_LED_BLINK_500MS = 2,
    SYS_LED_BLINK_125MS = 3,
    SYS_LED_OFF = 4,
    SYS_LED_MAX,
} bdd_sys_led_e;

typedef enum
{
    SYS_LED_TYPE_ALARM = 0,
    SYS_LED_TYPE_STATE = 1,
    SYS_LED_TYPE_MAX,
} bdd_sys_led_type_e;

typedef enum
{
    PLTFM_TYPE_PS8550 = 0,
	PLTFM_TYPE_PS8560 = 1,
	PLTFM_TYPE_PS7350 = 2,
    PLTFM_TYPE_MAX,
} bdd_pltfm_type_e;

int bf_pltfm_sys_led_set(bdd_pltfm_type_e pltfm_type, bdd_sys_led_type_e led_type, bdd_sys_led_e led_stat);
int bf_pltfm_8560_sys_led_init();
#endif /*__SYS_LED_H__ */