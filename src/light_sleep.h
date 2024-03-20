#ifndef __LIGHT_SLEEP_H__
#define __LIGHT_SLEEP_H__

#include "esp_check.h"

#define TIMER_WAKEUP_TIME_US (10 * 1000 * 1000)

esp_err_t set_timer_wakeup(void);
static void light_sleep(void *args);

#endif
