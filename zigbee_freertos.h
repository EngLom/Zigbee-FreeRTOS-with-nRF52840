#ifndef _ZIGBEE_FREERTOS_H
#define _ZIGBEE_FREERTOS_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" 
{
#endif

#if (NRF_LOG_ENABLED && NRF_LOG_DEFERRED)
/**@brief Task function responsible for deferred log processing.
/**@param pvParameter     FreeRTOS task parameter, unused here, required by FreeRTOS API.*/
static void logger_task(void *pvParameter)
{
    UNUSED_PARAMETER(pvParameter);

    while (true)
    {
        if (!(NRF_LOG_PROCESS()))
        {
            /* No more logs, let's sleep and wait for any */
            vTaskDelay(1);
        }
    }
}
#endif

typedef struct
{
    zb_int16_t  measured_value ;
} update_temperature_measurement_ctx_t;

bool led_toggle_task(void *pvParameter);

bool freertos_int(void);

bool zboss_signal_handler(zb_uint8_t param);

bool zigbee_main_task(void *pvParameter);

void multi_sensor_clusters_attr_init(void);

void update_temperature_measurement_cb(zb_uint8_t param);

void temperature_measurement_timer_handler(void * context);

void vApplicationIdleHook(void);
    
#ifdef __cplusplus
}
#endif

#endif
