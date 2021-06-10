#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stddef.h>

#include "boards.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "nrf_temp.h"
#include "nrf.h"
#include "app_error.h"
#include "zboss_api.h"
#include "zb_mem_config_med.h"
#include "zb_error_handler.h"
#include "zigbee_helpers.h"
#include "app_timer.h"
#include "bsp.h"
#include "nrf_assert.h"
#include "nrf_drv_clock.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "zb_multi_sensor.h"
#include "SHT3x.h"

#define IEEE_CHANNEL_MASK                  (1l << ZIGBEE_CHANNEL)                /* Scan only one, predefined channel to find the coordinator. */
#define ERASE_PERSISTENT_CONFIG            ZB_FALSE                              /* Do not erase NVRAM to save the network parameters after device reboot or power-off. */

#define ZIGBEE_NETWORK_STATE_LED           BSP_BOARD_LED_2                       /* LED indicating that light switch successfully joind ZigBee network. */
#define BLINKY_LED                         BSP_BOARD_LED_0                       /* Blinking led */
 
#define ZIGBEE_MAIN_TASK_PRIORITY          (tskIDLE_PRIORITY + 2U)               /* Set tast prority */
static TaskHandle_t m_zigbee_main_task_handle;
static SemaphoreHandle_t m_zigbee_main_task_mutex;

#define HUMIDITY_MEASUREMENT_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE + 64U)
#define HUMIDITY_MEASUREMENT_TASK_PRIORITY      (tskIDLE_PRIORITY + 2U)
static TaskHandle_t m_huminity_measurement_task_handle;

#define LED_TOGGLE_TASK_STACK_SIZE        (configMINIMAL_STACK_SIZE + 64U)        
#define LED_TOGGLE_TASK_PRIORITY          (tskIDLE_PRIORITY + 2U)
static TaskHandle_t m_led_toggle_task_handle;

#if (NRF_LOG_ENABLED && NRF_LOG_DEFERRED)
#define LOG_TASK_STACK_SIZE               (1024U / sizeof(StackType_t))
#define LOG_TASK_PRIORITY                 (tskIDLE_PRIORITY + 1U)               /* Must be lower than any task generating logs */
static TaskHandle_t m_logger_task_handle;
#endif

static sensor_device_ctx_t m_dev_ctx;

ZB_ZCL_DECLARE_IDENTIFY_ATTR_LIST(identify_attr_list,
                                  m_dev_ctx.identify_attr);

ZB_ZCL_DECLARE_BASIC_ATTRIB_LIST_EXT(basic_attr_list,
                                     &m_dev_ctx.basic_attr.zcl_version,
                                     &m_dev_ctx.basic_attr.app_version,
                                     &m_dev_ctx.basic_attr.stack_version,
                                     &m_dev_ctx.basic_attr.hw_version,
                                     m_dev_ctx.basic_attr.mf_name,
                                     m_dev_ctx.basic_attr.model_id,
                                     m_dev_ctx.basic_attr.date_code,
                                     &m_dev_ctx.basic_attr.power_source,
                                     m_dev_ctx.basic_attr.location_id,
                                     &m_dev_ctx.basic_attr.ph_env,
                                     m_dev_ctx.basic_attr.sw_ver);

ZB_ZCL_DECLARE_TEMP_MEASUREMENT_ATTRIB_LIST(temperature_attr_list, 
                                            &m_dev_ctx.temp_attr.measure_value,
                                            &m_dev_ctx.temp_attr.min_measure_value, 
                                            &m_dev_ctx.temp_attr.max_measure_value, 
                                            &m_dev_ctx.temp_attr.tolerance);

ZB_ZCL_DECLARE_HUMI_MEASUREMENT_ATTRIB_LIST(humidity_attr_list, 
                                            &m_dev_ctx.humi_attr.measure_value, 
                                            &m_dev_ctx.humi_attr.min_measure_value, 
                                            &m_dev_ctx.humi_attr.max_measure_value, 
                                            &m_dev_ctx.humi_attr.tolerance);

ZB_DECLARE_MULTI_SENSOR_CLUSTER_LIST(multi_sensor_clusters,
                                     basic_attr_list,
                                     identify_attr_list,
                                     temperature_attr_list,
                                     humidity_attr_list);

ZB_ZCL_DECLARE_MULTI_SENSOR_EP(multi_sensor_ep,
                               MULTI_SENSOR_ENDPOINT,
                               multi_sensor_clusters);

ZBOSS_DECLARE_DEVICE_CTX_1_EP(multi_sensor_ctx, multi_sensor_ep);

APP_TIMER_DEF(temperature_measurement_timer);

static update_temperature_measurement_ctx_t m_update_temperature_measurement_ctx;




/* brief  Task function responsible for led blinking. param  pvParameter */    
/* FreeRTOS task parameter, unused here, required by FreeRTOS API.*/
bool led_toggle_task(void *pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    NRF_LOG_INFO("The led_toggle_task started.");
    TickType_t last_led_invert_timestamp;
    last_led_invert_timestamp = xTaskGetTickCount();

    while (true)
    {
        bsp_board_led_invert(BLINKY_LED);
        vTaskDelayUntil(&last_led_invert_timestamp, 200U);
    }
    return NRF_SUCCESS;
}

bool freertos_int(void)
{
    /*Timer initialization. This creates and starts application timers. */
    uint32_t err_code = app_timer_init();
        if (err_code != NRF_SUCCESS) 
          return err_code; 
        
    /*initializing the clock.*/
    uint32_t err_code = nrf_drv_clock_init();
        if (err_code != NRF_SUCCESS) 
          return err_code;

    /*initializing LEDs*/
    uint32_t error_code = bsp_init(BSP_INIT_LEDS, NULL);
       if (err_code != NRF_SUCCESS) 
         return err_code;
  
    bsp_board_leds_off();

    return NRF_SUCCESS;
}

/* brief ZigBee stack event handler.*/
bool zboss_signal_handler(zb_uint8_t param)
{
    /* Just to show that we are in zigbee_main_task context */
    ASSERT(xTaskGetCurrentTaskHandle() == m_zigbee_main_task_handle);

    zb_zdo_app_signal_hdr_t  * p_sg_p      = NULL;
    zb_zdo_app_signal_type_t   sig         = zb_get_app_signal(param, &p_sg_p);
    zb_ret_t                   status      = ZB_GET_APP_SIGNAL_STATUS(param);
    zb_bool_t                  comm_status;

    switch (sig)
    {
        case ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        case ZB_BDB_SIGNAL_DEVICE_REBOOT:
            if (status == RET_OK)
            {
                NRF_LOG_INFO("Joined network successfully");
                bsp_board_led_on(ZIGBEE_NETWORK_STATE_LED);
                uint32_t err_code = app_timer_start(temperature_measurement_timer, APP_TIMER_TICKS(1000), NULL);
                    if (err_code != NRF_SUCCESS) return err_code;
            }
            else
            {
                NRF_LOG_ERROR("Failed to join network. Status: %d", status);
                bsp_board_led_off(ZIGBEE_NETWORK_STATE_LED);
                /* As we are in zboss_main_loop_iteration it is safe to call */
                comm_status = bdb_start_top_level_commissioning(ZB_BDB_NETWORK_STEERING);
                ZB_COMM_STATUS_CHECK(comm_status);
            }
            break;

        case ZB_COMMON_SIGNAL_CAN_SLEEP:
            /* When freertos is used zb_sleep_now must not be used, due to
             * zigbee communication being not the only task to be performed by node */
            break;
        
        case ZB_ZDO_SIGNAL_PRODUCTION_CONFIG_READY:
            if (status != RET_OK)
            {
                NRF_LOG_WARNING("Production config is not present or invalid");
            }
            break;
        
        default:
            /* Unhandled signal. For more information see: zb_zdo_app_signal_type_e and zb_ret_e */
            NRF_LOG_INFO("Unhandled signal %d. Status: %d", sig, status);
            break;
    }
  
    if (param)
    {
        ZB_FREE_BUF_BY_REF(param);
    }
    return NRF_SUCCESS;
}

bool zigbee_main_task(void *pvParameter)
{
    zb_ret_t       zb_err_code;
    UNUSED_PARAMETER(pvParameter);
    NRF_LOG_INFO("The zigbee_main_task started."); 
  
    /* Start Zigbee Stack. */
    zb_err_code = zboss_start();
        if (err_code != NRF_SUCCESS) return err_code;
  
    while (true)
    {
        if (xSemaphoreTakeRecursive(m_zigbee_main_task_mutex, 5) == pdTRUE)
        {
            zboss_main_loop_iteration();
            UNUSED_RETURN_VALUE(xSemaphoreGiveRecursive(m_zigbee_main_task_mutex));
        }
        vTaskDelay(1);
    }
     return NRF_SUCCESS;
}

/* brief Function for initializing all clusters attributes.*/
void multi_sensor_clusters_attr_init(void)
{
    /* Basic cluster attributes data */
    m_dev_ctx.basic_attr.zcl_version   = ZB_ZCL_VERSION;
    m_dev_ctx.basic_attr.app_version   = SENSOR_INIT_BASIC_APP_VERSION;
    m_dev_ctx.basic_attr.stack_version = SENSOR_INIT_BASIC_STACK_VERSION;
    m_dev_ctx.basic_attr.hw_version    = SENSOR_INIT_BASIC_HW_VERSION;

    /* Use ZB_ZCL_SET_STRING_VAL to set strings, because the first byte should contain string length without trailing zero.*/
    ZB_ZCL_SET_STRING_VAL(m_dev_ctx.basic_attr.mf_name,
                          SENSOR_INIT_BASIC_MANUF_NAME,
                          ZB_ZCL_STRING_CONST_SIZE(SENSOR_INIT_BASIC_MANUF_NAME));

    ZB_ZCL_SET_STRING_VAL(m_dev_ctx.basic_attr.model_id,
                          SENSOR_INIT_BASIC_MODEL_ID,
                          ZB_ZCL_STRING_CONST_SIZE(SENSOR_INIT_BASIC_MODEL_ID));

    ZB_ZCL_SET_STRING_VAL(m_dev_ctx.basic_attr.date_code,
                          SENSOR_INIT_BASIC_DATE_CODE,
                          ZB_ZCL_STRING_CONST_SIZE(SENSOR_INIT_BASIC_DATE_CODE));

    m_dev_ctx.basic_attr.power_source = SENSOR_INIT_BASIC_POWER_SOURCE;

    ZB_ZCL_SET_STRING_VAL(m_dev_ctx.basic_attr.location_id,
                          SENSOR_INIT_BASIC_LOCATION_DESC,
                          ZB_ZCL_STRING_CONST_SIZE(SENSOR_INIT_BASIC_LOCATION_DESC));


    m_dev_ctx.basic_attr.ph_env = SENSOR_INIT_BASIC_PH_ENV;

    /* Identify cluster attributes data */
    m_dev_ctx.identify_attr.identify_time        = ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE;

    /* Temperature measurement cluster attributes data */
    m_dev_ctx.temp_attr.measure_value            = ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_UNKNOWN;
    m_dev_ctx.temp_attr.min_measure_value        = ZB_ZCL_ATTR_TEMP_MEASUREMENT_MIN_VALUE_MIN_VALUE;
    m_dev_ctx.temp_attr.max_measure_value        = ZB_ZCL_ATTR_TEMP_MEASUREMENT_MAX_VALUE_MAX_VALUE;
    m_dev_ctx.temp_attr.tolerance                = ZB_ZCL_ATTR_TEMP_MEASUREMENT_TOLERANCE_MAX_VALUE;

    /* Huminity measurement cluster attributes data */
    m_dev_ctx.humi_attr.measure_value            = ZB_ZCL_ATTR_HUMI_MEASUREMENT_VALUE_UNKNOWN;
    m_dev_ctx.humi_attr.min_measure_value        = ZB_ZCL_ATTR_HUMI_MEASUREMENT_MIN_VALUE_MIN_VALUE;
    m_dev_ctx.humi_attr.max_measure_value        = ZB_ZCL_ATTR_HUMI_MEASUREMENT_MAX_VALUE_MAX_VALUE;
    m_dev_ctx.humi_attr.tolerance                = ZB_ZCL_ATTR_HUMI_MEASUREMENT_TOLERANCE_MAX_VALUE;
}

/* brief Callback scheduled from @ref temperature_measurement_timer_handler*/
void update_temperature_measurement_cb(zb_uint8_t param)
{
    UNUSED_PARAMETER(param);

    /* Just to show that we are in zigbee_main_task context, and locking is not required */
    ASSERT(xTaskGetCurrentTaskHandle() == m_zigbee_main_task_handle);

    zb_int16_t new_temp_value;

    vTaskSuspendAll();
    new_temp_value = m_update_temperature_measurement_ctx.measured_value;
    UNUSED_RETURN_VALUE(xTaskResumeAll());

    zb_zcl_status_t zcl_status;

    /* Note: As we are in zigbee_main_task context it is perfectly correct to update without mutex locking */
    zcl_status = zb_zcl_set_attr_val(MULTI_SENSOR_ENDPOINT,
                                     ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
                                     ZB_ZCL_CLUSTER_SERVER_ROLE,
                                     ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
                                     (zb_uint8_t *)&new_temp_value,
                                     ZB_FALSE);

    if (zcl_status != ZB_ZCL_STATUS_SUCCESS)
    {
        NRF_LOG_INFO("Set temperature value fail. zcl_status: %d", zcl_status);
    }
}

/* brief Function for handling nrf app timer*/
void temperature_measurement_timer_handler(void * context)
{
    UNUSED_PARAMETER(context);

    /* Just to show that we are NOT in zigbee_main_task context */
    ASSERT(xTaskGetCurrentTaskHandle() != m_zigbee_main_task_handle);

    SHT3x_read_temperature(&temperature_values);
   
    /* Get new temperature measured value */
    zb_int16_t new_temp_value = = temperature_values;
    
    vTaskSuspendAll();
     m_update_temperature_measurement_ctx.measured_value = new_temp_value;
     NRF_LOG_INFO("New temperature: %d", m_update_temperature_measurement_ctx.measured_value);
    UNUSED_RETURN_VALUE(xTaskResumeAll());

    zb_ret_t zb_ret;

    /* Note: ZB_SCHEDULE_CALLBACK is thread safe by exception, conversely to most ZBOSS API */
    zb_ret = ZB_SCHEDULE_CALLBACK(update_temperature_measurement_cb, 0U);
    if (zb_ret != RET_OK)
    {
        NRF_LOG_ERROR("Temperature sample lost.");
    }
}

/* brief  Task performing huminity measurement.*/
static void huminity_measurement_task(void *pvParam)
{
    UNUSED_PARAMETER(context);

    /* Just to show that we are NOT in zigbee_main_task context */
    ASSERT(xTaskGetCurrentTaskHandle() != m_zigbee_main_task_handle);

    SHT3x_read_humidity(&humidity_values);
   
    /* Get new humidity measured value */
    zb_int16_t new_temp_value = = humidity_values;
    
    vTaskSuspendAll();
     m_update_humidity_measurement_ctx.measured_value = new_humi_value;
     NRF_LOG_INFO("New humidity: %d", m_update_humidity_measurement_ctx.measured_value);
    UNUSED_RETURN_VALUE(xTaskResumeAll());

    zb_ret_t zb_ret;

    /* Note: ZB_SCHEDULE_CALLBACK is thread safe by exception, conversely to most ZBOSS API */
    zb_ret = ZB_SCHEDULE_CALLBACK(update_humidity_measurement_cb, 0U);
    if (zb_ret != RET_OK)
    {
        NRF_LOG_ERROR("Humidity sample lost.");
    }
}
         

/* brief FreeRTOS hook function called from idle task */
void vApplicationIdleHook(void)
{
    /* No task is running, just idle on lowest priority, so we can lower power usage */
    __WFE();
}

/* brief  FreeRTOS hook function called when stack overflow has been detected
/* note   See FreeRTOS API*/
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    NRF_LOG_ERROR("vApplicationStackOverflowHook(%x,\"%s\")", xTask, pcTaskName);
    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
}

/* brief Function which tries to sleep down the MCU
/* note  This function overrides implementation found in zb_nrf52840_common.c*/
zb_void_t zb_osif_go_idle(zb_void_t)
{
    /* Intentionally empty implementation */
}
