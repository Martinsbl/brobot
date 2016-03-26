/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_app_template_main main.c
 * @{
 * @ingroup ble_sdk_app_template
 * @brief Template project main file.
 *
 * This file contains a template for creating a new application. It has the code necessary to wakeup
 * from button, advertise, get a connection restart advertising on disconnect and if no new
 * connection created go back to system-off mode.
 * It can easily be used as a starting point for creating a new application, the comments identified
 * with 'YOUR_JOB' indicates where and how you can customize.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "boards.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "pstorage.h"
#include "bsp.h"
#include "nrf_gpio.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "nrf_delay.h"
#include "nrf_drv_config.h"
#include "qik_drv_mc.h"
#include "my_debug.h"
#include "SEGGER_RTT.h"
#include "ble_qik_cfg.h"
#include "msgs.h"
#include "ble_qik_motor.h"
#include "ble_bas.h"
#include "ble_brobot_error.h"
#include "hc_sr04.h"

#define QIK_TX_PIN_NUMBER   3
#define QIK_RX_PIN_NUMBER   4
#define QIK_RESET_PIN       28
#define QIK_ERR_PIN_NUMBER  29

#define IS_SRVC_CHANGED_CHARACT_PRESENT  1                                          /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define CENTRAL_LINK_COUNT               0                                          /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT            1                                          /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEVICE_NAME                      "Brobot"                          /**< Name of device. Will be included in the advertising data. */
#define APP_ADV_INTERVAL                 300                                        /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS       0                                        /**< The advertising timeout in units of seconds. */

#define APP_TIMER_PRESCALER              0                                          /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE          4                                          /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL                BLE_GAP_CP_MIN_CONN_INTVL_MIN//MSEC_TO_UNITS(100, UNIT_1_25_MS)           /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL                BLE_GAP_CP_MAX_CONN_INTVL_MIN//MSEC_TO_UNITS(200, UNIT_1_25_MS)           /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                    0                                          /**< Slave latency. */
#define CONN_SUP_TIMEOUT                 MSEC_TO_UNITS(4000, UNIT_10_MS)            /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY    APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)/**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT     3                                          /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                   1                                          /**< Perform bonding. */
#define SEC_PARAM_MITM                   0                                          /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                   0                                          /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS               0                                          /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES        BLE_GAP_IO_CAPS_NONE                       /**< No I/O capabilities. */
#define SEC_PARAM_OOB                    0                                          /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE           7                                          /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE           16                                         /**< Maximum encryption key size. */

static uint16_t                          m_conn_handle = BLE_CONN_HANDLE_INVALID;   /**< Handle of the current connection. */

ble_bas_t               m_bas;
ble_brobot_error_t      m_brobot_error;
qik_drv_mc_t            m_qik_motor_control;
ble_qik_cfg_t           m_ble_qik_cfg;
ble_qik_motor_t         m_ble_qik_motor;

volatile bool do_motor_measurements = false;

#define TIMER_INTERVAL_BATTERY APP_TIMER_TICKS(500, 0)
#define TIMER_INTERVAL APP_TIMER_TICKS(200, 0)

APP_TIMER_DEF(m_led_timer_id);
APP_TIMER_DEF(m_battery_timer_id);


#define TIMER_INTERVAL_MOTOR_SAFETY APP_TIMER_TICKS(300, 0)
APP_TIMER_DEF(m_motor_safety_timer_id);
bool turn_off_motors = false;


void timer_motor_safety_handler(void * p_context)
{    
    m_qik_motor_control.speed.m0_speed = 0;
    m_qik_motor_control.speed.m1_speed = 0;
    qik_drv_set_motor_speed(&m_qik_motor_control);
}



void timer_battery_handler(void * p_context)
{
    ble_bas_trigger_battery_level_update(&m_bas);
}

void timer_led_handler(void * p_context)
{
    static uint8_t led_timer = 0;
    static uint8_t measurement_timer = 0;
    
    if(m_conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        if(led_timer < 1)
            nrf_gpio_pin_clear(LED_1);
        else
            nrf_gpio_pin_set(LED_1);
    }
    else
    {
        nrf_gpio_pin_clear(LED_1);
    }
    
    led_timer++;
    if(led_timer > 10) led_timer = 0;
    
    measurement_timer++;
    if(measurement_timer > 5) 
    {    
        measurement_timer = 0;
        do_motor_measurements = true;
    }
    do_motor_measurements = true;
}

                                   
/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    SEGGER_RTT_printf(0, "SD Assert Line: %s, file: %s\n", line_num, p_file_name);
    while(1)
    {
        nrf_delay_ms(50);
        nrf_gpio_pin_toggle(LED_4);
    }
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{

    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    uint32_t err_code;
    err_code = app_timer_create(&m_led_timer_id, APP_TIMER_MODE_REPEATED, timer_led_handler);
    APP_ERROR_CHECK(err_code);
    
    err_code = app_timer_create(&m_battery_timer_id, APP_TIMER_MODE_REPEATED, timer_battery_handler);
    APP_ERROR_CHECK(err_code);
    
    
    err_code = app_timer_create(&m_motor_safety_timer_id, APP_TIMER_MODE_SINGLE_SHOT, timer_motor_safety_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    ble_qik_cfg_service_init(&m_qik_motor_control, &m_ble_qik_cfg);
    ble_qik_motor_service_init(&m_ble_qik_motor);
    ble_bas_init(&m_bas);
    ble_brobot_error_service_init(&m_brobot_error);
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting timers.
*/
static void application_timers_start(void)
{
    uint32_t err_code;
    err_code = app_timer_start(m_led_timer_id, TIMER_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
    
    err_code = app_timer_start(m_battery_timer_id, TIMER_INTERVAL_BATTERY, NULL);
    APP_ERROR_CHECK(err_code);   
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
     // Go to system-off mode (this function will not return; wakeup will cause a reset).
    uint32_t err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;
    if(p_ble_evt->evt.gatts_evt.params.write.handle != m_ble_qik_motor.qik_speed_char_handles.value_handle)
        print_evt(p_ble_evt);
    
    switch (p_ble_evt->header.evt_id)
            {
        case BLE_GAP_EVT_CONNECTED:
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;
        
        case BLE_GATTS_EVT_WRITE:
            if(p_ble_evt->evt.gatts_evt.params.write.handle == m_ble_qik_motor.qik_speed_char_handles.value_handle)
            {
                err_code = app_timer_stop(m_motor_safety_timer_id);
                APP_ERROR_CHECK(err_code);
                err_code = app_timer_start(m_motor_safety_timer_id, TIMER_INTERVAL_MOTOR_SAFETY, NULL);
                APP_ERROR_CHECK(err_code);
                nrf_gpio_pin_toggle(LED_4);
            }
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
    
    ble_qik_cfg_on_ble_evt(&m_qik_motor_control, &m_ble_qik_cfg, p_ble_evt);
    ble_qik_motor_on_ble_evt(&m_qik_motor_control, &m_ble_qik_motor, p_ble_evt);
    ble_bas_on_ble_evt(&m_bas, p_ble_evt);
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
    ble_advertising_on_sys_evt(sys_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
    
    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);
    
    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);
    
    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void gpio_init(bool * p_erase_bonds)
{
    nrf_gpio_range_cfg_output(LED_START, LED_STOP);
    nrf_gpio_pin_set(LED_1);
    nrf_gpio_pin_set(LED_2);
    nrf_gpio_pin_set(LED_3);
    nrf_gpio_pin_set(LED_4);

    //*p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for the Power manager.
 */
//static void power_manage(void)
//{
//    uint32_t err_code = sd_app_evt_wait();
//    APP_ERROR_CHECK(err_code);
//}

static void qik_drv_init(void)
{
    uint32_t err_code;      
    err_code = qik_drv_setup(&m_qik_motor_control, UART_BAUDRATE_BAUDRATE_Baud38400, QIK_RX_PIN_NUMBER, QIK_TX_PIN_NUMBER, QIK_ERR_PIN_NUMBER, QIK_RESET_PIN);
    MY_APP_ERROR_CHECK(err_code);
}

/**@brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code;
    bool erase_bonds;
    
    SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_WHITE);
    MY_DEBUG_PRINTF("\033[2J\033[;HBrobot! Code Compiled @ %s\n", __TIME__);
    nrf_delay_ms(400);
    
    qik_drv_init();
    
    qik_drv_set_parameter(QIK_PARAM_SERIAL_TIMEOUT, 0); // FAIL SAFE! TURNING OFF SERIAL TIMEOUT
    
    qik_drv_print_config_parameters(&m_qik_motor_control);

    // Initialize.
    timers_init();
    gpio_init(&erase_bonds);
    ble_stack_init();
    gap_params_init();
    advertising_init();
    services_init();
    conn_params_init();

    // Start execution.
    application_timers_start();
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);

    // Enter main loop.
    for (;;)
    {
        //power_manage();
    }
}

/**
 * @}
 */
