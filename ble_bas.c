/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
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

/* Attention! 
*  To maintain compliance with Nordic Semiconductor ASA’s Bluetooth profile 
*  qualification listings, this section of source code must not be modified.
*/

#include "ble_bas.h"
#include <string.h>
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "my_debug.h"
#include "nrf_drv_saadc.h"
#include "SEGGER_RTT.h"
#include "app_util_platform.h"

#define BATTERY_PIN             30

nrf_saadc_value_t battery_level;
int16_t battery_level_average = 715;

ble_bas_t * m_ble_bas;


/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_bas       Battery Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_bas_t * p_bas, ble_evt_t * p_ble_evt)
{
    p_bas->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_bas       Battery Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_bas_t * p_bas, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_bas->conn_handle = BLE_CONN_HANDLE_INVALID;
}

void ble_bas_on_ble_evt(ble_bas_t * p_bas, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_bas, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_bas, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for adding the Battery Level characteristic.
 *
 * @param[in]   p_bas        Battery Service structure.
 * @param[in]   p_bas_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t battery_level_char_add(ble_bas_t * p_bas)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    // authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.notify = 1;
    char_md.p_cccd_md         = &cccd_md;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_BATTERY_LEVEL_CHAR);

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;

//    uint16_t battery_level = nrf_adc_convert_single(BATTERY_PIN);
//    for(int i = 0; i < 32; i++)
//    {
//        battery_level = ((15 * battery_level) + nrf_adc_convert_single(BATTERY_PIN)) / 16;
//    }

    uint32_t err_code = nrf_drv_saadc_sample();
    MY_APP_ERROR_CHECK_NON_BLOCKING(err_code);  
    
    

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint16_t);
    attr_char_value.max_len   = sizeof(uint16_t);
    attr_char_value.p_value   = (uint8_t*)&battery_level;

    return sd_ble_gatts_characteristic_add(p_bas->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_bas->battery_level_handles);
}

uint32_t ble_bas_trigger_battery_level_update(ble_bas_t * p_bas)
{
    uint32_t err_code;
    m_ble_bas = p_bas;
    err_code = nrf_drv_saadc_buffer_convert(&battery_level, 1);
    MY_APP_ERROR_CHECK_NON_BLOCKING(err_code);  
    err_code = nrf_drv_saadc_sample();
    MY_APP_ERROR_CHECK_NON_BLOCKING(err_code);  
    return err_code;
}

void saadc_handler(nrf_drv_saadc_evt_t const * p_event)
{
    
    battery_level_average = ((3 * battery_level_average) + battery_level) / 4;
    SEGGER_RTT_printf(0, "adc %d\n", battery_level_average);
    static uint8_t i = 0;
    if(i++ >= 4)
    {
        ble_bas_battery_level_update(m_ble_bas);
        i = 0;
    }      
}

static void saadc_init(void)
{
    uint32_t   err_code;
    
    nrf_drv_saadc_config_t adc_config = NRF_DRV_SAADC_DEFAULT_CONFIG;
    nrf_drv_saadc_init(&adc_config, saadc_handler);
    
    nrf_saadc_channel_config_t channel_config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN6);
    channel_config.gain = NRF_SAADC_GAIN1_6;
    channel_config.reference = NRF_SAADC_REFERENCE_INTERNAL; // 0.6
    //Initialization and enabling of channel 0 to use analog input 6.
    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    MY_APP_ERROR_CHECK_NON_BLOCKING(err_code);
    
    err_code = nrf_drv_saadc_buffer_convert(&battery_level, 1);
    MY_APP_ERROR_CHECK_NON_BLOCKING(err_code);  
}
    

uint32_t ble_bas_init(ble_bas_t * p_bas)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_bas->conn_handle               = BLE_CONN_HANDLE_INVALID;
    
    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_BATTERY_SERVICE);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_bas->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    saadc_init();

    // Add battery level characteristic
    return battery_level_char_add(p_bas);
}

uint32_t ble_bas_battery_level_update(ble_bas_t * p_bas)
{
    uint32_t err_code;
    static uint16_t battery_level_last = 0;

    if (battery_level_average != battery_level_last)
    {
               // Send value if connected and notifying.
        if (p_bas->conn_handle != BLE_CONN_HANDLE_INVALID)
        {
            ble_gatts_hvx_params_t hvx_params;
            uint16_t len = sizeof(battery_level_average);

            memset(&hvx_params, 0, sizeof(hvx_params));

            hvx_params.handle = p_bas->battery_level_handles.value_handle;
            hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset = 0;
            hvx_params.p_len  = &len;
            hvx_params.p_data = (uint8_t *)&battery_level_average;

            err_code = sd_ble_gatts_hvx(p_bas->conn_handle, &hvx_params);
            
            if((err_code != BLE_GATTS_EVT_SYS_ATTR_MISSING) && (err_code != NRF_SUCCESS))
            {
                MY_APP_ERROR_CHECK_NON_BLOCKING(err_code);
            }
        }
    }
    
    battery_level_last = battery_level_average;

    return NRF_SUCCESS;
}
