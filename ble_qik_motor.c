#include <stdint.h>
#include <string.h>
#include "nrf_gpio.h"
#include "ble_qik_motor.h"
#include "ble_srv_common.h"
#include "app_error.h"
#include "qik_drv_mc.h"
#include "my_debug.h"


void ble_qik_motor_on_ble_evt(qik_drv_mc_t * p_qik_motor_control, ble_qik_motor_t * p_ble_qik_motor, ble_evt_t * p_ble_evt)
{
    // OUR_JOB: Step 3.D Implement switch case handling BLE events related to our service. 
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            p_ble_qik_motor->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
        case BLE_GAP_EVT_DISCONNECTED:
            p_ble_qik_motor->conn_handle = BLE_CONN_HANDLE_INVALID;
            break;
        case BLE_GATTS_EVT_WRITE:
            if(p_ble_evt->evt.gatts_evt.params.write.handle == p_ble_qik_motor->qik_speed_char_handles.value_handle)
            {
                //MY_DEBUG_PRINTF("Setting speed to %d/%d\n", p_ble_evt->evt.gatts_evt.params.write.data[0], p_ble_evt->evt.gatts_evt.params.write.data[1]);
                p_qik_motor_control->speed.m0_speed = p_ble_evt->evt.gatts_evt.params.write.data[0];
                p_qik_motor_control->speed.m1_speed = p_ble_evt->evt.gatts_evt.params.write.data[1];
                qik_drv_set_motor_speed(p_qik_motor_control);
            }

            break;
        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for adding characterstic
 *
 * @param[in]   p_ble_qik_motor        Motor Service structure.
 *
 */
static uint32_t measurements_char_add(ble_qik_motor_t * p_ble_qik_motor)
{
    uint32_t   err_code = 0; // Variable to hold return codes from library and softdevice functions
    
    ble_uuid_t          char_uuid;
    ble_uuid128_t       base_uuid = BLE_UUID_QIK_BASE;
    char_uuid.uuid = BLE_UUID_QIK_MEASUREMENTS_CHAR;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
    APP_ERROR_CHECK(err_code);   

    
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.write = 0;
    char_md.char_props.read  = 1;
    char_md.char_props.notify = 1;
    
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md));
    attr_md.vloc = BLE_GATTS_VLOC_STACK; 
    
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    
    const uint8_t char_length = sizeof(qik_drv_m_current_t);
    
    ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid      = &char_uuid;
    attr_char_value.p_attr_md   = &attr_md;
    attr_char_value.max_len     = char_length;
    attr_char_value.init_len    = char_length;
    uint8_t init_value[char_length] = {0};
    attr_char_value.p_value     = init_value;

    err_code = sd_ble_gatts_characteristic_add(p_ble_qik_motor->service_handle,
                                       &char_md,
                                       &attr_char_value,
                                       &p_ble_qik_motor->qik_measurements_char_handles);
    APP_ERROR_CHECK(err_code);   

    return NRF_SUCCESS;
}

/**@brief Function for adding characterstic
 *
 * @param[in]   p_ble_qik_motor        Motor Service structure.
 *
 */
static uint32_t speed_char_add(ble_qik_motor_t * p_ble_qik_motor)
{
    uint32_t   err_code = 0; // Variable to hold return codes from library and softdevice functions
    
    ble_uuid_t          char_uuid;
    ble_uuid128_t       base_uuid = BLE_UUID_QIK_BASE;
    char_uuid.uuid = BLE_UUID_QIK_SPEED_CHAR;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
    APP_ERROR_CHECK(err_code);   

    
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.write = 1;
    
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md));
    attr_md.vloc = BLE_GATTS_VLOC_STACK; 
    
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    
    ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid      = &char_uuid;
    attr_char_value.p_attr_md   = &attr_md;
    attr_char_value.max_len     = sizeof(qik_drv_speed_t);
    attr_char_value.init_len    = sizeof(qik_drv_speed_t);
    attr_char_value.p_value     = NULL;

    err_code = sd_ble_gatts_characteristic_add(p_ble_qik_motor->service_handle,
                                       &char_md,
                                       &attr_char_value,
                                       &p_ble_qik_motor->qik_speed_char_handles);
    APP_ERROR_CHECK(err_code);   

    return NRF_SUCCESS;
}


/**@brief Function for initiating our new service.
 *
 * @param[in]   p_ble_qik_motor        
 *
 */
void ble_qik_motor_service_init(ble_qik_motor_t * p_ble_qik_motor)
{
    uint32_t   err_code;

    ble_uuid_t        service_uuid;
    ble_uuid128_t     base_uuid = BLE_UUID_QIK_BASE;
    service_uuid.uuid = BLE_UUID_QIK_MOTOR_SERVICE;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);
    APP_ERROR_CHECK(err_code);    
    
    p_ble_qik_motor->conn_handle = BLE_CONN_HANDLE_INVALID;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &service_uuid,
                                        &p_ble_qik_motor->service_handle);
    
    APP_ERROR_CHECK(err_code);
    
    speed_char_add(p_ble_qik_motor);
    measurements_char_add(p_ble_qik_motor);
}


void ble_qik_motor_measurements_update(qik_drv_mc_t * p_qik, ble_qik_motor_t *p_ble_qik_motor)
{
    uint32_t err_code;
    if (p_ble_qik_motor->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        uint16_t         len = sizeof(qik_drv_m_current_t);
        ble_gatts_hvx_params_t hvx_params;
        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_ble_qik_motor->qik_measurements_char_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &len;
        hvx_params.p_data = (uint8_t*)&p_qik->measured_current;  

        err_code = sd_ble_gatts_hvx(p_ble_qik_motor->conn_handle, &hvx_params);
        if(err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
        {
            MY_APP_ERROR_CHECK(err_code);
        }
    }    
}
