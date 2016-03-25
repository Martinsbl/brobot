#include <stdint.h>
#include <string.h>
#include "nrf_gpio.h"
#include "ble_brobot_error.h"
#include "ble_srv_common.h"
#include "app_error.h"
#include "my_debug.h"



void ble_brobot_error_on_ble_evt(ble_brobot_error_t * p_ble_brobot_error, ble_evt_t * p_ble_evt)
{
    // OUR_JOB: Step 3.D Implement switch case handling BLE events related to our service. 
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            p_ble_brobot_error->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
        case BLE_GAP_EVT_DISCONNECTED:
            p_ble_brobot_error->conn_handle = BLE_CONN_HANDLE_INVALID;
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
static uint32_t char_brobot_error_add(ble_brobot_error_t * p_ble_brobot_error)
{
    uint32_t   err_code = 0; // Variable to hold return codes from library and softdevice functions
    
    ble_uuid_t          char_uuid;
    ble_uuid128_t       base_uuid = BLE_UUID_QIK_BASE;
    char_uuid.uuid = BLE_UUID_BROBOT_ERROR_CHAR;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
    APP_ERROR_CHECK(err_code);   

    
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read  = 1;
    char_md.char_props.notify = 1;
    
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md));
    attr_md.vloc = BLE_GATTS_VLOC_STACK; 
    
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    
    const uint8_t char_length = sizeof(brobot_error_t);
    
    ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid      = &char_uuid;
    attr_char_value.p_attr_md   = &attr_md;
    attr_char_value.max_len     = char_length;
    attr_char_value.init_len    = char_length;
    uint8_t init_value[char_length] = {0};
    attr_char_value.p_value     = init_value;

    err_code = sd_ble_gatts_characteristic_add(p_ble_brobot_error->service_handle,
                                       &char_md,
                                       &attr_char_value,
                                       &p_ble_brobot_error->brobot_error_char_handles);
    APP_ERROR_CHECK(err_code);   

    return NRF_SUCCESS;
}


/**@brief Function for initiating our new service.
 *
 * @param[in]   p_ble_qik_motor        
 *
 */
void ble_brobot_error_service_init(ble_brobot_error_t * p_ble_brobot_error)
{
    uint32_t   err_code;

    ble_uuid_t        service_uuid;
    ble_uuid128_t     base_uuid = BLE_UUID_QIK_BASE;
    service_uuid.uuid = BLE_UUID_BROBOT_ERROR_SERVICE;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);
    APP_ERROR_CHECK(err_code);    
    
    p_ble_brobot_error->conn_handle = BLE_CONN_HANDLE_INVALID;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &service_uuid,
                                        &p_ble_brobot_error->service_handle);
    
    APP_ERROR_CHECK(err_code);
    
    char_brobot_error_add(p_ble_brobot_error);
}


void ble_brobot_error_measurements_update(ble_brobot_error_t * p_ble_brobot_error, brobot_error_t *p_brobot_error)
{
    uint32_t err_code;
    if (p_ble_brobot_error->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        uint16_t         len = sizeof(brobot_error_t);
        ble_gatts_hvx_params_t hvx_params;
        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_ble_brobot_error->brobot_error_char_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &len;
        hvx_params.p_data = (uint8_t*)&p_brobot_error;  

        err_code = sd_ble_gatts_hvx(p_ble_brobot_error->conn_handle, &hvx_params);
        if(err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
        {
            MY_APP_ERROR_CHECK(err_code);
        }
    }    
}
