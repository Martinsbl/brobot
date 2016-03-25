#ifndef BLE_BROBOT_ERROR_H__
#define BLE_BROBOT_ERROR_H__

#include <stdint.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "qik_drv_mc.h"

#define BLE_UUID_QIK_BASE              			{0x23, 0xD1, 0x13, 0xEF, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00} // 128-bit base UUID
#define BLE_UUID_BROBOT_ERROR_SERVICE           0xEC0D // Just a random, but recognizable value
#define BLE_UUID_BROBOT_ERROR_CHAR        		0xC0DE // Just a random, but recognizable value


typedef struct
{
    uint16_t                    conn_handle;    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection).*/
    uint16_t                    service_handle; /**< Handle of Our Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t    brobot_error_char_handles;   /**< Handles related to the our new characteristic. */
}ble_brobot_error_t;

typedef struct 
{
    uint8_t err_type;
    uint8_t err_code;
}brobot_error_t;

/**@brief Function for handling BLE Stack events related to our service and characteristic.
 *
 */
void ble_brobot_error_on_ble_evt(ble_brobot_error_t * p_ble_brobot_error, ble_evt_t * p_ble_evt);

/**@brief Function for initializing our new service.
 *
 */
void ble_brobot_error_service_init(ble_brobot_error_t * p_ble_brobot_error);

/**@brief Function for updating and sending new characteristic values
 *
 * @details The application calls this function whenever our timer_timeout_handler triggers
 *
 */
void ble_brobot_error_measurements_update(ble_brobot_error_t * p_ble_brobot_error, brobot_error_t *p_brobot_error);

#endif  /* _ BLE_BROBOT_ERROR_H__ */
