#ifndef BLE_QIK_CFG_H__
#define BLE_QIK_CFG_H__

#include <stdint.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "qik_drv_mc.h"

#define BLE_UUID_QIK_BASE              {0x23, 0xD1, 0x13, 0xEF, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00} // 128-bit base UUID
#define BLE_UUID_QIK_CFG_SERVICE                0xB0B0 // Just a random, but recognizable value
#define BLE_UUID_QIK_CFG_CHAR          0x05A5 // Just a random, but recognizable value


typedef struct
{
    uint16_t                    conn_handle;    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection).*/
    uint16_t                    service_handle; /**< Handle of Our Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t    qik_cfg_char_handles;   /**< Handles related to the our new characteristic. */
}ble_qik_cfg_t;

/**@brief Function for handling BLE Stack events related to our service and characteristic.
 *
 * @details Handles all events from the BLE stack of interest to Our Service.
 *
 * @param[in]   qik_drv_mc_t           Qik global structure.
 * @param[in]   p_ble_qik_cfg       Qik ble structure.
 * @param[in]   p_ble_evt       Event received from the BLE stack.
 */
void ble_qik_cfg_on_ble_evt(qik_drv_mc_t * p_qik, ble_qik_cfg_t * p_ble_qik_cfg, ble_evt_t * p_ble_evt);

/**@brief Function for initializing our new service.
 *
 * @param[in]   qik_drv_mc_t           Qik global structure.
 * @param[in]   p_ble_qik_cfg       Qik ble structure.
 */
void ble_qik_cfg_service_init(qik_drv_mc_t * p_qik, ble_qik_cfg_t * p_ble_qik_cfg);

/**@brief Function for updating and sending new characteristic values
 *
 * @details The application calls this function whenever our timer_timeout_handler triggers
 *
 * @param[in]   qik_drv_mc_t           Qik global structure.     
 * @param[in]   p_ble_qik_cfg       Qik ble structure.
 */
void ble_qik_cfg_config_update(qik_drv_mc_t * p_qik, ble_qik_cfg_t *p_ble_qik_cfg);

#endif  /* _ BLE_QIK_CFG_H__ */
