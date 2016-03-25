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

/** @file
 *
 * @defgroup ble_sdk_srv_bas Battery Service
 * @{
 * @ingroup ble_sdk_srv
 * @brief Battery Service module.
 *
 * @details This module implements the Battery Service with the Battery Level characteristic.
 *          During initialization it adds the Battery Service and Battery Level characteristic
 *          to the BLE stack database. Optionally it can also add a Report Reference descriptor
 *          to the Battery Level characteristic (used when including the Battery Service in
 *          the HID service).
 *
 *          If specified, the module will support notification of the Battery Level characteristic
 *          through the ble_bas_battery_level_update() function.
 *          If an event handler is supplied by the application, the Battery Service will
 *          generate Battery Service events to the application.
 *
 * @note The application must propagate BLE stack events to the Battery Service module by calling
 *       ble_bas_on_ble_evt() from the @ref softdevice_handler callback.
 *
 * @note Attention! 
 *  To maintain compliance with Nordic Semiconductor ASA Bluetooth profile 
 *  qualification listings, this section of source code must not be modified.
 */

#ifndef BLE_BAS_H__
#define BLE_BAS_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

/**@brief Battery Service structure. This contains various status information for the service. */
typedef struct
{
    uint16_t                      service_handle;                 /**< Handle of Battery Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t      battery_level_handles;          /**< Handles related to the Battery Level characteristic. */
    uint16_t                      conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */bool                          is_notification_supported;      /**< TRUE if notification of Battery Level is supported. */
}ble_bas_t;

/**@brief Function for initializing the Battery Service.
 *
 * @param[out]  p_bas       Battery Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_bas_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_bas_init(ble_bas_t * p_bas);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Battery Service.
 *
 * @note For the requirements in the BAS specification to be fulfilled,
 *       ble_bas_battery_level_update() must be called upon reconnection if the
 *       battery level has changed while the service has been disconnected from a bonded
 *       client.
 *
 * @param[in]   p_bas      Battery Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_bas_on_ble_evt(ble_bas_t * p_bas, ble_evt_t * p_ble_evt);

uint32_t ble_bas_trigger_battery_level_update(ble_bas_t * p_bas);

/**@brief Function for updating the battery level.
 *
 * @details The application calls this function after having performed a battery measurement. If
 *          notification has been enabled, the battery level characteristic is sent to the client.
 *
 * @note For the requirements in the BAS specification to be fulfilled,
 *       this function must be called upon reconnection if the battery level has changed
 *       while the service has been disconnected from a bonded client.
 *
 * @param[in]   p_bas          Battery Service structure.
 * @param[in]   battery_level  New battery measurement value (in percent of full capacity).
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_bas_battery_level_update(ble_bas_t * p_bas);

#endif // BLE_BAS_H__

/** @} */
