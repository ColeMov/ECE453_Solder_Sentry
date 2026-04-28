/**
 * @file task_ble.h
 * @brief BLE notification task using Nordic UART Service (NUS).
 *
 * Exposes a BLE GATT server that advertises as "ECE453" and sends
 * console log messages as BLE notifications on the NUS TX characteristic.
 *
 * Phone app to use: "nRF Toolbox" (Nordic Semiconductor) or "LightBlue"
 * - Connect to "ECE453"
 * - Enable notifications on the UART TX characteristic
 *     UUID: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E
 *
 * ============================================================================
 * SETUP — must be done before this code will compile
 * ============================================================================
 * 1. Run:  make getlibs
 *    This downloads the bless library into mtb_shared/.
 *
 * 2. Open ModusToolbox -> Tools -> BT Configurator, create a design with:
 *
 *    [Generic Access Profile]       (keep defaults)
 *    [Generic Attribute Profile]    (keep defaults)
 *    [Custom Service]  — name: "Nordic UART Service"
 *        Service UUID (128-bit): 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 *
 *        Characteristic: "NUS TX"
 *            UUID (128-bit): 6E400003-B5A3-F393-E0A9-E50E24DCCA9E
 *            Properties: Notify
 *            Permissions: Read
 *            Add descriptor: Client Characteristic Configuration (CCCD)
 *
 *        Characteristic: "NUS RX"
 *            UUID (128-bit): 6E400002-B5A3-F393-E0A9-E50E24DCCA9E
 *            Properties: Write, Write Without Response
 *            Permissions: Write
 *
 *    GAP -> Advertising -> Device Name: "ECE453"
 *
 * 3. Save. The configurator writes cy_ble_config.h and cy_ble_config.c into
 *    the project's GeneratedSource folder and rebuilds the GATT database.
 *
 * 4. Update BLE_NUS_SERVICE_INDEX and BLE_NUS_TX_CHAR_INDEX below if the
 *    NUS service is not the first custom service in your design (usually 0).
 * ============================================================================
 */

#ifndef __TASK_BLE_H__
#define __TASK_BLE_H__

#include "main.h"

/* ModusToolbox sets COMPONENT_BLESS when BLE middleware is enabled. */
#ifdef COMPONENT_BLESS

#include "cycfg_ble.h"
#include "cy_ble.h"
#include "cy_ble_custom.h"

/* NUS service is first custom service. TX (notify) char comes first
 * in the BT Configurator design, RX (write/write-w/o-resp) is second. */
#define BLE_NUS_SERVICE_INDEX   (0u)
#define BLE_NUS_TX_CHAR_INDEX   (0u)
#define BLE_NUS_RX_CHAR_INDEX   (1u)

/* Max bytes per BLE notification (default ATT MTU minus 3 bytes overhead) */
#define BLE_NOTIF_MAX_LEN       (20u)

/* Must stay in sync with DEBUG_MESSAGE_MAX_LEN in task_console.h */
#define BLE_TX_ITEM_MSG_LEN     (100u)

/* Queue item sent from task_console -> task_ble */
typedef struct {
    char msg[BLE_TX_ITEM_MSG_LEN];
} ble_tx_item_t;

/* Push ble_tx_item_t into this queue to send a BLE notification */
extern QueueHandle_t q_ble_tx;
extern volatile uint32_t g_ble_diag_state;

/**
 * @brief Create the BLE TX queue and start the BLE task.
 *        Call from main() before vTaskStartScheduler().
 */
void task_ble_init(void);

/* Restart fast advertising — used as the long-press pairing trigger. */
void task_ble_force_pairing_mode(void);

#endif /* COMPONENT_BLESS */

#endif /* __TASK_BLE_H__ */
