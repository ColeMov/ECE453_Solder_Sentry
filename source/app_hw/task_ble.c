/**
 * @file task_ble.c
 * @brief BLE notification task (legacy PSoC 6 BLESS stack).
 */

#include "task_ble.h"

#ifdef COMPONENT_BLESS

#include "task_console.h"
#include <string.h>

#define BLE_TX_QUEUE_LEN (8u)
#ifndef CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX
#define CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX (0u)
#endif

/* Pointer helpers into generated custom service tables */
#define NUS_SERV()    (cy_ble_customsConfigPtr->attrInfo[BLE_NUS_SERVICE_INDEX])
#define NUS_TX_CHAR() (NUS_SERV().customServInfo[BLE_NUS_TX_CHAR_INDEX])

QueueHandle_t q_ble_tx = NULL;
volatile uint32_t g_ble_diag_state = 0u;

static cy_stc_ble_conn_handle_t ble_conn_handle;
static bool ble_connected = false;
static bool notifications_enabled = false;
static bool ble_stack_on = false;

/* Generated stack params are mutable in cycfg_ble.c */
extern cy_stc_ble_stack_params_t stackParam;

static void task_ble(void *param);
static void ble_event_handler(uint32_t eventCode, void *eventParam);
static void ble_send_notification(const char *str);
static void ble_start_advertising_fast(void);

static void ble_start_advertising_fast(void)
{
    if (!ble_stack_on)
    {
        return;
    }

    cy_en_ble_api_result_t rslt = Cy_BLE_GAPP_StartAdvertisement(
        CY_BLE_ADVERTISING_FAST,
        CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);

    if ((rslt == CY_BLE_SUCCESS) || (rslt == CY_BLE_ERROR_INVALID_OPERATION))
    {
        task_print_info("BLE: advertising request (FAST), result=0x%04X", (unsigned)rslt);
    }
    else
    {
        task_print_error("BLE: advertising start failed (0x%04X)", (unsigned)rslt);
    }
}

static void ble_event_handler(uint32_t eventCode, void *eventParam)
{
    switch (eventCode)
    {
        case CY_BLE_EVT_STACK_ON:
            ble_stack_on = true;
            task_print_info("BLE: addr=%02X:%02X:%02X:%02X:%02X:%02X type=%u",
                            (unsigned)cy_ble_configPtr->deviceAddress->bdAddr[5],
                            (unsigned)cy_ble_configPtr->deviceAddress->bdAddr[4],
                            (unsigned)cy_ble_configPtr->deviceAddress->bdAddr[3],
                            (unsigned)cy_ble_configPtr->deviceAddress->bdAddr[2],
                            (unsigned)cy_ble_configPtr->deviceAddress->bdAddr[1],
                            (unsigned)cy_ble_configPtr->deviceAddress->bdAddr[0],
                            (unsigned)cy_ble_configPtr->deviceAddress->type);
            ble_start_advertising_fast();
            task_print_info("BLE: stack on, start advertising");
            break;

        case CY_BLE_EVT_GAP_DEVICE_CONNECTED:
            ble_conn_handle = *(cy_stc_ble_conn_handle_t *)eventParam;
            ble_connected = true;
            task_print_info("BLE: connected (bdHandle=%u)", ble_conn_handle.bdHandle);
            break;

        case CY_BLE_EVT_GAP_DEVICE_DISCONNECTED:
            ble_connected = false;
            notifications_enabled = false;
            ble_start_advertising_fast();
            task_print_info("BLE: disconnected, re-advertising");
            break;

        case CY_BLE_EVT_GAPP_ADVERTISEMENT_START_STOP:
            task_print_info("BLE: advertisement state=%u", (unsigned)Cy_BLE_GetAdvertisementState());
            break;

        case CY_BLE_EVT_TIMEOUT:
            task_print_warning("BLE: timeout event, retrying FAST advertising");
            ble_start_advertising_fast();
            break;

        case CY_BLE_EVT_GATTS_WRITE_REQ:
        {
            cy_stc_ble_gatts_write_cmd_req_param_t *wr =
                (cy_stc_ble_gatts_write_cmd_req_param_t *)eventParam;

            (void)Cy_BLE_GATTS_WriteAttributeValuePeer(&wr->connHandle, &wr->handleValPair);

            /* CCCD of the TX characteristic controls notifications */
            cy_ble_gatt_db_attr_handle_t cccdHandle = NUS_TX_CHAR().customServCharDesc[0];
            if (wr->handleValPair.attrHandle == cccdHandle)
            {
                uint16_t cccd = 0u;
                if (wr->handleValPair.value.len >= 2u)
                {
                    cccd = (uint16_t)wr->handleValPair.value.val[0] |
                           ((uint16_t)wr->handleValPair.value.val[1] << 8u);
                }
                notifications_enabled = (cccd == CY_BLE_GATT_CLI_CNFG_NOTIFICATION);
                task_print_info("BLE: notifications %s",
                                notifications_enabled ? "enabled" : "disabled");
            }

            (void)Cy_BLE_GATTS_WriteRsp(wr->connHandle);
            break;
        }

        default:
            break;
    }
}

static void ble_send_notification(const char *str)
{
    if ((str == NULL) || !ble_connected || !notifications_enabled)
    {
        return;
    }

    cy_ble_gatt_db_attr_handle_t txHandle = NUS_TX_CHAR().customServCharHandle;
    size_t total = strlen(str);
    size_t offset = 0u;

    while (offset < total)
    {
        uint8_t chunk[BLE_NOTIF_MAX_LEN];
        uint16_t len = (uint16_t)(total - offset);
        if (len > BLE_NOTIF_MAX_LEN)
        {
            len = BLE_NOTIF_MAX_LEN;
        }
        memcpy(chunk, str + offset, len);

        cy_stc_ble_gatt_handle_value_pair_t hvp = {
            .attrHandle = txHandle,
            .value = { .val = chunk, .len = len }
        };

        if (Cy_BLE_GATTS_SendNotification(&ble_conn_handle, &hvp) != CY_BLE_SUCCESS)
        {
            break;
        }
        offset += len;
    }
}

static void task_ble(void *param)
{
    (void)param;
    uint32_t heartbeat_ms = 0u;
    g_ble_diag_state = 12u;
    task_print_info("BLE: task started");

    for (;;)
    {
        Cy_BLE_ProcessEvents();
        heartbeat_ms += 10u;
        if (heartbeat_ms >= 2000u)
        {
            g_ble_diag_state = 13u;
            task_print_info("BLE[HB]: conn=%u notif=%u adv_state=%u stack_on=%u addr=%02X:%02X:%02X:%02X:%02X:%02X",
                            ble_connected ? 1u : 0u,
                            notifications_enabled ? 1u : 0u,
                            (unsigned)Cy_BLE_GetAdvertisementState(),
                            ble_stack_on ? 1u : 0u,
                            (unsigned)cy_ble_configPtr->deviceAddress->bdAddr[5],
                            (unsigned)cy_ble_configPtr->deviceAddress->bdAddr[4],
                            (unsigned)cy_ble_configPtr->deviceAddress->bdAddr[3],
                            (unsigned)cy_ble_configPtr->deviceAddress->bdAddr[2],
                            (unsigned)cy_ble_configPtr->deviceAddress->bdAddr[1],
                            (unsigned)cy_ble_configPtr->deviceAddress->bdAddr[0]);
            heartbeat_ms = 0u;

            /* Demo fallback: low-rate retry using generated BT Configurator settings only. */
            if (!ble_connected && ble_stack_on &&
                (Cy_BLE_GetAdvertisementState() == CY_BLE_ADV_STATE_STOPPED))
            {
                ble_start_advertising_fast();
            }
        }

        ble_tx_item_t item;
        if (xQueueReceive(q_ble_tx, &item, 0u) == pdPASS)
        {
            ble_send_notification(item.msg);
        }

        vTaskDelay(pdMS_TO_TICKS(10u));
    }
}

void task_ble_init(void)
{
    g_ble_diag_state = 1u;
    task_print_info("BLE: init called");

    /* Demo mode: disable LL privacy so scanner sees a stable public address. */
    stackParam.featureMask &= ~CY_BLE_LL_PRIVACY_FEATURE;

    q_ble_tx = xQueueCreate(BLE_TX_QUEUE_LEN, sizeof(ble_tx_item_t));
    if (q_ble_tx == NULL)
    {
        g_ble_diag_state = 0xF1u;
        task_print_error("BLE: queue create failed");
        return;
    }
    g_ble_diag_state = 2u;

    /* Must be registered before Cy_BLE_Init() in host mode. */
    Cy_BLE_RegisterEventCallback(ble_event_handler);

    cy_en_ble_api_result_t result = Cy_BLE_Init(&cy_ble_config);
    if (result != CY_BLE_SUCCESS)
    {
        g_ble_diag_state = 0xE1u;
        task_print_error("BLE: Cy_BLE_Init failed (0x%04X)", (unsigned)result);
        return;
    }
    g_ble_diag_state = 11u;

    result = Cy_BLE_Enable();
    if (result != CY_BLE_SUCCESS)
    {
        g_ble_diag_state = 0xE2u;
        task_print_error("BLE: Cy_BLE_Enable failed (0x%04X)", (unsigned)result);
        return;
    }
    g_ble_diag_state = 12u;
    task_print_info("BLE: stack enable requested");

    BaseType_t task_ok = xTaskCreate(task_ble,
                                     "BLE",
                                     6 * configMINIMAL_STACK_SIZE,
                                     NULL,
                                     configMAX_PRIORITIES - 5,
                                     NULL);
    if (task_ok != pdPASS)
    {
        g_ble_diag_state = 0xF2u;
        task_print_error("BLE: task create failed");
        return;
    }
    g_ble_diag_state = 3u;
}

#else

QueueHandle_t q_ble_tx = NULL;
void task_ble_init(void) {}

#endif
