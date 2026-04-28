/**
 * @file task_ble.c
 * @brief BLE notification task (legacy PSoC 6 BLESS stack).
 */

#include "task_ble.h"

#ifdef COMPONENT_BLESS

#include "task_console.h"
#include "task_ir_sensor.h"
#include "task_audio.h"
#include "cy_ble_clk.h"
#include "cy_sysint.h"
#include <string.h>

/* BLESS interrupt is MANDATORY for single-CM4 mode. Without it the link
 * layer can't service CONNECT_REQ and central sees a 10 s timeout. */
static const cy_stc_sysint_t s_bless_isr_cfg =
{
    .intrSrc      = bless_interrupt_IRQn,
    .intrPriority = 1u,
};

static void bless_interrupt_isr(void)
{
    Cy_BLE_BlessIsrHandler();
}

/*
 * IR frame BLE protocol (binary, 9 packets per frame at 10 Hz):
 *
 *   Header packet (20 bytes):
 *     [0]    = 0xAA  (frame-start marker)
 *     [1]    = frame sequence number (wraps 0-255)
 *     [2:3]  = thermistor as int16, units = 0.0625 °C/LSB  (big-endian)
 *     [4:19] = 0x00 padding
 *
 *   Data packets (8 total, 20 bytes each):
 *     [0]    = 0xBB  (data marker)
 *     [1]    = chunk index (0-7)
 *     [2:17] = 8 pixels as int16, units = 0.25 °C/LSB  (big-endian)
 *     [18:19]= 0x00 padding
 *
 *   Pixel layout: chunk 0 = pixels[0..7], chunk 1 = pixels[8..15], ...
 *   Row-major: pixel[r][c] = pixels[r*8 + c]
 */
#define BLE_IR_FRAME_MARKER  (0xAAu)
#define BLE_IR_DATA_MARKER   (0xBBu)
#define BLE_IR_CHUNKS        (8u)
#define BLE_IR_PIX_PER_CHUNK (8u)
#define BLE_IR_PACKET_LEN    (20u)

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
static volatile bool ble_stack_busy = false;

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

void task_ble_force_pairing_mode(void)
{
    if (!ble_stack_on)
    {
        task_print_warning("BLE: pairing-mode requested before stack ready");
        return;
    }
    task_print_info("BLE: entering pairing mode (restart FAST adv)");
    /* INVALID_OPERATION returned if not currently advertising — ignore. */
    (void)Cy_BLE_GAPP_StopAdvertisement();
    ble_start_advertising_fast();
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

        case CY_BLE_EVT_STACK_BUSY_STATUS:
        {
            cy_stc_ble_l2cap_state_info_t *st =
                (cy_stc_ble_l2cap_state_info_t *)eventParam;
            ble_stack_busy = (st->flowState == CY_BLE_STACK_STATE_BUSY);
            break;
        }

        case CY_BLE_EVT_GATT_CONNECT_IND:
            /* This event carries the real cy_stc_ble_conn_handle_t needed
             * for all subsequent GATT operations (notifications, etc). */
            ble_conn_handle = *(cy_stc_ble_conn_handle_t *)eventParam;
            ble_connected = true;
            task_print_info("BLE: GATT connected");
            (void)task_audio_say("success_chime");
            break;

        case CY_BLE_EVT_GAP_DEVICE_CONNECTED:
        {
            /* Request fastest connection parameters macOS will accept:
             *   interval 7.5 ms .. 15 ms, latency 0, timeout 5 s */
            static cy_stc_ble_gap_conn_update_param_info_t connParams;
            connParams.connIntvMin   = 6u;    /* 6 * 1.25 ms = 7.5 ms */
            connParams.connIntvMax   = 12u;   /* 12 * 1.25 ms = 15 ms */
            connParams.connLatency   = 0u;
            connParams.supervisionTO = 500u;  /* 5 s */
            connParams.bdHandle      = ble_conn_handle.bdHandle;
            (void)Cy_BLE_L2CAP_LeConnectionParamUpdateRequest(&connParams);
            break;
        }

        case CY_BLE_EVT_GATT_DISCONNECT_IND:
            ble_connected = false;
            notifications_enabled = false;
            task_print_info("BLE: GATT disconnected");
            break;

        case CY_BLE_EVT_GAP_DEVICE_DISCONNECTED:
            ble_connected = false;
            notifications_enabled = false;
            task_print_info("BLE: disconnected (no auto re-advertise; long-press captouch to re-pair)");
            (void)task_audio_say("disconnected");
            break;

        case CY_BLE_EVT_GAPP_ADVERTISEMENT_START_STOP:
            task_print_info("BLE: advertisement state=%u", (unsigned)Cy_BLE_GetAdvertisementState());
            break;

        case CY_BLE_EVT_TIMEOUT:
            /* Don't auto-restart advertising — only the long-press captouch
             * trigger should put us back into pairing mode. */
            task_print_info("BLE: timeout (advertisement window ended)");
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

/* Send one 8x8 IR frame as 9 BLE notification packets. */
/* Send one notification directly via Cy_BLE_GATTS_Notification (bypasses
 * local GATT DB write — the generated DB has the TX char storage sized at
 * 0 bytes which would make SendNotification fail with INVALID_OPERATION). */
/* Strictly non-blocking: one attempt, returns immediately.
 * Caller is responsible for retrying on BUSY (next task loop iter). */
static cy_en_ble_api_result_t ble_tx_notify_raw(
    cy_ble_gatt_db_attr_handle_t hdl, const uint8_t *data, uint16_t len)
{
    if (ble_stack_busy)
    {
        return CY_BLE_ERROR_INVALID_OPERATION;
    }
    cy_stc_ble_gatts_handle_value_ntf_t p;
    p.connHandle              = ble_conn_handle;
    p.handleValPair.attrHandle = hdl;
    p.handleValPair.value.val = (uint8_t *)data;
    p.handleValPair.value.len = len;
    return Cy_BLE_GATTS_Notification(&p);
}

/* Diagnostic counters (cleared by task loop after print) */
static volatile uint32_t dbg_frames_attempted = 0u;
static volatile uint32_t dbg_frames_ok        = 0u;
static volatile uint32_t dbg_last_err         = 0u;
static volatile uint32_t dbg_ir_queue_hits    = 0u;

/* Stateful IR frame pump.
 *
 * Instead of trying to send all 9 packets in one call (which either blocks
 * on stack back-pressure or drops entire frames), we hold the current frame
 * in a buffer and send ONE packet per task-loop iteration. The BLE task's
 * 5 ms vTaskDelay then runs between every packet, giving all higher-priority
 * tasks (IR / TOF / captouch / IO expander) plenty of CPU.
 *
 * On BUSY we simply don't advance the state — next tick retries the same
 * packet. No busy-waiting, no frame drops under light back-pressure. */
typedef enum {
    IR_TX_IDLE = 0,     /* need to dequeue a frame */
    IR_TX_HEADER,       /* send header packet */
    IR_TX_DATA          /* send data packet [ir_tx_chunk] */
} ir_tx_state_t;

static ir_tx_state_t ir_tx_state = IR_TX_IDLE;
static amg8834_frame_t ir_tx_frame;
static uint8_t ir_tx_chunk = 0u;
static uint8_t ir_tx_seq = 0u;

static void ble_ir_pump(void)
{
    if (!ble_connected || !notifications_enabled)
    {
        ir_tx_state = IR_TX_IDLE;
        return;
    }

    cy_ble_gatt_db_attr_handle_t txHandle = NUS_TX_CHAR().customServCharHandle;
    uint8_t pkt[BLE_IR_PACKET_LEN];
    int16_t val;

    /* Try to start a new frame */
    if (ir_tx_state == IR_TX_IDLE)
    {
        if ((q_ir_sensor_frame == NULL) ||
            (xQueueReceive(q_ir_sensor_frame, &ir_tx_frame, 0u) != pdPASS))
        {
            return;
        }
        dbg_ir_queue_hits++;
        dbg_frames_attempted++;
        ir_tx_chunk = 0u;
        ir_tx_state = IR_TX_HEADER;
    }

    /* Send exactly one packet per call */
    if (ir_tx_state == IR_TX_HEADER)
    {
        memset(pkt, 0, BLE_IR_PACKET_LEN);
        pkt[0] = BLE_IR_FRAME_MARKER;
        pkt[1] = ir_tx_seq;
        val = (int16_t)(ir_tx_frame.thermistor_c / AMG8834_THERM_LSB_DEGC);
        pkt[2] = (uint8_t)((uint16_t)val >> 8u);
        pkt[3] = (uint8_t)((uint16_t)val & 0xFFu);

        cy_en_ble_api_result_t rc = ble_tx_notify_raw(txHandle, pkt, BLE_IR_PACKET_LEN);
        if (rc == CY_BLE_SUCCESS)
        {
            ir_tx_seq++;
            ir_tx_state = IR_TX_DATA;
            ir_tx_chunk = 0u;
        }
        else
        {
            dbg_last_err = (uint32_t)rc;
        }
        return;
    }

    if (ir_tx_state == IR_TX_DATA)
    {
        memset(pkt, 0, BLE_IR_PACKET_LEN);
        pkt[0] = BLE_IR_DATA_MARKER;
        pkt[1] = ir_tx_chunk;
        for (uint32_t p = 0u; p < BLE_IR_PIX_PER_CHUNK; p++)
        {
            uint32_t idx = ((uint32_t)ir_tx_chunk * BLE_IR_PIX_PER_CHUNK) + p;
            val = (int16_t)(ir_tx_frame.pixels_c[idx] / AMG8834_PIXEL_LSB_DEGC);
            pkt[2u + (p * 2u)]      = (uint8_t)((uint16_t)val >> 8u);
            pkt[2u + (p * 2u) + 1u] = (uint8_t)((uint16_t)val & 0xFFu);
        }

        cy_en_ble_api_result_t rc = ble_tx_notify_raw(txHandle, pkt, BLE_IR_PACKET_LEN);
        if (rc == CY_BLE_SUCCESS)
        {
            ir_tx_chunk++;
            if (ir_tx_chunk >= BLE_IR_CHUNKS)
            {
                ir_tx_state = IR_TX_IDLE;
                dbg_frames_ok++;
            }
        }
        else
        {
            dbg_last_err = (uint32_t)rc;
        }
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
        heartbeat_ms += 5u;
        /* Heartbeat retry of advertising removed on purpose: pairing mode
         * is now strictly long-press driven via task_ble_force_pairing_mode().
         * Once a connection drops, the board stays quiet until the user
         * holds the captouch pad. */
        (void)heartbeat_ms;

        /* Forward console log messages */
        ble_tx_item_t item;
        if (xQueueReceive(q_ble_tx, &item, 0u) == pdPASS)
        {
            ble_send_notification(item.msg);
        }

        /* Stream IR frames: one packet per iteration, back-pressure aware */
        ble_ir_pump();

        /* Diagnostic dump every 2 s while connected */
        static uint32_t dbg_ms = 0u;
        dbg_ms += 5u;
        if (dbg_ms >= 2000u)
        {
            dbg_ms = 0u;
            if (ble_connected)
            {
                unsigned qlen = (q_ir_sensor_frame != NULL)
                                ? (unsigned)uxQueueMessagesWaiting(q_ir_sensor_frame)
                                : 0u;
                task_print_info(
                    "BLE diag: conn=%d notif=%d ir_hits=%lu attempt=%lu ok=%lu lastErr=0x%06lX qlen=%u",
                    (int)ble_connected, (int)notifications_enabled,
                    (unsigned long)dbg_ir_queue_hits,
                    (unsigned long)dbg_frames_attempted,
                    (unsigned long)dbg_frames_ok,
                    (unsigned long)dbg_last_err,
                    qlen);
                dbg_ir_queue_hits = 0u;
                dbg_frames_attempted = 0u;
                dbg_frames_ok = 0u;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5u));
    }
}

void task_ble_init(void)
{
    g_ble_diag_state = 1u;
    task_print_info("BLE: init called");

    /* The custom BSP does NOT enable ALTHF / BLE ECO (32 MHz crystal) in its
     * clock config, so the BLE radio has no clock despite the stack reporting
     * adv_state=2. Start it manually with standard CY8CProto-063-BLE values.
     *
     * cLoad: 9.9 pF typical for the proto kit crystal (register value 32)
     *        formula: reg = (pF - 7.5) / 0.075 → (9.9 - 7.5) / 0.075 = 32
     * xtalStartUpTime: 1500 us typical (register value 48 = 1500/31.25)
     */
    cy_en_ble_eco_status_t eco_rslt =
        Cy_BLE_EcoConfigure(CY_BLE_BLESS_ECO_FREQ_32MHZ,
                            CY_BLE_SYS_ECO_CLK_DIV_1,
                            32u,    /* loadCap register value -> ~9.9 pF */
                            48u,    /* xtalStartUpTime register value -> 1500 us */
                            CY_BLE_ECO_VOLTAGE_REG_AUTO);
    if ((eco_rslt != CY_BLE_ECO_SUCCESS) &&
        (eco_rslt != CY_BLE_ECO_ALREADY_STARTED))
    {
        task_print_error("BLE: ECO start failed (0x%08lX)", (unsigned long)eco_rslt);
    }
    else
    {
        task_print_info("BLE: ECO started (rslt=0x%08lX)", (unsigned long)eco_rslt);
    }

    /* Demo mode: disable LL privacy (stable public address) and all the
     * flash-backed list / bonding / SC features. The custom BSP doesn't
     * wire up flash storage for any of these, and leaving them enabled
     * causes the stack to silently drop connection requests (central
     * sees a 10s connect timeout). Only DLE and PHY update remain. */
    stackParam.featureMask &= ~(CY_BLE_LL_PRIVACY_FEATURE |
                                CY_BLE_SECURE_CONN_FEATURE |
                                CY_BLE_STORE_BONDLIST_FEATURE |
                                CY_BLE_STORE_RESOLVING_LIST_FEATURE |
                                CY_BLE_STORE_WHITELIST_FEATURE |
                                CY_BLE_TX_POWER_CALIBRATION_FEATURE);

    q_ble_tx = xQueueCreate(BLE_TX_QUEUE_LEN, sizeof(ble_tx_item_t));
    if (q_ble_tx == NULL)
    {
        g_ble_diag_state = 0xF1u;
        task_print_error("BLE: queue create failed");
        return;
    }
    g_ble_diag_state = 2u;

    /* Install the BLESS ISR BEFORE Cy_BLE_Init so the link-layer can
     * service CONNECT_REQ and all subsequent link events. Without this
     * the stack advertises fine but silently ignores connection requests. */
    cy_ble_config.hw->blessIsrConfig = &s_bless_isr_cfg;
    (void)Cy_SysInt_Init(&s_bless_isr_cfg, &bless_interrupt_isr);
    NVIC_ClearPendingIRQ(s_bless_isr_cfg.intrSrc);
    NVIC_EnableIRQ(s_bless_isr_cfg.intrSrc);
    task_print_info("BLE: BLESS ISR hooked (IRQn=%d)", (int)s_bless_isr_cfg.intrSrc);

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
                                     configMAX_PRIORITIES - 2,
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
