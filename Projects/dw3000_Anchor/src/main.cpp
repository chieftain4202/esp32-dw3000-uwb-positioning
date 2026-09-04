// DW3000 four-anchor SS-TWR responder for Makerfabs ESP32 UWB DW3000.
// Change ANCHOR_ID to 1, 2, 3, or 4 before uploading each board.
#include <Arduino.h>
#include <string.h>

#include "dw3000.h"

#define ANCHOR_ID 1

#if ANCHOR_ID < 1 || ANCHOR_ID > 4
#error "ANCHOR_ID must be 1, 2, 3, or 4"
#endif

#define PIN_RST 27
#define PIN_IRQ 34
#define PIN_SS 4

#define NUM_ANCHORS 1
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

#define ALL_MSG_COMMON_LEN 10
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_ANCHOR_ID_IDX 10
#define RESP_MSG_POLL_RX_TS_IDX 11
#define RESP_MSG_RESP_TX_TS_IDX 15
#define RESP_MSG_TS_LEN 4

#define POLL_FRAME_LEN 12
#define POLL_DATA_LEN (POLL_FRAME_LEN - 2)
#define RESP_FRAME_LEN 21

// Anchor 1 responds first. The other anchors use separate 1200 uus slots.
// The Tag starts listening before 450 uus, so this remains compatible with it.
#define RESP_BASE_UUS 1000
#define RESP_SLOT_UUS 1200
#define DW3000_IDLE_TIMEOUT_MS 1000
#define TX_COMPLETE_TIMEOUT_MS 20

#define RX_EVENT_MASK (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR)

static const uint8_t expected_poll_msg[POLL_FRAME_LEN] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0
};

// Frame layout including the two FCS bytes appended by the DW3000:
// [0:9] header, [10] anchor ID, [11:14] poll RX timestamp,
// [15:18] response TX timestamp, [19:20] FCS.
static uint8_t tx_resp_msg[RESP_FRAME_LEN] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1,
    ANCHOR_ID, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

static uint8_t rx_buffer[POLL_DATA_LEN];
static uint32_t poll_count = 0;
static uint32_t response_count = 0;
static uint32_t late_tx_count = 0;
static uint32_t invalid_frame_count = 0;
static uint32_t last_stats_ms = 0;

static dwt_config_t config = {
    5,
    DWT_PLEN_128,
    DWT_PAC8,
    9,
    9,
    1,
    DWT_BR_6M8,
    DWT_PHRMODE_STD,
    DWT_PHRRATE_STD,
    (129 + 8 - 8),
    DWT_STS_MODE_OFF,
    DWT_STS_LEN_64,
    DWT_PDOA_M0
};

extern dwt_txconfig_t txconfig_options;

static bool deadline_expired(uint32_t deadline_ms)
{
    return (int32_t)(millis() - deadline_ms) >= 0;
}

static void halt_with_message(const char *message)
{
    Serial.println(message);
    while (true) {
        delay(1000);
    }
}

static void clear_rx_status(void)
{
    dwt_write32bitreg(
        SYS_STATUS_ID,
        SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
}

static bool poll_header_matches(const uint8_t *frame)
{
    // Byte 2 varies for each Poll and is checked only by echoing it in response.
    return memcmp(frame, expected_poll_msg, ALL_MSG_SN_IDX) == 0 &&
           memcmp(frame + ALL_MSG_SN_IDX + 1,
                  expected_poll_msg + ALL_MSG_SN_IDX + 1,
                  ALL_MSG_COMMON_LEN - ALL_MSG_SN_IDX - 1) == 0;
}

static void print_stats_periodically(void)
{
    const uint32_t now_ms = millis();
    if ((uint32_t)(now_ms - last_stats_ms) < 1000) {
        return;
    }

    last_stats_ms = now_ms;
    Serial.print("STATS,ANCHOR=");
    Serial.print(ANCHOR_ID);
    Serial.print(",POLL=");
    Serial.print(poll_count);
    Serial.print(",RESP=");
    Serial.print(response_count);
    Serial.print(",LATE=");
    Serial.print(late_tx_count);
    Serial.print(",INVALID=");
    Serial.println(invalid_frame_count);
}

static bool send_delayed_response(uint8_t poll_sequence, uint64_t poll_rx_ts)
{
    const uint32_t response_delay_uus =
        RESP_BASE_UUS + ((uint32_t)(ANCHOR_ID - 1) * RESP_SLOT_UUS);
    const uint32_t resp_tx_time =
        (uint32_t)((poll_rx_ts + response_delay_uus * UUS_TO_DWT_TIME) >> 8);

    dwt_setdelayedtrxtime(resp_tx_time);

    const uint64_t resp_tx_ts =
        (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

    tx_resp_msg[ALL_MSG_SN_IDX] = poll_sequence;
    tx_resp_msg[RESP_MSG_ANCHOR_ID_IDX] = ANCHOR_ID;
    resp_msg_set_ts(&tx_resp_msg[RESP_MSG_POLL_RX_TS_IDX], poll_rx_ts);
    resp_msg_set_ts(&tx_resp_msg[RESP_MSG_RESP_TX_TS_IDX], resp_tx_ts);

    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    if (dwt_writetxdata(RESP_FRAME_LEN, tx_resp_msg, 0) != DWT_SUCCESS) {
        return false;
    }
    dwt_writetxfctrl(RESP_FRAME_LEN, 0, 1);

    if (dwt_starttx(DWT_START_TX_DELAYED) != DWT_SUCCESS) {
        late_tx_count++;
        dwt_forcetrxoff();
        return false;
    }

    const uint32_t tx_deadline = millis() + TX_COMPLETE_TIMEOUT_MS;
    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {
        if (deadline_expired(tx_deadline)) {
            dwt_forcetrxoff();
            return false;
        }
        yield();
    }

    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    response_count++;
    return true;
}

void setup()
{
    Serial.begin(115200);
    spiBegin(PIN_IRQ, PIN_RST);
    spiSelect(PIN_SS);
    delay(2);

    const uint32_t idle_deadline = millis() + DW3000_IDLE_TIMEOUT_MS;
    while (!dwt_checkidlerc() && !deadline_expired(idle_deadline)) {
        delay(1);
    }
    if (!dwt_checkidlerc()) {
        halt_with_message("ERROR,DW3000_IDLE");
    }

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
        halt_with_message("ERROR,DW3000_INIT");
    }
    if (dwt_configure(&config) == DWT_ERROR) {
        halt_with_message("ERROR,DW3000_CONFIG");
    }

    dwt_configuretxrf(&txconfig_options);
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

    clear_rx_status();
    Serial.print("READY,UWB_ANCHOR,");
    Serial.println(ANCHOR_ID);
}

void loop()
{
    clear_rx_status();
    if (dwt_rxenable(DWT_START_RX_IMMEDIATE) != DWT_SUCCESS) {
        dwt_forcetrxoff();
        delay(1);
        return;
    }

    uint32_t status = 0;
    while (!((status = dwt_read32bitreg(SYS_STATUS_ID)) & RX_EVENT_MASK)) {
        print_stats_periodically();
        yield();
    }

    if (!(status & SYS_STATUS_RXFCG_BIT_MASK)) {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        return;
    }

    const uint64_t poll_rx_ts = get_rx_timestamp_u64();
    const uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

    if (frame_len != POLL_FRAME_LEN) {
        invalid_frame_count++;
        return;
    }

    memset(rx_buffer, 0, sizeof(rx_buffer));
    dwt_readrxdata(rx_buffer, POLL_DATA_LEN, 0);
    if (!poll_header_matches(rx_buffer)) {
        invalid_frame_count++;
        return;
    }

    poll_count++;
    send_delayed_response(rx_buffer[ALL_MSG_SN_IDX], poll_rx_ts);
    print_stats_periodically();
}
