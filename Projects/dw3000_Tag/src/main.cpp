// Compile as Arduino C++ (.ino or .cpp), not as a standalone C source file.
#include <Arduino.h>
#include <math.h>
#include <string.h>

#include "dw3000.h"

#define PIN_RST 27
#define PIN_IRQ 34
#define PIN_SS 4

#define NUM_ANCHORS 4
#define DIST_BUF_SIZE 7
#define POS_BUF_SIZE 5

#define RNG_DELAY_MS 20
#define HOST_ROUND_TIMEOUT_MS 15
#define DW3000_IDLE_TIMEOUT_MS 1000
#define FILTER_STALE_TIMEOUT_MS 500

#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

#define ALL_MSG_COMMON_LEN 10
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_ANCHOR_ID_IDX 10
#define RESP_MSG_POLL_RX_TS_IDX 11
#define RESP_MSG_RESP_TX_TS_IDX 15
#define RESP_MSG_TS_LEN 4
#define RESP_FRAME_LEN 21
#define RESP_DATA_LEN (RESP_FRAME_LEN - 2)

#define RESP_BASE_UUS 350
#define RESP_SLOT_UUS 1200
#define RX_START_GUARD_UUS 50
#define RX_REENABLE_TIMEOUT_UUS (RESP_SLOT_UUS + 400)
#define INITIAL_RX_TIMEOUT_UUS \
    (RESP_BASE_UUS + RESP_SLOT_UUS * NUM_ANCHORS + 800)

#define MAX_VALID_DISTANCE_M 10.0
#define WLS_MAX_ITERS 6
#define WLS_LAMBDA 1e-6
#define WLS_EPS 1e-6
#define WLS_VARIANCE_FLOOR 0.0025
#define WLS_MIN_WEIGHT 0.25
#define WLS_MAX_WEIGHT 4.0
#define WLS_HUBER_THRESHOLD_M 0.5

#define WORKSPACE_MIN_X 0.0
#define WORKSPACE_MAX_X 2.0
#define WORKSPACE_MIN_Y 0.0
#define WORKSPACE_MAX_Y 2.0

#define RX_EVENT_MASK                                                        \
    (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)

// Response frame layout, including two FCS bytes appended by the DW3000:
// [0:9] common header, [10] anchor ID, [11:14] poll RX timestamp,
// [15:18] response TX timestamp, [19:20] FCS.
// The responder must copy the poll sequence into response byte 2 and use a
// distinct response slot based on its anchor ID.
static uint8_t tx_poll_msg[] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0
};

static const uint8_t expected_resp_msg[RESP_FRAME_LEN] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

static uint8_t rx_buffer[RESP_FRAME_LEN];
static uint8_t frame_seq_nb = 0;

// Four anchors are placed at the corners of the 2 m x 2 m workspace.
static const double anchor_x[NUM_ANCHORS] = {0.0, 2.0, 2.0, 0.0};
static const double anchor_y[NUM_ANCHORS] = {0.0, 0.0, 2.0, 2.0};

// Apply only small, measured residual biases after antenna-delay calibration.
static const double anchor_range_bias_m[NUM_ANCHORS] = {0.0, 0.0, 0.0, 0.0};

static double dist_buf[NUM_ANCHORS][DIST_BUF_SIZE] = {{0.0}};
static uint8_t dist_buf_index[NUM_ANCHORS] = {0};
static uint8_t dist_buf_count[NUM_ANCHORS] = {0};
static uint32_t last_distance_sample_ms[NUM_ANCHORS] = {0};

static double pos_x_buf[POS_BUF_SIZE] = {0.0};
static double pos_y_buf[POS_BUF_SIZE] = {0.0};
static uint8_t pos_buf_index = 0;
static uint8_t pos_buf_count = 0;
static uint32_t last_position_sample_ms = 0;
static uint8_t last_round_anchor_mask = 0;
static uint32_t last_incomplete_log_ms = 0;
static uint32_t diag_rx_frames = 0;
static uint32_t diag_bad_length = 0;
static uint32_t diag_bad_header = 0;
static uint32_t diag_bad_id = 0;
static uint32_t diag_bad_sequence = 0;
static uint32_t diag_bad_timing = 0;
static uint32_t diag_bad_distance = 0;
static uint32_t diag_rx_errors = 0;
static uint32_t diag_poll_tx_failures = 0;

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

static double clamp_value(double value, double min_value, double max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static bool deadline_expired(uint32_t deadline_ms)
{
    return (int32_t)(millis() - deadline_ms) >= 0;
}

static void clear_rx_status(void)
{
    dwt_write32bitreg(
        SYS_STATUS_ID,
        SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
}

static void stop_and_clear_receiver(void)
{
    dwt_forcetrxoff();
    clear_rx_status();
}

static void halt_with_message(const char *message)
{
    Serial.println(message);
    while (true) {
        delay(1000);
    }
}

static bool response_header_matches(const uint8_t *frame)
{
    // Byte 2 is the responder's sequence number and is intentionally ignored.
    return memcmp(frame, expected_resp_msg, ALL_MSG_SN_IDX) == 0 &&
           memcmp(frame + ALL_MSG_SN_IDX + 1,
                  expected_resp_msg + ALL_MSG_SN_IDX + 1,
                  ALL_MSG_COMMON_LEN - ALL_MSG_SN_IDX - 1) == 0;
}

static double add_distance_sample(uint8_t anchor_index, double distance_m)
{
    const uint32_t now_ms = millis();
    if (dist_buf_count[anchor_index] > 0 &&
        (uint32_t)(now_ms - last_distance_sample_ms[anchor_index]) >
            FILTER_STALE_TIMEOUT_MS) {
        memset(dist_buf[anchor_index], 0, sizeof(dist_buf[anchor_index]));
        dist_buf_index[anchor_index] = 0;
        dist_buf_count[anchor_index] = 0;
    }
    last_distance_sample_ms[anchor_index] = now_ms;

    dist_buf[anchor_index][dist_buf_index[anchor_index]] = distance_m;
    dist_buf_index[anchor_index] =
        (uint8_t)((dist_buf_index[anchor_index] + 1) % DIST_BUF_SIZE);

    if (dist_buf_count[anchor_index] < DIST_BUF_SIZE) {
        dist_buf_count[anchor_index]++;
    }

    double sum = 0.0;
    for (uint8_t i = 0; i < dist_buf_count[anchor_index]; i++) {
        sum += dist_buf[anchor_index][i];
    }
    return sum / dist_buf_count[anchor_index];
}

static void update_anchor_weights(double weights[NUM_ANCHORS])
{
    double raw_weights[NUM_ANCHORS];
    double raw_sum = 0.0;

    for (uint8_t i = 0; i < NUM_ANCHORS; i++) {
        if (dist_buf_count[i] < 2) {
            raw_weights[i] = 1.0;
        } else {
            double sum = 0.0;
            double sum_sq = 0.0;

            for (uint8_t k = 0; k < dist_buf_count[i]; k++) {
                const double value = dist_buf[i][k];
                sum += value;
                sum_sq += value * value;
            }

            const double count = dist_buf_count[i];
            const double mean = sum / count;
            double variance = (sum_sq / count) - (mean * mean);
            if (variance < 0.0) {
                variance = 0.0;
            }
            raw_weights[i] = 1.0 / (variance + WLS_VARIANCE_FLOOR);
        }
        raw_sum += raw_weights[i];
    }

    const double raw_mean = raw_sum / NUM_ANCHORS;
    for (uint8_t i = 0; i < NUM_ANCHORS; i++) {
        const double normalized = raw_weights[i] / raw_mean;
        weights[i] = clamp_value(normalized, WLS_MIN_WEIGHT, WLS_MAX_WEIGHT);
    }
}

static bool initial_position_estimate(
    const double distances[NUM_ANCHORS], double *x, double *y)
{
    // Linearized least-squares estimate using anchor 0 as the reference.
    double ata_11 = 0.0;
    double ata_12 = 0.0;
    double ata_22 = 0.0;
    double atb_1 = 0.0;
    double atb_2 = 0.0;

    for (uint8_t i = 1; i < NUM_ANCHORS; i++) {
        const double a1 = 2.0 * (anchor_x[i] - anchor_x[0]);
        const double a2 = 2.0 * (anchor_y[i] - anchor_y[0]);
        const double b = distances[0] * distances[0] -
                         distances[i] * distances[i] +
                         anchor_x[i] * anchor_x[i] -
                         anchor_x[0] * anchor_x[0] +
                         anchor_y[i] * anchor_y[i] -
                         anchor_y[0] * anchor_y[0];

        ata_11 += a1 * a1;
        ata_12 += a1 * a2;
        ata_22 += a2 * a2;
        atb_1 += a1 * b;
        atb_2 += a2 * b;
    }

    const double determinant = ata_11 * ata_22 - ata_12 * ata_12;
    if (fabs(determinant) < 1e-12) {
        return false;
    }

    *x = (ata_22 * atb_1 - ata_12 * atb_2) / determinant;
    *y = (-ata_12 * atb_1 + ata_11 * atb_2) / determinant;
    *x = clamp_value(*x, WORKSPACE_MIN_X, WORKSPACE_MAX_X);
    *y = clamp_value(*y, WORKSPACE_MIN_Y, WORKSPACE_MAX_Y);
    return isfinite(*x) && isfinite(*y);
}

static bool refine_position_wls(
    const double distances[NUM_ANCHORS],
    const double weights[NUM_ANCHORS],
    double *x,
    double *y)
{
    for (uint8_t iteration = 0; iteration < WLS_MAX_ITERS; iteration++) {
        double jtj_11 = WLS_LAMBDA;
        double jtj_12 = 0.0;
        double jtj_22 = WLS_LAMBDA;
        double jtr_1 = 0.0;
        double jtr_2 = 0.0;

        for (uint8_t i = 0; i < NUM_ANCHORS; i++) {
            const double delta_x = *x - anchor_x[i];
            const double delta_y = *y - anchor_y[i];
            double predicted = sqrt(delta_x * delta_x + delta_y * delta_y);
            if (predicted < WLS_EPS) {
                predicted = WLS_EPS;
            }

            const double residual = distances[i] - predicted;
            const double jacobian_x = delta_x / predicted;
            const double jacobian_y = delta_y / predicted;
            const double residual_magnitude = fabs(residual);
            const double robust_weight =
                residual_magnitude <= WLS_HUBER_THRESHOLD_M
                    ? 1.0
                    : WLS_HUBER_THRESHOLD_M / residual_magnitude;
            const double weight = weights[i] * robust_weight;

            jtj_11 += weight * jacobian_x * jacobian_x;
            jtj_12 += weight * jacobian_x * jacobian_y;
            jtj_22 += weight * jacobian_y * jacobian_y;
            jtr_1 += weight * jacobian_x * residual;
            jtr_2 += weight * jacobian_y * residual;
        }

        const double determinant = jtj_11 * jtj_22 - jtj_12 * jtj_12;
        if (fabs(determinant) < 1e-12) {
            return false;
        }

        const double step_x =
            (jtj_22 * jtr_1 - jtj_12 * jtr_2) / determinant;
        const double step_y =
            (-jtj_12 * jtr_1 + jtj_11 * jtr_2) / determinant;

        *x = clamp_value(*x + step_x, WORKSPACE_MIN_X, WORKSPACE_MAX_X);
        *y = clamp_value(*y + step_y, WORKSPACE_MIN_Y, WORKSPACE_MAX_Y);

        if (!isfinite(*x) || !isfinite(*y)) {
            return false;
        }
        if (step_x * step_x + step_y * step_y < 1e-8) {
            break;
        }
    }
    return true;
}

static void smooth_position(double x, double y, double *smooth_x, double *smooth_y)
{
    const uint32_t now_ms = millis();
    if (pos_buf_count > 0 &&
        (uint32_t)(now_ms - last_position_sample_ms) > FILTER_STALE_TIMEOUT_MS) {
        memset(pos_x_buf, 0, sizeof(pos_x_buf));
        memset(pos_y_buf, 0, sizeof(pos_y_buf));
        pos_buf_index = 0;
        pos_buf_count = 0;
    }
    last_position_sample_ms = now_ms;

    pos_x_buf[pos_buf_index] = x;
    pos_y_buf[pos_buf_index] = y;
    pos_buf_index = (uint8_t)((pos_buf_index + 1) % POS_BUF_SIZE);

    if (pos_buf_count < POS_BUF_SIZE) {
        pos_buf_count++;
    }

    double sum_x = 0.0;
    double sum_y = 0.0;
    for (uint8_t i = 0; i < pos_buf_count; i++) {
        sum_x += pos_x_buf[i];
        sum_y += pos_y_buf[i];
    }

    *smooth_x = sum_x / pos_buf_count;
    *smooth_y = sum_y / pos_buf_count;
}

static bool process_response(
    uint32_t poll_tx_ts,
    uint8_t poll_sequence,
    bool received_this_round[NUM_ANCHORS],
    double round_distances[NUM_ANCHORS])
{
    const uint32_t resp_rx_ts = dwt_readrxtimestamplo32();
    const float clock_offset_ratio =
        ((float)dwt_readclockoffset()) / (uint32_t)(1UL << 26);
    const uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    diag_rx_frames++;

    if (frame_len != RESP_FRAME_LEN) {
        diag_bad_length++;
        return false;
    }

    memset(rx_buffer, 0, sizeof(rx_buffer));
    dwt_readrxdata(rx_buffer, RESP_DATA_LEN, 0);

    if (!response_header_matches(rx_buffer)) {
        diag_bad_header++;
        return false;
    }

    const uint8_t anchor_id = rx_buffer[RESP_MSG_ANCHOR_ID_IDX];
    if (anchor_id < 1 || anchor_id > NUM_ANCHORS) {
        diag_bad_id++;
        return false;
    }

    const uint8_t anchor_index = anchor_id - 1;
    if (received_this_round[anchor_index]) {
        return false;
    }

    // Protocol contract: each responder echoes the poll sequence in byte 2.
    if (rx_buffer[ALL_MSG_SN_IDX] != poll_sequence) {
        diag_bad_sequence++;
        return false;
    }

    uint32_t poll_rx_ts = 0;
    uint32_t resp_tx_ts = 0;
    resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
    resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

    const int32_t rtd_initiator = (int32_t)(resp_rx_ts - poll_tx_ts);
    const int32_t rtd_responder = (int32_t)(resp_tx_ts - poll_rx_ts);
    if (rtd_initiator <= 0 || rtd_responder <= 0) {
        diag_bad_timing++;
        return false;
    }

    const double tof =
        ((rtd_initiator - rtd_responder * (1.0 - clock_offset_ratio)) / 2.0) *
        DWT_TIME_UNITS;
    double distance_m = tof * SPEED_OF_LIGHT - anchor_range_bias_m[anchor_index];

    if (!isfinite(distance_m) || distance_m < 0.0 ||
        distance_m > MAX_VALID_DISTANCE_M) {
        diag_bad_distance++;
        return false;
    }

    round_distances[anchor_index] = add_distance_sample(anchor_index, distance_m);
    received_this_round[anchor_index] = true;
    return true;
}

static bool collect_ranging_round(double round_distances[NUM_ANCHORS])
{
    bool received_this_round[NUM_ANCHORS] = {false};
    uint8_t received_count = 0;
    const uint8_t poll_sequence = frame_seq_nb++;

    tx_poll_msg[ALL_MSG_SN_IDX] = poll_sequence;
    dwt_write32bitreg(
        SYS_STATUS_ID,
        SYS_STATUS_TXFRS_BIT_MASK | SYS_STATUS_RXFCG_BIT_MASK |
            SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
    if (dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0) != DWT_SUCCESS) {
        diag_poll_tx_failures++;
        stop_and_clear_receiver();
        return false;
    }
    dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);

    if (dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED) !=
        DWT_SUCCESS) {
        diag_poll_tx_failures++;
        stop_and_clear_receiver();
        return false;
    }

    const uint32_t deadline_ms = millis() + HOST_ROUND_TIMEOUT_MS;
    bool first_response = true;
    uint32_t poll_tx_ts = 0;

    while (!deadline_expired(deadline_ms) && received_count < NUM_ANCHORS) {
        uint32_t status = 0;
        while (!deadline_expired(deadline_ms)) {
            status = dwt_read32bitreg(SYS_STATUS_ID);
            if (status & RX_EVENT_MASK) {
                break;
            }
            yield();
        }

        if (deadline_expired(deadline_ms)) {
            break;
        }

        if (status & SYS_STATUS_RXFCG_BIT_MASK) {
            if (first_response) {
                poll_tx_ts = dwt_readtxtimestamplo32();
                first_response = false;
            }

            if (process_response(
                    poll_tx_ts,
                    poll_sequence,
                    received_this_round,
                    round_distances)) {
                received_count++;
            }
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
        } else {
            diag_rx_errors++;
            dwt_write32bitreg(
                SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        }

        if (received_count < NUM_ANCHORS && !deadline_expired(deadline_ms)) {
            dwt_setrxtimeout(RX_REENABLE_TIMEOUT_UUS);
            if (dwt_rxenable(DWT_START_RX_IMMEDIATE) != DWT_SUCCESS) {
                break;
            }
        }
    }

    stop_and_clear_receiver();
    dwt_setrxtimeout(INITIAL_RX_TIMEOUT_UUS);

    last_round_anchor_mask = 0;
    for (uint8_t i = 0; i < NUM_ANCHORS; i++) {
        if (received_this_round[i]) {
            last_round_anchor_mask |= (uint8_t)(1U << i);
        }
    }
    return received_count == NUM_ANCHORS;
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
    dwt_setrxaftertxdelay(RESP_BASE_UUS - RX_START_GUARD_UUS);
    dwt_setrxtimeout(INITIAL_RX_TIMEOUT_UUS);
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

    Serial.println("READY,UWB_4ANCHOR_WLS");
}

void loop()
{
    double round_distances[NUM_ANCHORS] = {0.0};
    if (!collect_ranging_round(round_distances)) {
        const uint32_t now_ms = millis();
        if ((uint32_t)(now_ms - last_incomplete_log_ms) >= 500) {
            last_incomplete_log_ms = now_ms;
            Serial.print("WARN,INCOMPLETE_ROUND,MASK=0x");
            Serial.print(last_round_anchor_mask, HEX);
            Serial.print(",RX=");
            Serial.print(diag_rx_frames);
            Serial.print(",LEN=");
            Serial.print(diag_bad_length);
            Serial.print(",HDR=");
            Serial.print(diag_bad_header);
            Serial.print(",ID=");
            Serial.print(diag_bad_id);
            Serial.print(",SEQ=");
            Serial.print(diag_bad_sequence);
            Serial.print(",TIME=");
            Serial.print(diag_bad_timing);
            Serial.print(",DIST=");
            Serial.print(diag_bad_distance);
            Serial.print(",RXERR=");
            Serial.print(diag_rx_errors);
            Serial.print(",TXFAIL=");
            Serial.println(diag_poll_tx_failures);
        }
        delay(RNG_DELAY_MS);
        return;
    }

    double weights[NUM_ANCHORS];
    update_anchor_weights(weights);

    double x = 0.0;
    double y = 0.0;
    if (!initial_position_estimate(round_distances, &x, &y) ||
        !refine_position_wls(round_distances, weights, &x, &y)) {
        Serial.println("WARN,POSITION_SOLVE");
        delay(RNG_DELAY_MS);
        return;
    }

    double smooth_x = 0.0;
    double smooth_y = 0.0;
    smooth_position(x, y, &smooth_x, &smooth_y);

    Serial.print(smooth_x, 3);
    Serial.print(',');
    Serial.println(smooth_y, 3);
    delay(RNG_DELAY_MS);
}
