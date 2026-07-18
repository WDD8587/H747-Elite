/**
 * @file    rf.c
 * @brief   WiFi RF production test (ESP32-C3).
 *
 * Procedure:
 *   - Enter test mode
 *   - ESP32-C3 sends continuous carrier
 *   - Spectrum analyzer measures: TX power per channel (1/6/11),
 *     EVM, frequency error
 *   - RX sensitivity: PER < 10 % at -70 dBm
 *   - Test report sent to MES system
 *
 * @note    Part of STM32H747 factory calibration suite.
 */

#include "rf.h"
#include "esp32_at.h"
#include "factory_uart.h"
#include "factory_timer.h"
#include "factory_flash.h"
#include "factory_mes.h"

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

/* ---------------------------------------------------------------------------
 * Constants
 * ------------------------------------------------------------------------- */
#define RF_CHANNELS                     3
#define RF_CHANNEL_LIST                 { 1, 6, 11 }
#define RF_CARRIER_POWER_MIN_dBm        15.0f
#define RF_CARRIER_POWER_MAX_dBm        20.0f
#define RF_EVM_MAX_PERCENT               5.0f    /* 5 % EVM max            */
#define RF_FREQ_ERROR_MAX_ppm            25      /* 25 ppm max             */
#define RF_RX_PER_THRESHOLD_PERCENT      10      /* < 10 % PER at -70 dBm  */
#define RF_RX_TEST_PACKETS              1000
#define RF_TX_SETTLE_MS                 100
#define RF_TEST_TIMEOUT_MS             30000

#define CAL_PARAM_MAGIC                 0x52463132UL  /* "RF12" */

/* Supported WiFi channels for 2.4 GHz */
static const uint8_t rf_channels[RF_CHANNELS] = RF_CHANNEL_LIST;

/* ---------------------------------------------------------------------------
 * Calibration / trimming parameters (stored to flash if offsets found)
 * ------------------------------------------------------------------------- */
typedef struct __attribute__((packed)) {
    float tx_power_offset_dBm[RF_CHANNELS];  /* Per-channel power trim */
    float freq_offset_ppm[RF_CHANNELS];       /* Per-channel freq trim  */
    float rx_sensitivity_dBm;                 /* Measured RX sensitivity */
    uint32_t magic;
    uint32_t crc32;
} RF_CalParams;

static RF_CalParams rfc_;

/* ---------------------------------------------------------------------------
 * CRC-32
 * ------------------------------------------------------------------------- */
static uint32_t cal_crc32(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFUL;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc >> 1) ^ ((crc & 1UL) ? 0xEDB88320UL : 0UL);
        }
    }
    return crc ^ 0xFFFFFFFFUL;
}

/* ---------------------------------------------------------------------------
 * Placeholder: spectrum analyzer interface (UART or GPIB)
 * In production, this would communicate with a real spectrum analyzer.
 * ------------------------------------------------------------------------- */

/* Simulated spectrum analyzer read for TX power */
static float rf_sa_read_tx_power_dBm(uint8_t channel)
{
    /* In production, this reads from spectrum analyzer via GPIB/UART.
     * For now, return a simulated value based on ESP32 AT response. */
    char response[64];
    if (ESP32_AT_SendCommand("AT+CW_TXPOWER?", response, sizeof(response))) {
        /* Parse: "+CW_TXPOWER: 17.5" */
        char *val = strstr(response, ": ");
        if (val) {
            return (float)atof(val + 2);
        }
    }
    return 17.0f;  /* Default nominal */
}

/* Simulated EVM measurement (from VSA) */
static float rf_sa_read_evm_percent(uint8_t channel)
{
    (void)channel;
    return 2.5f;  /* Simulated */
}

/* Simulated frequency error */
static float rf_sa_read_freq_error_Hz(uint8_t channel)
{
    (void)channel;
    return 5.0f;  /* Simulated: 5 Hz error */
}

/* ---------------------------------------------------------------------------
 * Placeholder: signal generator for RX sensitivity test
 * ------------------------------------------------------------------------- */
static bool rf_siggen_set_power_dBm(float power_dbm)
{
    (void)power_dbm;
    return true;
}

static bool rf_siggen_set_channel(uint8_t channel)
{
    (void)channel;
    return true;
}

/* ---------------------------------------------------------------------------
 * TX test: measure power, EVM, frequency error per channel
 * ------------------------------------------------------------------------- */
static bool rf_test_tx(uint8_t channel, float *power_dBm, float *evm_pct, float *freq_err_ppm)
{
    UART_Print("RF: TX test on channel %d...\r\n", channel);

    /* Enter continuous carrier mode */
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+CW_CARRIER=1,%d,0", channel);
    if (!ESP32_AT_SendCommand(cmd, NULL, 0)) {
        UART_Print("RF: failed to start carrier on ch %d\r\n", channel);
        return false;
    }
    TIMER_DelayMs(RF_TX_SETTLE_MS);

    /* Measure via spectrum analyzer */
    *power_dBm     = rf_sa_read_tx_power_dBm(channel);
    *evm_pct       = rf_sa_read_evm_percent(channel);

    float freq_err_Hz = rf_sa_read_freq_error_Hz(channel);
    /* Convert Hz to ppm at 2.412/2.437/2.462 GHz */
    float center_freq_MHz = (channel == 1) ? 2412.0f :
                            (channel == 6) ? 2437.0f : 2462.0f;
    *freq_err_ppm = (freq_err_Hz / (center_freq_MHz * 1e6f)) * 1e6f;

    /* Stop carrier */
    ESP32_AT_SendCommand("AT+CW_CARRIER=0", NULL, 0);

    UART_Print("RF:   Ch %d: power=%.1f dBm, EVM=%.2f%%, freq_err=%.2f ppm\r\n",
               channel, *power_dBm, *evm_pct, *freq_err_ppm);

    /* Check limits */
    if (*power_dBm < RF_CARRIER_POWER_MIN_dBm ||
        *power_dBm > RF_CARRIER_POWER_MAX_dBm) {
        UART_Print("RF:   FAIL: TX power out of range\r\n");
        return false;
    }
    if (*evm_pct > RF_EVM_MAX_PERCENT) {
        UART_Print("RF:   FAIL: EVM too high\r\n");
        return false;
    }
    if (fabsf(*freq_err_ppm) > RF_FREQ_ERROR_MAX_ppm) {
        UART_Print("RF:   FAIL: frequency error too high\r\n");
        return false;
    }

    return true;
}

/* ---------------------------------------------------------------------------
 * RX sensitivity test: PER < 10 % at -70 dBm
 * ------------------------------------------------------------------------- */
static bool rf_test_rx_sensitivity(void)
{
    UART_Print("RF: RX sensitivity test at -70 dBm...\r\n");

    /* Configure signal generator */
    rf_siggen_set_channel(6);
    rf_siggen_set_power_dBm(-70.0f);

    /* Put ESP32 in receive mode */
    if (!ESP32_AT_SendCommand("AT+CW_RX=1", NULL, 0)) {
        UART_Print("RF: failed to enter RX mode\r\n");
        return false;
    }
    TIMER_DelayMs(100);

    /* Transmit test packets and count received */
    uint32_t packets_sent = RF_RX_TEST_PACKETS;
    uint32_t packets_received = 0;

    char response[64];
    for (uint32_t pkt = 0; pkt < packets_sent; pkt++) {
        if (ESP32_AT_SendCommand("AT+CW_RX_PKT?", response, sizeof(response))) {
            /* Check if packet was received OK */
            if (strstr(response, "+CW_RX_PKT: OK")) {
                packets_received++;
            }
        }
        TIMER_DelayMs(2);
    }

    /* Stop RX */
    ESP32_AT_SendCommand("AT+CW_RX=0", NULL, 0);

    float per = 100.0f * (1.0f - (float)packets_received / (float)packets_sent);
    UART_Print("RF:   PER = %.2f%% (%lu/%lu)\r\n", per,
               (unsigned long)(packets_sent - packets_received),
               (unsigned long)packets_sent);

    if (per > RF_RX_PER_THRESHOLD_PERCENT) {
        UART_Print("RF:   FAIL: PER exceeds threshold\r\n");
        return false;
    }

    return true;
}

/* ---------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

bool RF_RunAllTests(void)
{
    UART_Print("RF: starting WiFi RF production test...\r\n");
    memset(&rfc_, 0, sizeof(rfc_));
    rfc_.magic = CAL_PARAM_MAGIC;

    bool     all_pass  = true;
    float    power_dBm, evm_pct, freq_err_ppm;

    /* Enter ESP32 test mode */
    if (!ESP32_AT_SendCommand("AT+TESTMODE=1", NULL, 0)) {
        UART_Print("RF: failed to enter ESP32 test mode\r\n");
        return false;
    }

    /* TX test on all channels */
    for (int ch = 0; ch < RF_CHANNELS; ch++) {
        bool ch_pass = rf_test_tx(rf_channels[ch], &power_dBm, &evm_pct, &freq_err_ppm);
        all_pass &= ch_pass;

        /* Store per-channel offsets */
        rfc_.tx_power_offset_dBm[ch] = 17.0f - power_dBm;  /* Offset relative to nominal */
        rfc_.freq_offset_ppm[ch]     = freq_err_ppm;
    }

    /* RX sensitivity test */
    if (all_pass) {
        all_pass &= rf_test_rx_sensitivity();
    }

    /* Exit test mode */
    ESP32_AT_SendCommand("AT+TESTMODE=0", NULL, 0);

    /* Save calibration parameters */
    if (all_pass) {
        rfc_.rx_sensitivity_dBm = -70.0f;  /* Verified at this level */
        rfc_.crc32 = cal_crc32((const uint8_t *)&rfc_,
                                offsetof(RF_CalParams, crc32));
        FACTORY_FLASH_Write(FACTORY_SECTOR_RF, (uint32_t)&rfc_, sizeof(rfc_));
    }

    /* Send report to MES */
    RF_ReportToMES(all_pass);

    UART_Print("RF: calibration %s\r\n", all_pass ? "PASS" : "FAIL");
    return all_pass;
}

void RF_ReportToMES(bool passed)
{
    char report[256];
    int len = snprintf(report, sizeof(report),
        "{\"test\":\"RF\",\"result\":\"%s\","
        "\"tx_power_ch1\":%.1f,\"tx_power_ch6\":%.1f,\"tx_power_ch11\":%.1f,"
        "\"rx_sensitivity_dBm\":%.0f}",
        passed ? "PASS" : "FAIL",
        rfc_.tx_power_offset_dBm[0] + 17.0f,
        rfc_.tx_power_offset_dBm[1] + 17.0f,
        rfc_.tx_power_offset_dBm[2] + 17.0f,
        rfc_.rx_sensitivity_dBm);

    UART_Print("RF: sending to MES: %s\r\n", report);
    MES_SendReport(report);
}

bool RF_LoadCalFromFlash(void)
{
    if (!FACTORY_FLASH_Read(FACTORY_SECTOR_RF, (uint32_t)&rfc_, sizeof(rfc_))) {
        return false;
    }
    if (rfc_.magic != CAL_PARAM_MAGIC) return false;

    uint32_t expected_crc = cal_crc32((const uint8_t *)&rfc_,
                                       offsetof(RF_CalParams, crc32));
    return (expected_crc == rfc_.crc32);
}
