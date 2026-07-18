/**
 * @file    debug_cmd.c
 * @brief   Debug command handler via UART auxiliary port.
 *
 * @details Parses space-delimited commands from the debug UART:
 *   "dump motor"   — dump FOC state (current, velocity, angle, PWM duty)
 *   "dump bms"     — dump BMS registers (voltage, current, SOC, temperature)
 *   "dump safety"  — dump fail-safe state (watchdog, timeout, brake status)
 *   "dump fault"   — dump last 10 fault codes
 *   "cal motor"    — trigger motor calibration
 *   "cal imu"      — trigger IMU calibration
 *   "reboot"       — trigger software system reset
 *   "bootloader"   — jump to bootloader (set flag and reset)
 *
 * Commands are read from UART7 (shared between M4 and M7 via mailbox
 * or direct UART). The M4 handles auxiliary debug commands and responds
 * on the same UART.
 */

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include "stm32h7xx_hal.h"
#include "debug_cmd.h"

/* ---------------------------------------------------------------------------
 * UART handle (external, must be initialised by application)
 * -------------------------------------------------------------------------*/
extern UART_HandleTypeDef huart7;

/* ---------------------------------------------------------------------------
 * Command buffer
 * -------------------------------------------------------------------------*/
#define CMD_BUF_SIZE       128
#define CMD_RESP_SIZE      512
#define MAX_ARGS           8

static char rx_buf[CMD_BUF_SIZE];
static uint32_t rx_idx = 0;

/* ---------------------------------------------------------------------------
 * External state references
 * -------------------------------------------------------------------------*/
extern volatile float motor_current_d;    /* FOC: direct axis current     */
extern volatile float motor_current_q;    /* FOC: quadrature axis current */
extern volatile float motor_velocity;     /* Electrical velocity (rad/s)  */
extern volatile float motor_angle;        /* Rotor angle (rad)            */
extern volatile float motor_pwm_duty;     /* PWM duty cycle (0.0-1.0)    */

extern volatile float bms_voltage;        /* BMS pack voltage (V)         */
extern volatile float bms_current;        /* BMS current (A)              */
extern volatile float bms_soc;            /* BMS state of charge (0-100)  */
extern volatile float bms_temp;           /* BMS temperature (deg C)      */

extern volatile uint8_t  safety_brake_engaged;  /* 1 = brake active      */
extern volatile uint32_t safety_watchdog_count;  /* WDT timeout count    */
extern volatile uint32_t safety_timeout_ms;      /* Last timeout (ms)    */

extern volatile uint32_t fault_ring[10];  /* Last 10 fault codes         */
extern volatile uint32_t fault_ring_idx;  /* Current index in ring       */

/* Calibration triggers (set by application) */
extern volatile uint32_t cal_motor_trigger;
extern volatile uint32_t cal_imu_trigger;

/* ---------------------------------------------------------------------------
 * Response output
 * -------------------------------------------------------------------------*/

/**
 * @brief  Send a response string via UART7.
 *
 * @param  str  Null-terminated string to send.
 */
static void cmd_respond(const char *str)
{
    HAL_UART_Transmit(&huart7, (uint8_t *)str, strlen(str), 100);
}

/**
 * @brief  Send a formatted response via UART7.
 *
 * @param  fmt  Printf-style format string.
 */
static void cmd_respondf(const char *fmt, ...)
{
    char buf[CMD_RESP_SIZE];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    cmd_respond(buf);
}

/* ---------------------------------------------------------------------------
 * Command handlers
 * -------------------------------------------------------------------------*/

/**
 * @brief  "dump motor" — dump FOC state variables.
 */
static void cmd_dump_motor(void)
{
    cmd_respondf(
        "=== FOC State ===\r\n"
        "  Current D:   %.3f A\r\n"
        "  Current Q:   %.3f A\r\n"
        "  Velocity:    %.2f rad/s\r\n"
        "  Angle:       %.3f rad\r\n"
        "  PWM Duty:    %.3f\r\n"
        "==================\r\n",
        (double)motor_current_d,
        (double)motor_current_q,
        (double)motor_velocity,
        (double)motor_angle,
        (double)motor_pwm_duty);
}

/**
 * @brief  "dump bms" — dump BMS registers.
 */
static void cmd_dump_bms(void)
{
    cmd_respondf(
        "=== BMS State ===\r\n"
        "  Voltage:     %.3f V\r\n"
        "  Current:     %.3f A\r\n"
        "  SOC:         %.1f %%\r\n"
        "  Temp:        %.1f C\r\n"
        "==================\r\n",
        (double)bms_voltage,
        (double)bms_current,
        (double)bms_soc,
        (double)bms_temp);
}

/**
 * @brief  "dump safety" — dump fail-safe state.
 */
static void cmd_dump_safety(void)
{
    cmd_respondf(
        "=== Safety State ===\r\n"
        "  Brake Engaged:  %s\r\n"
        "  WDT Count:      %u\r\n"
        "  Last Timeout:   %u ms\r\n"
        "=====================\r\n",
        safety_brake_engaged ? "YES" : "NO",
        (unsigned int)safety_watchdog_count,
        (unsigned int)safety_timeout_ms);
}

/**
 * @brief  "dump fault" — dump last 10 faults.
 */
static void cmd_dump_fault(void)
{
    cmd_respond("=== Fault Log (last 10) ===\r\n");

    for (uint32_t i = 0; i < 10; i++) {
        uint32_t idx = (fault_ring_idx + i) % 10;
        uint32_t code = fault_ring[idx];

        if (code == 0) {
            cmd_respondf("  [%u] (empty)\r\n", (unsigned int)i);
        } else {
            cmd_respondf("  [%u] 0x%08X\r\n",
                         (unsigned int)i, (unsigned int)code);
        }
    }

    cmd_respond("=============================\r\n");
}

/**
 * @brief  "cal motor" — trigger motor calibration.
 */
static void cmd_cal_motor(void)
{
    cal_motor_trigger = 1;
    cmd_respond("Motor calibration triggered.\r\n");
}

/**
 * @brief  "cal imu" — trigger IMU calibration.
 */
static void cmd_cal_imu(void)
{
    cal_imu_trigger = 1;
    cmd_respond("IMU calibration triggered.\r\n");
}

/**
 * @brief  "reboot" — software reset.
 */
static void cmd_reboot(void)
{
    cmd_respond("Rebooting...\r\n");

    /* Small delay to allow UART to flush */
    HAL_Delay(10);

    /* Generate software reset */
    NVIC_SystemReset();
}

/**
 * @brief  "bootloader" — jump to bootloader.
 */
static void cmd_bootloader(void)
{
    /*
     * Set OTA flag in SRAM2 and reset.
     * The bootloader will see the flag and enter DFU mode.
     */
    volatile uint32_t *ota_flag = (volatile uint32_t *)0x20020000UL;
    *ota_flag = 0xDEADBEAFUL;
    __DSB();

    cmd_respond("Jumping to bootloader...\r\n");
    HAL_Delay(10);

    NVIC_SystemReset();
}

/* ---------------------------------------------------------------------------
 * Command dispatch table
 * -------------------------------------------------------------------------*/

typedef struct {
    const char *verb;       /* First word      */
    const char *noun;       /* Second word     */
    void (*handler)(void);  /* Handler function */
    const char *help;       /* Help text       */
} cmd_entry_t;

static const cmd_entry_t cmd_table[] = {
    {"dump",  "motor",    cmd_dump_motor,   "Dump FOC motor state"},
    {"dump",  "bms",      cmd_dump_bms,     "Dump BMS registers"},
    {"dump",  "safety",   cmd_dump_safety,  "Dump fail-safe state"},
    {"dump",  "fault",    cmd_dump_fault,   "Dump last 10 fault codes"},
    {"cal",   "motor",    cmd_cal_motor,    "Trigger motor calibration"},
    {"cal",   "imu",      cmd_cal_imu,      "Trigger IMU calibration"},
    {"reboot", "",        cmd_reboot,       "Software system reset"},
    {"bootloader", "",    cmd_bootloader,   "Jump to bootloader (DFU)"},
    {"help",  "",         NULL,             "Show this help"},
};

static const uint32_t cmd_table_count =
    sizeof(cmd_table) / sizeof(cmd_table[0]);

/* ---------------------------------------------------------------------------
 * Parser
 * -------------------------------------------------------------------------*/

/**
 * @brief  Tokenise a command string into arguments.
 *
 * @param  line   Input null-terminated string.
 * @param  argv   Output: array of pointers to tokens.
 * @param  max    Maximum number of tokens.
 * @return Number of tokens parsed.
 */
static int tokenise(char *line, char *argv[], int max)
{
    int argc = 0;
    char *p = line;

    while (*p && argc < max) {
        /* Skip leading whitespace */
        while (*p && (unsigned char)(*p) <= ' ') p++;
        if (!*p) break;

        argv[argc++] = p;

        /* Skip to next whitespace */
        while (*p && (unsigned char)(*p) > ' ') p++;

        if (*p) {
            *p++ = '\0';
        }
    }

    return argc;
}

/**
 * @brief  Show help text listing all commands.
 */
static void cmd_help(void)
{
    cmd_respond("Available commands:\r\n");
    for (uint32_t i = 0; i < cmd_table_count; i++) {
        if (cmd_table[i].noun[0] != '\0') {
            cmd_respondf("  %s %s  -- %s\r\n",
                         cmd_table[i].verb,
                         cmd_table[i].noun,
                         cmd_table[i].help);
        } else {
            cmd_respondf("  %s       -- %s\r\n",
                         cmd_table[i].verb,
                         cmd_table[i].help);
        }
    }
}

/**
 * @brief  Parse and execute a command line.
 *
 * @param  line  Null-terminated command string.
 */
static void cmd_execute(char *line)
{
    char *argv[MAX_ARGS];
    int argc = tokenise(line, argv, MAX_ARGS);

    if (argc == 0) return;

    /* Look up command in dispatch table */
    for (uint32_t i = 0; i < cmd_table_count; i++) {
        if (strcmp(argv[0], cmd_table[i].verb) != 0) continue;

        /* Match noun (if expected) */
        if (argc >= 2 && cmd_table[i].noun[0] != '\0') {
            if (strcmp(argv[1], cmd_table[i].noun) != 0) continue;
        } else if (cmd_table[i].noun[0] != '\0') {
            continue;  /* Noun required but not provided */
        }

        /* Execute handler */
        if (cmd_table[i].handler != NULL) {
            cmd_table[i].handler();
        } else {
            cmd_help();
        }
        return;
    }

    cmd_respondf("Unknown command: %s\r\nType 'help' for available commands.\r\n",
                 argv[0]);
}

/* ---------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------*/

/**
 * @brief  Initialise the debug command handler.
 *
 * Clears the RX buffer and sends a welcome banner.
 */
void debug_cmd_init(void)
{
    rx_idx = 0;
    memset(rx_buf, 0, sizeof(rx_buf));

    cmd_respond("\r\n=== H747 Debug Console ===\r\n");
    cmd_respond("Type 'help' for commands.\r\n");
}

/**
 * @brief  Process a single received character.
 *
 * Call this from the UART7 RX interrupt handler or DMA callback.
 * Characters are accumulated until newline, then the line is parsed.
 *
 * @param  c  Received character.
 */
void debug_cmd_process_char(char c)
{
    /* Echo character */
    HAL_UART_Transmit(&huart7, (uint8_t *)&c, 1, 10);

    if (c == '\r' || c == '\n') {
        /* End of line */
        cmd_respond("\r\n");

        if (rx_idx > 0) {
            rx_buf[rx_idx] = '\0';
            cmd_execute(rx_buf);
            rx_idx = 0;
            memset(rx_buf, 0, sizeof(rx_buf));
        }

        /* Prompt */
        cmd_respond("> ");
        return;
    }

    if (c == '\b' || c == 0x7F) {
        /* Backspace */
        if (rx_idx > 0) {
            rx_idx--;
            rx_buf[rx_idx] = '\0';
            cmd_respond(" \b");  /* Erase echoed char */
        }
        return;
    }

    /* Regular character */
    if (rx_idx < CMD_BUF_SIZE - 1) {
        rx_buf[rx_idx++] = c;
    }
}

/**
 * @brief  Process a block of received data (DMA callback).
 *
 * @param  data  Received data buffer.
 * @param  len   Number of bytes.
 */
void debug_cmd_process_buffer(const uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        debug_cmd_process_char((char)data[i]);
    }
}
