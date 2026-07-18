/**
 * @file    ethercat.c
 * @brief   EtherCAT slave stack placeholder for future upgrade path
 * @details Provides ESC (EtherCAT Slave Controller) register interface
 *          and process data mapping for future migration from CAN to
 *          EtherCAT.  Currently a placeholder / skeleton.
 *
 *          Process data:
 *            TxPDO: status word (2B), actual position (4B),
 *                   actual torque (2B), digital inputs (2B)
 *            RxPDO: control word (2B), target position (4B),
 *                   target torque (2B), digital outputs (2B)
 *
 *          Distributed Clock (DC) sync support.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

/* ------------------------------------------------------------------ */
/*  ESC register map (standard EtherCAT Slave Information Interface)   */
/* ------------------------------------------------------------------ */

/* Register addresses (0x0000 - 0x0FFF) */
#define EC_REG_TYPE                 0x0000
#define EC_REG_REVISION             0x0002
#define EC_REG_BUILD                0x0004
#define EC_REG_FMMU_0               0x0600
#define EC_REG_SM_0                 0x0800
#define EC_REG_DL_STATUS            0x0110
#define EC_REG_AL_STATUS            0x0130
#define EC_REG_AL_CODE              0x0134
#define EC_REG_RUN_LED_OVERRIDE     0x0138
#define EC_REG_ERR_LED_OVERRIDE     0x0139

/* AL (Application Layer) state machine */
#define EC_AL_STATE_INIT            0x01
#define EC_AL_STATE_PREOP           0x02
#define EC_AL_STATE_SAFEOP          0x04
#define EC_AL_STATE_OP              0x08
#define EC_AL_STATE_ACK             0x10
#define EC_AL_STATE_ERROR           0x20

/* ------------------------------------------------------------------ */
/*  Process Data Object mapping                                        */
/* ------------------------------------------------------------------ */

/* TxPDO (Slave -> Master) */
typedef struct __attribute__((packed)) {
    uint16_t status_word;        /* bit0=ready, bit1=enable, bit2=fault */
    int32_t  actual_position;    /* encoder counts */
    int16_t  actual_torque;      /* Nm * 1000 */
    uint16_t digital_inputs;     /* bitmapped */
} ec_txpdo_t;

/* RxPDO (Master -> Slave) */
typedef struct __attribute__((packed)) {
    uint16_t control_word;       /* bit0=enable, bit1=reset, bit2=halt */
    int32_t  target_position;    /* encoder counts */
    int16_t  target_torque;      /* Nm * 1000 */
    uint16_t digital_outputs;    /* bitmapped */
} ec_rxpdo_t;

/* ------------------------------------------------------------------ */
/*  ESC context                                                        */
/* ------------------------------------------------------------------ */
typedef struct {
    /* AL state */
    uint8_t  al_state;
    uint8_t  al_control;         /* requested state from master */
    uint32_t al_status_code;

    /* ESC registers (simplified) */
    uint16_t type;
    uint16_t revision;
    uint16_t build;

    /* Sync Manager configuration */
    uint8_t  sm_channel_count;
    uint32_t sm_start_addr[4];
    uint32_t sm_length[4];

    /* Process data */
    ec_txpdo_t txpdo;
    ec_rxpdo_t rxpdo;

    /* Distributed Clock */
    int64_t  dc_system_time_ns;
    uint32_t dc_drift_ns;

    /* State */
    bool     initialized;
} ec_ctx_t;

static ec_ctx_t g_ec;

/* ------------------------------------------------------------------ */
/*  ESC register read/write (stub — replaced by hardware ESC access)   */
/* ------------------------------------------------------------------ */

static uint32_t ec_read_reg(uint16_t addr)
{
    /* STUB: read ESC register via SPI/eSCL interface */
    (void)addr;
    return 0;
}

static void ec_write_reg(uint16_t addr, uint32_t value)
{
    /* STUB: write ESC register */
    (void)addr; (void)value;
}

/* ------------------------------------------------------------------ */
/*  AL State Machine                                                   */
/* ------------------------------------------------------------------ */

static const char *ec_al_state_name(uint8_t state)
{
    switch (state) {
    case EC_AL_STATE_INIT:   return "INIT";
    case EC_AL_STATE_PREOP:  return "PREOP";
    case EC_AL_STATE_SAFEOP: return "SAFEOP";
    case EC_AL_STATE_OP:     return "OP";
    case EC_AL_STATE_ERROR:  return "ERROR";
    default:                 return "UNKNOWN";
    }
}

static int ec_al_state_transition(uint8_t target_state)
{
    /* Validate state transitions per ETG.1000 */
    static const uint8_t valid_transitions[4][4] = {
        /* from\to: INIT PREOP SAFEOP OP */
        /* INIT   */ {1, 1, 0, 0},
        /* PREOP  */ {1, 1, 1, 0},
        /* SAFEOP */ {1, 0, 1, 1},
        /* OP     */ {1, 0, 0, 1},
    };

    uint8_t current = g_ec.al_state;
    int from = 0, to = 0;

    /* Map state to index */
    while (current > 1) { current >>= 1; from++; }
    while (target_state > 1) { target_state >>= 1; to++; }

    if (!valid_transitions[from][to]) {
        fprintf(stderr, "[ECAT] invalid state transition: %s -> %s\n",
                ec_al_state_name(g_ec.al_state),
                ec_al_state_name(target_state));
        g_ec.al_status_code = 0x001A; /* Invalid state change */
        return -EINVAL;
    }

    /* Execute transition */
    switch (target_state) {
    case EC_AL_STATE_INIT:
        ec_write_reg(EC_REG_AL_STATUS, EC_AL_STATE_INIT);
        break;
    case EC_AL_STATE_PREOP:
        /* Configure Sync Manager channels */
        ec_write_reg(EC_REG_AL_STATUS, EC_AL_STATE_PREOP);
        break;
    case EC_AL_STATE_SAFEOP:
        /* Enable inputs (outputs remain off) */
        ec_write_reg(EC_REG_AL_STATUS, EC_AL_STATE_SAFEOP);
        break;
    case EC_AL_STATE_OP:
        /* Enable outputs */
        ec_write_reg(EC_REG_AL_STATUS, EC_AL_STATE_OP);
        break;
    default:
        return -EINVAL;
    }

    g_ec.al_state = target_state;
    g_ec.al_status_code = 0x0000;

    fprintf(stderr, "[ECAT] AL state: %s\n", ec_al_state_name(target_state));
    return 0;
}

/* ------------------------------------------------------------------ */
/*  Process Data Interface                                             */
/* ------------------------------------------------------------------ */

/**
 * ec_update_txpdo - update TxPDO with current drive data for master
 */
void ec_update_txpdo(uint16_t status, int32_t position,
                      int16_t torque, uint16_t digital_in)
{
    g_ec.txpdo.status_word      = status;
    g_ec.txpdo.actual_position  = position;
    g_ec.txpdo.actual_torque    = torque;
    g_ec.txpdo.digital_inputs   = digital_in;

    /* Trigger TxPDO transfer via ESC */
    /* STUB: set event register, SM2 update */
}

/**
 * ec_get_rxpdo - read latest RxPDO from master
 */
void ec_get_rxpdo(uint16_t *control_word, int32_t *target_position,
                   int16_t *target_torque, uint16_t *digital_out)
{
    if (control_word)    *control_word    = g_ec.rxpdo.control_word;
    if (target_position) *target_position = g_ec.rxpdo.actual_position;
    if (target_torque)   *target_torque   = g_ec.rxpdo.actual_torque;
    if (digital_out)     *digital_out     = g_ec.rxpdo.digital_outputs;
}

/* ------------------------------------------------------------------ */
/*  Distributed Clock                                                  */
/* ------------------------------------------------------------------ */

/**
 * ec_dc_sync - synchronize to distributed clock
 * @system_time_ns: DC system time from master
 *
 * Returns drift in ns (for compensation).
 */
int64_t ec_dc_sync(int64_t system_time_ns)
{
    int64_t drift = system_time_ns - g_ec.dc_system_time_ns;
    g_ec.dc_system_time_ns = system_time_ns;
    g_ec.dc_drift_ns = (uint32_t)(drift < 0 ? -drift : drift);
    return drift;
}

/**
 * ec_dc_get_drift_ns - get accumulated clock drift
 */
uint32_t ec_dc_get_drift_ns(void)
{
    return g_ec.dc_drift_ns;
}

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

int ec_init(void)
{
    memset(&g_ec, 0, sizeof(g_ec));

    /* ESC identity */
    g_ec.type     = 0x0001; /* vendor-specific */
    g_ec.revision = 0x0001;
    g_ec.build    = 0x2025;

    ec_write_reg(EC_REG_TYPE,     g_ec.type);
    ec_write_reg(EC_REG_REVISION, g_ec.revision);
    ec_write_reg(EC_REG_BUILD,    g_ec.build);

    g_ec.al_state = EC_AL_STATE_INIT;
    ec_write_reg(EC_REG_AL_STATUS, EC_AL_STATE_INIT);

    /* Set DL status (link status) */
    ec_write_reg(EC_REG_DL_STATUS, 0x000C); /* port0+port1 link detected */

    g_ec.initialized = true;
    fprintf(stderr, "[ECAT] initialized, state=INIT\n");
    return 0;
}

/**
 * ec_process - main processing function, called from control loop
 *
 * Handles:
 *   - AL state machine transitions requested by master
 *   - Process data exchange (if in OP state)
 *   - Distributed Clock sync
 */
int ec_process(void)
{
    if (!g_ec.initialized) return -EPERM;

    /* Check for AL control register change (master requesting state) */
    uint8_t al_control = (uint8_t)ec_read_reg(EC_REG_AL_STATUS + 2); /* AL Control */
    if (al_control != g_ec.al_control && al_control != 0) {
        g_ec.al_control = al_control;
        ec_al_state_transition(al_control);
    }

    /* Check for error */
    if (ec_read_reg(EC_REG_AL_STATUS) & EC_AL_STATE_ERROR) {
        fprintf(stderr, "[ECAT] AL error state\n");
        return -EIO;
    }

    /* In OP state, process data exchange */
    if (g_ec.al_state == EC_AL_STATE_OP) {
        /* Read RxPDO from ESC memory */
        /* STUB: memcpy from ESC SM buffer */
    }

    return 0;
}

/**
 * ec_get_al_state - get current AL state
 */
uint8_t ec_get_al_state(void)
{
    return g_ec.al_state;
}

/**
 * ec_get_al_status_code - get last AL status code
 */
uint32_t ec_get_al_status_code(void)
{
    return g_ec.al_status_code;
}

void ec_shutdown(void)
{
    /* Request INIT state */
    ec_al_state_transition(EC_AL_STATE_INIT);
    g_ec.initialized = false;
    fprintf(stderr, "[ECAT] shutdown\n");
}

/* ------------------------------------------------------------------ */
/*  Self-test                                                          */
/* ------------------------------------------------------------------ */
#ifdef TEST_ECAT
int main(void)
{
    ec_init();

    /* Simulate state transitions */
    ec_al_state_transition(EC_AL_STATE_PREOP);
    ec_al_state_transition(EC_AL_STATE_SAFEOP);
    ec_al_state_transition(EC_AL_STATE_OP);

    /* Update process data */
    ec_update_txpdo(0x0001, 12345, 500, 0x0000);

    /* DC sync */
    ec_dc_sync(1000000000LL);
    printf("DC drift: %u ns\n", ec_dc_get_drift_ns());

    printf("AL state: %s\n", ec_al_state_name(ec_get_al_state()));

    ec_shutdown();
    return 0;
}
#endif /* TEST_ECAT */
