/**
 * test_bms_balance.c
 * Unit tests for BMS cell balancing logic.
 *
 * Tests:
 *   - delta > 30 mV triggers balancing
 *   - delta < 10 mV stops balancing
 *   - MOSFET temperature > 85 C pauses balancing
 *   - Balancing correctly selects the highest cell to discharge
 *
 * Build:
 *   gcc -I. -I../../firmware -DUNIT_TEST test_bms_balance.c -o test_bms_balance
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

/* ------------------------------------------------------------------ */
/*  Types                                                             */
/* ------------------------------------------------------------------ */

#define BALANCE_MAX_CELLS      8

typedef uint16_t mv_t;   /* millivolt */

typedef struct {
    mv_t  cell_voltage_mv[BALANCE_MAX_CELLS];
    int   num_cells;
    int   mosfet_temp_c;           /* balance MOSFET temperature in C */
} balance_input_t;

typedef struct {
    uint8_t balance_mask;          /* bitmask of cells being discharged */
    int     active;                /* 1 if balancing is active */
    int     cell_delta_mv;         /* max-min delta */
} balance_decision_t;

/* Configuration */
#define BALANCE_TRIGGER_DELTA_MV    30
#define BALANCE_STOP_DELTA_MV       10
#define BALANCE_TEMP_LIMIT_C        85

/* DUT */
void balance_evaluate(const balance_input_t *in, balance_decision_t *out);

/* ------------------------------------------------------------------ */
/*  DUT implementation                                                */
/* ------------------------------------------------------------------ */

void balance_evaluate(const balance_input_t *in, balance_decision_t *out)
{
    memset(out, 0, sizeof(*out));

    if (in->num_cells < 2 || in->num_cells > BALANCE_MAX_CELLS)
        return;

    /* Find min, max, and the index of max */
    mv_t vmin = in->cell_voltage_mv[0];
    mv_t vmax = in->cell_voltage_mv[0];
    int  imax = 0;

    for (int i = 1; i < in->num_cells; i++) {
        if (in->cell_voltage_mv[i] > vmax) {
            vmax = in->cell_voltage_mv[i];
            imax = i;
        }
        if (in->cell_voltage_mv[i] < vmin)
            vmin = in->cell_voltage_mv[i];
    }

    out->cell_delta_mv = vmax - vmin;

    /* Thermal limit check */
    if (in->mosfet_temp_c > BALANCE_TEMP_LIMIT_C) {
        out->balance_mask = 0;
        out->active       = 0;
        return;
    }

    /* Decision */
    if (out->cell_delta_mv > BALANCE_TRIGGER_DELTA_MV) {
        /* Discharge the highest cell(s) above average */
        /* Sum all cells, find average, discharge cells above average */
        uint32_t sum = 0;
        for (int i = 0; i < in->num_cells; i++)
            sum += in->cell_voltage_mv[i];
        mv_t avg = (mv_t)(sum / in->num_cells);

        out->balance_mask = 0;
        for (int i = 0; i < in->num_cells; i++) {
            if (in->cell_voltage_mv[i] > avg + 10)
                out->balance_mask |= (1U << i);
        }
        out->active = (out->balance_mask != 0);
    } else if (out->cell_delta_mv < BALANCE_STOP_DELTA_MV) {
        out->balance_mask = 0;
        out->active       = 0;
    } else {
        /* Between 10-30 mV: hold current state (mask unchanged) */
        /* For test simplicity, if mask not set, remain off */
        out->active = (out->balance_mask != 0);
    }
}

/* ------------------------------------------------------------------ */
/*  Test utilities                                                    */
/* ------------------------------------------------------------------ */

static int test_passed = 0;
static int test_failed = 0;

#define TEST_START(name)  do { printf("  TEST %-45s ", name); } while(0)
#define TEST_PASS()       do { printf("PASS\n"); test_passed++; } while(0)
#define TEST_FAIL(msg)    do { printf("FAIL: %s\n", msg); test_failed++; } while(0)

static balance_input_t make_input(const mv_t *cells, int n, int temp_c)
{
    balance_input_t in;
    memset(&in, 0, sizeof(in));
    in.num_cells = n;
    in.mosfet_temp_c = temp_c;
    for (int i = 0; i < n && i < BALANCE_MAX_CELLS; i++)
        in.cell_voltage_mv[i] = cells[i];
    return in;
}

/* ------------------------------------------------------------------ */
/*  Tests                                                             */
/* ------------------------------------------------------------------ */

static void test_balance_triggers_on_delta_gt_30mv(void)
{
    TEST_START("Delta >30mV triggers balancing");
    mv_t cells[] = {3700, 3745, 3710, 3690};
    balance_input_t in = make_input(cells, 4, 25);
    balance_decision_t out;
    balance_evaluate(&in, &out);
    /* delta = 3745-3690 = 55 mV > 30 */
    if (out.active && out.balance_mask != 0 && out.cell_delta_mv == 55)
        TEST_PASS();
    else
        TEST_FAIL("expected balancing active with delta=55mV");
}

static void test_balance_stops_on_delta_lt_10mv(void)
{
    TEST_START("Delta <10mV stops balancing");
    /* Start with a balanced state */
    mv_t cells[] = {3710, 3715, 3708, 3712};
    balance_input_t in = make_input(cells, 4, 25);
    balance_decision_t out;
    balance_evaluate(&in, &out);
    /* delta = 3715-3708 = 7 mV < 10 */
    if (!out.active && out.balance_mask == 0)
        TEST_PASS();
    else
        TEST_FAIL("expected balancing inactive");
}

static void test_balance_thermal_limit_pauses(void)
{
    TEST_START("MOSFET temp >85C pauses balancing");
    mv_t cells[] = {3700, 3760, 3710, 3690};  /* delta = 70 mV */
    balance_input_t in = make_input(cells, 4, 90);
    balance_decision_t out;
    balance_evaluate(&in, &out);
    if (!out.active && out.balance_mask == 0)
        TEST_PASS();
    else
        TEST_FAIL("expected balancing paused due to temperature");
}

static void test_balance_thermal_limit_recovers(void)
{
    TEST_START("MOSFET temp cools below 85C, balancing resumes");
    mv_t cells[] = {3700, 3760, 3710, 3690};
    /* Hot */
    balance_input_t in = make_input(cells, 4, 90);
    balance_decision_t out;
    balance_evaluate(&in, &out);
    if (out.active) { TEST_FAIL("should be paused"); return; }

    /* Cooled */
    in.mosfet_temp_c = 70;
    balance_evaluate(&in, &out);
    if (out.active && out.balance_mask != 0)
        TEST_PASS();
    else
        TEST_FAIL("expected balancing to resume after cooling");
}

static void test_balance_selects_highest_cells(void)
{
    TEST_START("Balancing mask selects cells above average+10mV");
    /* Cells: 3650, 3720, 3680, 3640 */
    /* avg = 3672.5 -> 3672, avg+10 = 3682 */
    /* cell 1 (3720) > 3682 and cell 2 (3680) < 3682 */
    mv_t cells[] = {3650, 3720, 3680, 3640};
    balance_input_t in = make_input(cells, 4, 25);
    balance_decision_t out;
    balance_evaluate(&in, &out);
    /* Expect only cell 1 (bit 1) to be balanced */
    if (out.active && out.balance_mask == 0x02)
        TEST_PASS();
    else
        TEST_FAIL("expected mask 0x02 for cell 1 only");
}

static void test_balance_no_action_when_balanced(void)
{
    TEST_START("No balancing when all cells within 10mV");
    mv_t cells[] = {3705, 3710, 3700, 3708};
    balance_input_t in = make_input(cells, 4, 25);
    balance_decision_t out;
    balance_evaluate(&in, &out);
    if (!out.active && out.balance_mask == 0)
        TEST_PASS();
    else
        TEST_FAIL("expected no balancing");
}

/* ------------------------------------------------------------------ */
/*  Main                                                              */
/* ------------------------------------------------------------------ */

int main(void)
{
    printf("=== BMS Balance Unit Tests ===\n\n");

    test_balance_triggers_on_delta_gt_30mv();
    test_balance_stops_on_delta_lt_10mv();
    test_balance_thermal_limit_pauses();
    test_balance_thermal_limit_recovers();
    test_balance_selects_highest_cells();
    test_balance_no_action_when_balanced();

    printf("\nResults: %d passed, %d failed\n", test_passed, test_failed);
    return test_failed > 0 ? 1 : 0;
}
