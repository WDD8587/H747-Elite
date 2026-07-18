/**
 * ui.c
 * Factory production test UI for RK3566 LCD.
 *
 * Displays a test list with per-test status (RUNNING/PASS/FAIL)
 * and elapsed time. Shows overall PASS/FAIL at the bottom.
 * Supports barcode scan input for serial number.
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>

/* ------------------------------------------------------------------ */
/*  Configuration                                                     */
/* ------------------------------------------------------------------ */

#define MAX_TESTS           16
#define MAX_TEST_NAME_LEN   32
#define MAX_SERIAL_LEN      32
#define LCD_WIDTH           800
#define LCD_HEIGHT          480
#define LINE_HEIGHT         22
#define LEFT_MARGIN         20
#define CHECKBOX_SIZE       16

/* ------------------------------------------------------------------ */
/*  Types                                                             */
/* ------------------------------------------------------------------ */

typedef enum {
    TEST_IDLE = 0,
    TEST_RUNNING,
    TEST_PASS,
    TEST_FAIL
} test_status_t;

typedef struct {
    char            name[MAX_TEST_NAME_LEN];
    test_status_t   status;
    uint32_t        start_time_ms;
    uint32_t        elapsed_ms;
    int             checked;            /* checkbox state */
    int             auto_run;           /* run automatically */
} test_item_t;

typedef struct {
    test_item_t     tests[MAX_TESTS];
    int             num_tests;
    int             current_test;
    int             all_pass;           /* overall result */
    int             production_complete;
    char            serial_number[MAX_SERIAL_LEN];
    int             serial_entered;
    uint32_t        start_time_ms;
} production_ui_t;

/* Color definitions (RGB565 for simple framebuffer) */
#define COLOR_BLACK     0x0000
#define COLOR_WHITE     0xFFFF
#define COLOR_GREEN     0x07E0
#define COLOR_RED       0xF800
#define COLOR_YELLOW    0xFFE0
#define COLOR_BLUE      0x001F
#define COLOR_GRAY      0x8410
#define COLOR_DARK_GRAY 0x4208

/* Framebuffer simulation */
static uint16_t fb[LCD_HEIGHT][LCD_WIDTH];
static int fb_initialized = 0;

/* ------------------------------------------------------------------ */
/*  Low-level drawing (simulated for test)                            */
/* ------------------------------------------------------------------ */

void ui_fb_clear(uint16_t color)
{
    for (int y = 0; y < LCD_HEIGHT; y++)
        for (int x = 0; x < LCD_WIDTH; x++)
            fb[y][x] = color;
    fb_initialized = 1;
}

void ui_draw_pixel(int x, int y, uint16_t color)
{
    if (x >= 0 && x < LCD_WIDTH && y >= 0 && y < LCD_HEIGHT)
        fb[y][x] = color;
}

void ui_draw_rect(int x, int y, int w, int h, uint16_t color)
{
    for (int row = y; row < y + h && row < LCD_HEIGHT; row++)
        for (int col = x; col < x + w && col < LCD_WIDTH; col++)
            fb[row][col] = color;
}

void ui_draw_checkbox(int x, int y, int checked, uint16_t color)
{
    /* Draw border */
    for (int dy = 0; dy < CHECKBOX_SIZE; dy++) {
        for (int dx = 0; dx < CHECKBOX_SIZE; dx++) {
            if (dy == 0 || dy == CHECKBOX_SIZE - 1 ||
                dx == 0 || dx == CHECKBOX_SIZE - 1) {
                ui_draw_pixel(x + dx, y + dy, COLOR_WHITE);
            } else if (checked && dx > 2 && dx < CHECKBOX_SIZE - 3 &&
                       dy > 2 && dy < CHECKBOX_SIZE - 3) {
                ui_draw_pixel(x + dx, y + dy, color);
            } else {
                ui_draw_pixel(x + dx, y + dy, COLOR_BLACK);
            }
        }
    }
}

/* Simple 8x8 bitmap font glyph rendering (simplified) */
static const uint8_t font8x8_basic[128][8] = {
    {' ', 0,0,0,0,0,0,0,0},
    /* Only a subset implemented for the test UI */
};

void ui_draw_char(int x, int y, char c, uint16_t color)
{
    (void)x; (void)y; (void)c; (void)color;
    /* In real implementation, renders 8x8 glyph from font table */
}

void ui_draw_text(int x, int y, const char *text, uint16_t color)
{
    int cx = x;
    while (*text) {
        ui_draw_char(cx, y, *text, color);
        cx += 8;
        text++;
    }
}

/* ------------------------------------------------------------------ */
/*  UI state management                                               */
/* ------------------------------------------------------------------ */

static production_ui_t g_ui;

void ui_init(void)
{
    memset(&g_ui, 0, sizeof(g_ui));
    ui_fb_clear(COLOR_BLACK);
}

void ui_add_test(const char *name, int auto_run)
{
    if (g_ui.num_tests >= MAX_TESTS) return;
    test_item_t *t = &g_ui.tests[g_ui.num_tests++];
    strncpy(t->name, name, MAX_TEST_NAME_LEN - 1);
    t->name[MAX_TEST_NAME_LEN - 1] = '\0';
    t->status    = TEST_IDLE;
    t->checked   = 1;
    t->auto_run  = auto_run;
    t->start_time_ms = 0;
    t->elapsed_ms    = 0;
}

void ui_set_serial(const char *serial)
{
    strncpy(g_ui.serial_number, serial, MAX_SERIAL_LEN - 1);
    g_ui.serial_number[MAX_SERIAL_LEN - 1] = '\0';
    g_ui.serial_entered = 1;
}

void ui_start_test(int test_idx)
{
    if (test_idx < 0 || test_idx >= g_ui.num_tests) return;
    test_item_t *t = &g_ui.tests[test_idx];
    t->status       = TEST_RUNNING;
    t->start_time_ms = 0;  /* set by timer in real app */
    g_ui.current_test = test_idx;
}

void ui_complete_test(int test_idx, int passed, uint32_t elapsed_ms)
{
    if (test_idx < 0 || test_idx >= g_ui.num_tests) return;
    test_item_t *t = &g_ui.tests[test_idx];
    t->status     = passed ? TEST_PASS : TEST_FAIL;
    t->elapsed_ms = elapsed_ms;
}

void ui_set_overall_result(int passed)
{
    g_ui.all_pass = passed;
}

/* ------------------------------------------------------------------ */
/*  Rendering                                                         */
/* ------------------------------------------------------------------ */

static const char *status_text(test_status_t s)
{
    switch (s) {
    case TEST_IDLE:    return "WAITING";
    case TEST_RUNNING: return "RUNNING";
    case TEST_PASS:    return "PASS";
    case TEST_FAIL:    return "FAIL";
    default:           return "?";
    }
}

static uint16_t status_color(test_status_t s)
{
    switch (s) {
    case TEST_RUNNING: return COLOR_YELLOW;
    case TEST_PASS:    return COLOR_GREEN;
    case TEST_FAIL:    return COLOR_RED;
    default:           return COLOR_GRAY;
    }
}

void ui_render(void)
{
    ui_fb_clear(COLOR_BLACK);

    int y = 10;

    /* Title */
    ui_draw_text(LEFT_MARGIN, y, "PRODUCTION TEST", COLOR_WHITE);
    y += LINE_HEIGHT + 4;

    /* Serial number */
    {
        char buf[48];
        if (g_ui.serial_entered)
            snprintf(buf, sizeof(buf), "S/N: %s", g_ui.serial_number);
        else
            snprintf(buf, sizeof(buf), "S/N: [SCAN BARCODE]");
        ui_draw_text(LEFT_MARGIN, y, buf, g_ui.serial_entered ? COLOR_GREEN : COLOR_YELLOW);
    }
    y += LINE_HEIGHT + 4;

    /* Header line */
    ui_draw_text(LEFT_MARGIN, y, "Test                    Status    Time", COLOR_GRAY);
    y += LINE_HEIGHT;

    /* Test list */
    for (int i = 0; i < g_ui.num_tests; i++) {
        test_item_t *t = &g_ui.tests[i];

        /* Checkbox */
        ui_draw_checkbox(LEFT_MARGIN, y + 3, t->checked,
                         t->checked ? COLOR_GREEN : COLOR_GRAY);

        /* Test name */
        ui_draw_text(LEFT_MARGIN + CHECKBOX_SIZE + 8, y + 3, t->name, COLOR_WHITE);

        /* Status badge */
        {
            char st[16];
            snprintf(st, sizeof(st), "%s", status_text(t->status));
            uint16_t sc = status_color(t->status);
            /* Background rectangle for status */
            int bx = 380;
            int bw = 70;
            ui_draw_rect(bx, y + 1, bw, LINE_HEIGHT - 2, sc & 0x7BEF);
            ui_draw_text(bx + 8, y + 3, st, COLOR_WHITE);
        }

        /* Elapsed time */
        if (t->status != TEST_IDLE) {
            char et[16];
            snprintf(et, sizeof(et), "%u ms", (unsigned)t->elapsed_ms);
            ui_draw_text(480, y + 3, et, COLOR_GRAY);
        }

        /* Highlight current running test */
        if (t->status == TEST_RUNNING) {
            ui_draw_rect(LEFT_MARGIN - 2, y - 1, LCD_WIDTH - LEFT_MARGIN * 2,
                         LINE_HEIGHT, COLOR_YELLOW);
        }

        y += LINE_HEIGHT + 2;
    }

    /* Overall result at bottom */
    y = LCD_HEIGHT - 40;
    ui_draw_rect(0, y, LCD_WIDTH, 40, COLOR_DARK_GRAY);

    if (g_ui.production_complete) {
        const char *result_text = g_ui.all_pass ? "OVERALL: PASS" : "OVERALL: FAIL";
        uint16_t result_color = g_ui.all_pass ? COLOR_GREEN : COLOR_RED;
        ui_draw_text(LCD_WIDTH / 2 - 80, y + 10, result_text, result_color);
    } else if (g_ui.current_test > 0 || g_ui.serial_entered) {
        ui_draw_text(LCD_WIDTH / 2 - 80, y + 10, "TESTING IN PROGRESS...", COLOR_YELLOW);
    }
}

/* ------------------------------------------------------------------ */
/*  High-level test runner                                            */
/* ------------------------------------------------------------------ */

/* Simulated test function pointer */
typedef int (*test_fn_t)(void);

typedef struct {
    const char *name;
    test_fn_t   fn;
} registered_test_t;

static registered_test_t test_registry[MAX_TESTS];
static int num_registered = 0;

void ui_register_test(const char *name, test_fn_t fn)
{
    if (num_registered >= MAX_TESTS) return;
    test_registry[num_registered].name = name;
    test_registry[num_registered].fn   = fn;
    ui_add_test(name, 1);
    num_registered++;
}

int ui_run_all_tests(void)
{
    g_ui.all_pass = 1;
    g_ui.start_time_ms = (uint32_t)clock();

    for (int i = 0; i < g_ui.num_tests; i++) {
        test_item_t *t = &g_ui.tests[i];
        if (!t->checked) continue;

        ui_start_test(i);

        /* Run the test */
        uint32_t test_start = (uint32_t)clock();
        int passed = 0;
        if (i < num_registered && test_registry[i].fn) {
            passed = test_registry[i].fn();
        }
        uint32_t test_end = (uint32_t)clock();
        uint32_t elapsed = test_end - test_start;

        ui_complete_test(i, passed, elapsed);
        if (!passed) g_ui.all_pass = 0;

        ui_render();
    }

    g_ui.production_complete = 1;
    ui_set_overall_result(g_ui.all_pass);
    ui_render();

    return g_ui.all_pass;
}

/* ------------------------------------------------------------------ */
/*  Example built-in factory tests                                    */
/* ------------------------------------------------------------------ */

static int test_motor_cal(void)
{
    /* Simulate motor calibration (100 ms) */
    volatile int delay = 0;
    for (int i = 0; i < 1000000; i++) delay++;
    (void)delay;
    return 1; /* pass */
}

static int test_imu_cal(void)
{
    volatile int delay = 0;
    for (int i = 0; i < 500000; i++) delay++;
    (void)delay;
    return 1;
}

static int test_tof_cal(void)
{
    volatile int delay = 0;
    for (int i = 0; i < 300000; i++) delay++;
    (void)delay;
    return 1;
}

static int test_battery(void)
{
    return 1;
}

static int test_wifi_rf(void)
{
    return 1;
}

static int test_aging(void)
{
    return 1;
}

void ui_register_default_tests(void)
{
    ui_register_test("Motor Cal",  test_motor_cal);
    ui_register_test("IMU Cal",    test_imu_cal);
    ui_register_test("ToF Cal",    test_tof_cal);
    ui_register_test("Battery",    test_battery);
    ui_register_test("WiFi RF",    test_wifi_rf);
    ui_register_test("Aging",      test_aging);
}

/* ------------------------------------------------------------------ */
/*  Entry point for test                                              */
/* ------------------------------------------------------------------ */

#ifdef UNIT_TEST
int main(void)
{
    printf("=== Factory Production UI ===\n\n");

    ui_init();
    ui_register_default_tests();
    ui_set_serial("H747-ELITE-00001");
    ui_render();

    printf("UI initialized with %d tests, serial=%s\n",
           g_ui.num_tests, g_ui.serial_number);
    printf("Running all tests...\n");

    int result = ui_run_all_tests();

    printf("All tests complete. Overall: %s\n", result ? "PASS" : "FAIL");

    /* Verify fb has been written */
    if (fb_initialized)
        printf("Framebuffer: %dx%d, first pixel=0x%04X\n",
               LCD_WIDTH, LCD_HEIGHT, fb[0][0]);

    return result ? 0 : 1;
}
#endif
