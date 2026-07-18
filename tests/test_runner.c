#include <stdio.h>
#include "unity/unity.h"

/* Self-contained test suites */
extern void test_clarke_park_balanced(void);
extern void test_park_at_0(void);
extern void test_svpwm_sectors(void);

extern void test_smbus_readword_valid(void);
extern void test_smbus_pec_mismatch(void);

extern void test_charge_trickle_to_cc(void);
extern void test_charge_fault_ovp(void);

extern void test_safety_m7_alive(void);
extern void test_safety_m7_dead(void);

extern void test_diff_identical_empty(void);
extern void test_diff_roundtrip(void);

extern void test_spi_frame_size(void);
extern void test_spi_pack_roundtrip(void);
extern void test_spi_corrupted_frame_rejected(void);
extern void test_spi_cmd_pack_roundtrip(void);
extern void test_spi_double_buffer_pingpong(void);
extern void test_spi_bad_header_rejected(void);

extern void test_usb_dev_desc_size(void);
extern void test_usb_cfg_desc_size(void);
extern void test_usb_ep_max_packet(void);
extern void test_usb_end_to_end(void);
extern void test_usb_ring_buffer_overflow(void);
extern void test_usb_disconnect_reject(void);

int main(void)
{
    unity_init();
    printf("\n  H747 Elite Unit Tests\n\n");

    printf("[1] Motor FOC Math\n");
    test_clarke_park_balanced();
    test_park_at_0();
    test_svpwm_sectors();

    printf("\n[2] BMS SMBus\n");
    test_smbus_readword_valid();
    test_smbus_pec_mismatch();

    printf("\n[3] BMS Charge\n");
    test_charge_trickle_to_cc();
    test_charge_fault_ovp();

    printf("\n[4] Safety Watchdog\n");
    test_safety_m7_alive();
    test_safety_m7_dead();

    printf("\n[5] OTA Diff\n");
    test_diff_identical_empty();
    test_diff_roundtrip();

    printf("\n[6] SPI IPC Transport\n");
    test_spi_frame_size();
    test_spi_pack_roundtrip();
    test_spi_corrupted_frame_rejected();
    test_spi_cmd_pack_roundtrip();
    test_spi_double_buffer_pingpong();
    test_spi_bad_header_rejected();

    printf("\n[7] USB CDC Transport\n");
    test_usb_dev_desc_size();
    test_usb_cfg_desc_size();
    test_usb_ep_max_packet();
    test_usb_end_to_end();
    test_usb_ring_buffer_overflow();
    test_usb_disconnect_reject();

    unity_print_summary();
    return 0;
}
