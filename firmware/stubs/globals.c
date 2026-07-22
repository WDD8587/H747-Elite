/* CI-only global definitions for symbols extern'd by firmware code.
 * In real hardware these are provided by HAL/CMSIS libraries.
 */
#include "stm32h7xx_hal.h"

UART_HandleTypeDef huart7;
uint32_t SystemCoreClock = 480000000;

/* Stub main for CI build — real main is in main_m7.c (excluded) */
int main(void) { while(1); }

/* FocSetSpeed: defined in foc_isr.c (simplified) or motor/foc_isr.c (detailed).
 * The CI build uses the detailed motor/foc_isr.c; provide a stub here. */
void FocSetSpeed(float vl, float vr) { (void)vl; (void)vr; }
