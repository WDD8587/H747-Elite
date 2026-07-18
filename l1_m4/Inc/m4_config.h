#ifndef M4_CONFIG_H
#define M4_CONFIG_H

#define M4_CLOCK_MHZ       240
#define TOF_RATE_HZ        100
#define BMS_RATE_HZ        10
#define DOCK_RATE_HZ       10
#define SAFETY_RATE_HZ     100
#define M7_DEAD_MS         50
#define PWM_SHUT_GPIO      GPIOB
#define PWM_SHUT_PIN       GPIO_PIN_0
#define BQ40Z50_I2C_ADDR   0x16
#define DOCK_VOLT_THRESH   19000
#define DOCK_IR_GPIO       GPIOA
#define DOCK_IR_PIN        GPIO_PIN_3
#endif
