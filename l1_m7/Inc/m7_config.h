#ifndef M7_CONFIG_H
#define M7_CONFIG_H

/* ---- Robot geometry ---- */
#define WHEEL_RADIUS      0.035f
#define WHEEL_BASE        0.150f

/* ---- Timing ---- */
#define FOC_PWM_FREQ      20000
#define IMU_RATE_HZ       1000
#define BRIDGE_RATE_HZ    100
#define M4_WAKE_TIMEOUT_MS 100

/* ---- IPC Transport selection ---- */
#define IPC_TRANSPORT     IPC_TRANSPORT_SPI  /* UART | SPI | USB */

/* ---- UART IPC ---- */
#define IPC_UART          USART7
#define IPC_UART_BAUD     3000000

/* ---- SPI IPC (Slave) ---- */
#define IPC_SPI           SPI6
#define IPC_SPI_BAUDRATE  20000000UL        /* 20 MHz */
#define IPC_SPI_FRAME     64                /* transaction frame bytes */
#define IPC_SPI_RDY_PORT  GPIOE
#define IPC_SPI_RDY_PIN   GPIO_PIN_1        /* PE1: data-ready IRQ to RK3566 */
/* SPI6 pins: SCK=PG13(AF5), MISO=PG12(AF5), MOSI=PG14(AF5), NSS=PG9(AF5) */
#define IPC_SPI_GPIO_AF   5

/* ---- USB HS CDC ---- */
#define USB_HS_CDC_INTERVAL 1               /* 1ms polling interval */
#define USB_HS_CDC_VCP     "/dev/ttyACM0"   /* Linux CDC ACM device node */

#endif
