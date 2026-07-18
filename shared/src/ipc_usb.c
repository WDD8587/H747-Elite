/**
 * @file    ipc_usb.c
 * @brief   USB HS CDC ACM transport (STM32H747 Device -> RK3566 Host).
 *
 * Uses USB_OTG_FS with internal FS PHY (12 Mbps, no ULPI chip needed).
 * Appears as /dev/ttyACM0 on RK3566 Linux.
 *
 * Endpoints:
 *   EP0 — Control (enumeration)
 *   EP1 — BULK IN  (STM32 -> RK3566): l1_report_t
 *   EP2 — BULK OUT (RK3566 -> STM32): l2_cmd_t
 *   EP3 — INT IN   (CDC notifications: serial state)
 *
 * Upgrade path to true HS (480 Mbps): add USB3320 ULPI PHY on PB5/PB13/PC0/etc,
 * change PCD instance to USB_OTG_HS, recompile. Descriptor code is unchanged.
 */

#include "stm32h7xx_hal.h"
#include "ipc_proto.h"
#include "ipc_transport.h"
#include "m7_config.h"
#include <string.h>

/* ---- PCD handle ---- */
static PCD_HandleTypeDef gPcd;
static volatile bool gUsbConnected = false;
static volatile bool gUsbConfigured = false;

/* ---- TX ring (reports to RK3566) ---- */
static uint8_t  gTxRing[4][sizeof(l1_report_t)] __attribute__((aligned(32)));
static volatile uint8_t gTxHead = 0;
static volatile uint8_t gTxTail = 0;
static volatile bool    gTxBusy = false;

/* ---- RX ring (commands from RK3566) ---- */
static l2_cmd_t gCmdRing[4];
static volatile uint8_t gCmdHead = 0;
static volatile uint8_t gCmdTail = 0;

/* ---- CDC line coding (for Linux CDC ACM compatibility) ---- */
static uint8_t gLineCoding[7] = {
    0x00, 0xC2, 0x01, 0x00,  /* dwDTERate = 115200 (ignored for USB) */
    0x00,                       /* bCharFormat: 1 stop bit */
    0x00,                       /* bParityType: none */
    0x08                        /* bDataBits: 8 */
};

/* ---- USB Descriptors ---- */

static const uint8_t usb_dev_desc[18] = {
    0x12,                       /* bLength */
    0x01,                       /* bDescriptorType: DEVICE */
    0x00, 0x02,                 /* bcdUSB 2.0 */
    0x02,                       /* bDeviceClass: CDC */
    0x00,                       /* bDeviceSubClass */
    0x00,                       /* bDeviceProtocol */
    64,                         /* bMaxPacketSize0 */
    0x83, 0x04,                 /* idVendor 0x0483 (STMicro) */
    0x40, 0x57,                 /* idProduct 0x5740 (H747 CDC) */
    0x00, 0x01,                 /* bcdDevice 1.00 */
    0x01, 0x02, 0x03,           /* iManufacturer, iProduct, iSerialNumber */
    0x01                        /* bNumConfigurations */
};

/* CDC ACM configuration descriptor: 67 bytes */
static const uint8_t usb_cfg_desc[67] = {
    /* Config */
    0x09, 0x02, 67, 0x00, 0x02, 0x01, 0x00, 0xC0, 0x32,
    /* Interface 0 (Comm): header, call mgmt, ACM, union */
    0x09, 0x04, 0x00, 0x00, 0x01, 0x02, 0x02, 0x01, 0x00,
    0x05, 0x24, 0x00, 0x10, 0x01,
    0x05, 0x24, 0x01, 0x01, 0x01,
    0x04, 0x24, 0x02, 0x02,
    0x05, 0x24, 0x06, 0x00, 0x01,
    /* INT IN EP3 (notifications) */
    0x07, 0x05, 0x83, 0x03, 0x08, 0x00, 0x10,
    /* Interface 1 (Data) */
    0x09, 0x04, 0x01, 0x00, 0x02, 0x0A, 0x00, 0x00, 0x00,
    /* BULK IN EP1 */
    0x07, 0x05, 0x81, 0x02, 64, 0x00, 0x00,
    /* BULK OUT EP2 */
    0x07, 0x05, 0x02, 0x02, 64, 0x00, 0x00,
};

static const uint8_t usb_langid[4]   = {0x04, 0x03, 0x09, 0x04};  /* en-US */
static const uint8_t usb_str_manuf[18] = {
    0x12, 0x03, 'S',0,'T',0,'M',0,'3',0,'2',0,'-',0,'H',0,'7',0,'4',0,'7',0
};
static const uint8_t usb_str_product[42] = {
    0x2A, 0x03,
    'H',0,'7',0,'4',0,'7',0,' ',0,
    'E',0,'l',0,'i',0,'t',0,'e',0,' ',0,
    'C',0,'D',0,'C',0,' ',0,
    'I',0,'P',0,'C',0
};
static const uint8_t usb_str_serial[18] = {
    0x12, 0x03, '0',0,'0',0,'0',0,'1',0,'2',0,'3',0,'4',0,'5',0
};

/* ---- PCD Callbacks ---- */

static void cdc_setup_handler(PCD_HandleTypeDef *hpcd)
{
    uint8_t buf[64];
    uint16_t len = 0;
    USB_Setup_TypeDef setup = {0};

    HAL_PCD_EP_Receive(hpcd, 0x00, buf, 8);
    setup.bmRequestType.bitmap = buf[0];
    setup.bRequest            = buf[1];
    setup.wValue.w            = *(uint16_t *)(buf + 2);
    setup.wIndex.w            = *(uint16_t *)(buf + 4);
    setup.wLength.w           = *(uint16_t *)(buf + 6);
    HAL_PCD_EP_SetStall(hpcd, 0x00);

    uint8_t dir = setup.bmRequestType.bitmap >> 7;

    /* Standard request to interface */
    if ((setup.bmRequestType.bitmap & 0x60) == 0x20) {
        switch (setup.bwValue.w & 0xFF) {
        case 0xFE:  /* CDC: SET_LINE_CODING */
            if (setup.bmRequestType.bitmap == 0x21) {
                HAL_PCD_EP_Receive(hpcd, 0x00, gLineCoding,
                                   setup.wLength.w & 0x7F);
                return;
            }
            break;
        case 0xFF:  /* CDC: SET_CONTROL_LINE_STATE */
            gUsbConnected = (setup.wValue.w & 0x01) != 0;
            HAL_PCD_EP_Transmit(hpcd, 0x00, NULL, 0);
            return;
        }
    }

    /* GET_DESCRIPTOR */
    if (setup.bmRequestType.bitmap == 0x80 && setup.bRequest == 0x06) {
        uint8_t desc_type = setup.wValue.w >> 8;
        uint8_t desc_idx  = setup.wValue.w & 0xFF;
        const uint8_t *ptr = NULL;

        switch (desc_type) {
        case 0x01: ptr = usb_dev_desc;  len = 18; break;
        case 0x02: ptr = usb_cfg_desc;  len = 67; break;
        case 0x03:
            if (desc_idx == 0) { ptr = usb_langid;    len = 4;  }
            if (desc_idx == 1) { ptr = usb_str_manuf;  len = 18; }
            if (desc_idx == 2) { ptr = usb_str_product; len = 42; }
            if (desc_idx == 3) { ptr = usb_str_serial; len = 18; }
            break;
        }

        if (ptr) {
            if (setup.wLength.w < len) len = setup.wLength.w;
            HAL_PCD_EP_Transmit(hpcd, 0x00, (uint8_t *)ptr, len);
            return;
        }
    }

    /* SET_CONFIGURATION */
    if (setup.bmRequestType.bitmap == 0x00 && setup.bRequest == 0x09) {
        gUsbConfigured = (setup.wValue.w & 0xFF) != 0;
        if (gUsbConfigured) {
            HAL_PCD_EP_Open(hpcd, 0x81, 64, 0x02);  /* BULK IN  EP1 */
            HAL_PCD_EP_Open(hpcd, 0x02, 64, 0x02);  /* BULK OUT EP2 */
            HAL_PCD_EP_Open(hpcd, 0x83, 8,  0x03);  /* INT IN   EP3 */
            HAL_PCD_EP_Receive(hpcd, 0x02, (uint8_t *)&gCmdRing[gCmdHead],
                               sizeof(l2_cmd_t));
        } else {
            HAL_PCD_EP_Close(hpcd, 0x81);
            HAL_PCD_EP_Close(hpcd, 0x02);
            HAL_PCD_EP_Close(hpcd, 0x83);
        }
        HAL_PCD_EP_Transmit(hpcd, 0x00, NULL, 0);
        return;
    }

    /* Default: ACK zero-length */
    HAL_PCD_EP_Transmit(hpcd, 0x00, NULL, 0);
}

/* ---- Data OUT callback: RK3566 -> STM32 (l2_cmd_t) ---- */
static void cdc_data_out(PCD_HandleTypeDef *hpcd, uint8_t ep)
{
    if (ep != 0x02) return;

    uint8_t next = (gCmdHead + 1) & 3;
    if (next != gCmdTail) {
        l2_cmd_t *cmd = &gCmdRing[gCmdHead];
        if (cmd->head == IPC_HEAD_RK) {
            gCmdHead = next;
        }
    }

    /* Re-arm OUT endpoint for next command */
    HAL_PCD_EP_Receive(hpcd, 0x02, (uint8_t *)&gCmdRing[gCmdHead],
                       sizeof(l2_cmd_t));
}

/* ---- Data IN callback: STM32 -> RK3566 TX complete ---- */
static void cdc_data_in(PCD_HandleTypeDef *hpcd, uint8_t ep)
{
    (void)hpcd;
    if (ep == 0x81) {
        gTxBusy = false;
        gTxTail = (gTxTail + 1) & 3;
    }
}

static void cdc_reset(PCD_HandleTypeDef *hpcd)
{
    (void)hpcd;
    gUsbConfigured = false;
    gUsbConnected  = false;
    gTxHead = 0; gTxTail = 0; gTxBusy = false;
    gCmdHead = 0; gCmdTail = 0;
}

static void cdc_connect(PCD_HandleTypeDef *hpcd)  { (void)hpcd; }
static void cdc_disconnect(PCD_HandleTypeDef *hpcd) {
    (void)hpcd;
    gUsbConnected  = false;
    gUsbConfigured = false;
}
static void cdc_suspend(PCD_HandleTypeDef *hpcd) { (void)hpcd; }
static void cdc_resume(PCD_HandleTypeDef *hpcd)  { (void)hpcd; }
static void cdc_SOF(PCD_HandleTypeDef *hpcd)     { (void)hpcd; }
static void cdc_iso_incomplete(PCD_HandleTypeDef *hpcd, uint8_t ep) {
    (void)hpcd; (void)ep;
}

/* ---- USB Init ---- */

static int usb_init(void)
{
    /* GPIO: USB FS on PA11 (DM), PA12 (DP) */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = 10;

    gpio.Pin = GPIO_PIN_11; HAL_GPIO_Init(GPIOA, &gpio);  /* DM */
    gpio.Pin = GPIO_PIN_12; HAL_GPIO_Init(GPIOA, &gpio);  /* DP */

    /* VBUS sense (optional): PA9 AF10, for now input with pull-down */
    gpio.Mode      = GPIO_MODE_INPUT;
    gpio.Pull      = GPIO_PULLDOWN;
    gpio.Alternate = 0;
    gpio.Pin       = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* ---- USB PCD init ---- */
    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();

    memset(&gPcd, 0, sizeof(gPcd));
    gPcd.Instance = USB_OTG_FS;
    gPcd.Init.dev_endpoints    = 4;
    gPcd.Init.speed            = PCD_SPEED_FULL;
    gPcd.Init.ep0_mps          = 64;
    gPcd.Init.phy_itface       = PCD_PHY_EMBEDDED;
    gPcd.Init.Sof_enable       = 0;
    gPcd.Init.low_power_enable = 0;
    gPcd.Init.lpm_enable       = 0;
    gPcd.Init.battery_charging_enable = 0;
    if (HAL_PCD_Init(&gPcd) != HAL_OK) return -1;

    /* Register callbacks */
    HAL_PCDEx_SetRxFiFo(&gPcd, 0x80);
    HAL_PCDEx_SetTxFiFo(&gPcd, 0, 0x40);
    HAL_PCDEx_SetTxFiFo(&gPcd, 1, 0x80);  /* EP1 BULK IN */
    HAL_PCDEx_SetTxFiFo(&gPcd, 2, 0x20);  /* EP2 BULK OUT doesn't need tx fifo */
    HAL_PCDEx_SetTxFiFo(&gPcd, 3, 0x10);  /* EP3 INT IN */

    HAL_PCD_RegisterCallback(&gPcd, HAL_PCD_SETUP_STAGE_CB_ID,
                             (void (*)(PCD_HandleTypeDef *))cdc_setup_handler);
    HAL_PCD_RegisterDataOutStageCallback(&gPcd, cdc_data_out);
    HAL_PCD_RegisterDataInStageCallback(&gPcd, cdc_data_in);
    HAL_PCD_RegisterResetCallback(&gPcd, cdc_reset);
    HAL_PCD_RegisterConnectCallback(&gPcd, cdc_connect);
    HAL_PCD_RegisterDisconnectCallback(&gPcd, cdc_disconnect);
    HAL_PCD_RegisterSuspendCallback(&gPcd, cdc_suspend);
    HAL_PCD_RegisterResumeCallback(&gPcd, cdc_resume);
    HAL_PCD_RegisterSOFCallback(&gPcd, cdc_SOF);
    HAL_PCD_RegisterIsoOutIncpltCallback(&gPcd, cdc_iso_incomplete);
    HAL_PCD_RegisterIsoInIncpltCallback(&gPcd, cdc_iso_incomplete);

    HAL_NVIC_SetPriority(OTG_FS_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(OTG_FS_IRQn);

    HAL_PCD_Start(&gPcd);
    return 0;
}

/* ---- Public Transport API ---- */

static int usb_send(const uint8_t *buf, uint16_t len)
{
    if (!gUsbConfigured || gTxBusy) return -1;
    if (len > sizeof(l1_report_t)) return -2;

    uint8_t next = (gTxHead + 1) & 3;
    if (next == gTxTail) return -3;  /* ring full */

    memcpy(gTxRing[gTxHead], buf, len);
    gTxHead = next;

    gTxBusy = true;
    HAL_PCD_EP_Transmit(&gPcd, 0x81, gTxRing[gTxTail],
                        sizeof(l1_report_t));
    return (int)len;
}

static int usb_recv(uint8_t *buf, uint16_t capacity, uint32_t timeout_ms)
{
    (void)timeout_ms;
    if (!gUsbConfigured) return 0;

    uint8_t tail = gCmdTail;
    if (tail != gCmdHead) {
        if (capacity >= sizeof(l2_cmd_t)) {
            memcpy(buf, &gCmdRing[tail], sizeof(l2_cmd_t));
        }
        gCmdTail = (tail + 1) & 3;
        return sizeof(l2_cmd_t);
    }
    return 0;
}

static bool usb_ready(void)
{
    return gUsbConfigured && !gTxBusy;
}

static void usb_process(void)
{
    /* PCD interrupt-driven; nothing to poll. */
}

static void usb_deinit(void)
{
    HAL_PCD_Stop(&gPcd);
    HAL_PCD_DeInit(&gPcd);
    __HAL_RCC_USB_OTG_FS_CLK_DISABLE();
    gUsbConfigured = false;
    gUsbConnected  = false;
}

const ipc_transport_t ipc_transport_usb = {
    .init    = usb_init,
    .send    = usb_send,
    .recv    = usb_recv,
    .ready   = usb_ready,
    .process = usb_process,
    .type    = IPC_TRANSPORT_USB,
    .name    = "USB-CDC-ACM"
};
