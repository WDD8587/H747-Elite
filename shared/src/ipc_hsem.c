#include "ipc_proto.h"
#include "stm32h7xx_hal.h"
#include <string.h>

static l1_odom_t *gOdom = (l1_odom_t *)SRAM3_BASE;

void IPC_Init(void)
{
    __HAL_RCC_HSEM_CLK_ENABLE();
    memset(gOdom, 0, sizeof(l1_odom_t));
    gOdom->magic = 0x4C314C31;
}

void IPC_M7_WriteOdom(const l1_odom_t *odom)
{
    HAL_HSEM_Take(HSEM_ODOM_READY, 0);
    memcpy(gOdom, odom, sizeof(l1_odom_t));
    HAL_HSEM_Release(HSEM_ODOM_READY, 0);
}

void IPC_M4_ReadOdom(l1_odom_t *odom)
{
    HAL_HSEM_Take(HSEM_ODOM_READY, 0);
    memcpy(odom, gOdom, sizeof(l1_odom_t));
}

void IPC_M4_WriteBMS(uint16_t rsoc, uint16_t mv)
{
    gOdom->bms_rsoc = rsoc;
    gOdom->bms_mv   = mv;
    HAL_HSEM_Release(HSEM_BMS_READY, 0);
}

void IPC_M7_Heartbeat(void)
{
    static int cnt = 0;
    if (++cnt >= 5) { cnt = 0; HAL_HSEM_Release(HSEM_M7_HEARTBEAT, 0); }
}
