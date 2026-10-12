#include "assemble.h"

#include "fdcan.h"
#include "laser.h"
#include "stm32_hal_can.h"

static void laser_can_rx_callback(FDCAN_HandleTypeDef* hcan,
                                  const FDCAN_RxHeaderTypeDef* header,
                                  const uint8_t data[8],
                                  void* user) {
    (void)hcan;
    (void)user;

    if(header == NULL || data == NULL ||
       header->IdType != FDCAN_STANDARD_ID ||
       header->Identifier != 0x601u ||
       header->RxFrameType != FDCAN_DATA_FRAME ||
       header->FDFormat != FDCAN_CLASSIC_CAN ||
       header->DataLength != FDCAN_DLC_BYTES_8) {
        return;
    }
    laser_accept_frame(data, HAL_GetTick());
}

SystemStatus assemble_laser(void) {
    FDCAN_FilterTypeDef filter = {0};

    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0u;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0x601u;
    filter.FilterID2 = 0x7FFu;
    if(HAL_FDCAN_ConfigFilter(&hfdcan3, &filter) != HAL_OK ||
       HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT,
                                    FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK ||
       can_register_rx_callback(&hfdcan3, laser_can_rx_callback, NULL) != STM32_HAL_CAN_OK ||
       can_start(&hfdcan3) != STM32_HAL_CAN_OK) {
        return SYSTEM_STATUS_ERROR;
    }
    return SYSTEM_STATUS_OK;
}
