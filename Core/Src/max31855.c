#include "max31855.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1;

HAL_StatusTypeDef MAX31855_ReadRaw(uint32_t *raw)
{
    uint8_t buf[4] = {0, 0, 0, 0};
    HAL_StatusTypeDef st;

    if (raw == NULL) {
        return HAL_ERROR;
    }

    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
    st = HAL_SPI_Receive(&hspi1, buf, 4, 50);
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

    if (st != HAL_OK) {
        return st;
    }

    *raw = ((uint32_t)buf[0] << 24) |
           ((uint32_t)buf[1] << 16) |
           ((uint32_t)buf[2] << 8)  |
           ((uint32_t)buf[3] << 0);

    return HAL_OK;
}

HAL_StatusTypeDef MAX31855_ReadTemperature(MAX31855_Data *out)
{
    uint32_t v;
    if (out == NULL) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef st = MAX31855_ReadRaw(&v);
    if (st != HAL_OK) {
        return st;
    }

    int32_t tc_raw = (int32_t)(v >> 18);
    if (tc_raw & 0x2000) {
        tc_raw |= ~0x3FFF;
    }
    out->thermocouple_temp_c = (float)tc_raw * 0.25f;

    int32_t cj_raw = (int32_t)((v >> 4) & 0x0FFF);
    if (cj_raw & 0x0800) {
        cj_raw |= ~0x0FFF;
    }
    out->internal_temp_c = (float)cj_raw * 0.0625f;

    out->fault = (uint8_t)((v >> 16) & 0x1);
    out->scv   = (uint8_t)((v >> 2)  & 0x1);
    out->scg   = (uint8_t)((v >> 1)  & 0x1);
    out->oc    = (uint8_t)((v >> 0)  & 0x1);

    return HAL_OK;
}

