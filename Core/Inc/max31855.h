#ifndef MAX31855_H
#define MAX31855_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32h5xx_hal.h"

typedef struct {
    float thermocouple_temp_c;
    float internal_temp_c;
    uint8_t fault;
    uint8_t scv;
    uint8_t scg;
    uint8_t oc;
} MAX31855_Data;

HAL_StatusTypeDef MAX31855_ReadRaw(uint32_t *raw);
HAL_StatusTypeDef MAX31855_ReadTemperature(MAX31855_Data *out);

#ifdef __cplusplus
}
#endif

#endif /* MAX31855_H */

