#ifndef _SHT3X_H
#define _SHT3X_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" 
{
#endif

bool SHT3x_twi_init(void);

typedef float temp_values_t;                
void SHT3x_read_temperature(temp_values_t *temperature);

typedef float humi_values_t;
void  SHT3x_read_humidity(humi_values_t *humidity);

#ifdef __cplusplus
}
#endif

#endif