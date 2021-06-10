#include <stdbool.h>
#include <stdint.h>

#include "SHT3x.h"
#include "boards.h"
#include "nrf_drv_twi.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/*lint ++flb "Enter library region" */

/*TWI instance ID*/
#define TWI_INSTANCE_ID 0

/*SHT3x address*/
#define SHT3x_ADDR 0x44 

/* Single shot sensor read command */
#define CMD_READ_SINGLE 0x2C06

/*TWI instance*/
static const nrf_drv_twi_t twi_sht3x = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/*Buffer for samples read from temperature sensor SHT3x*/
static uint8_t rx_buf[6];

/* CRC-8 calculation*/
static const uint8_t CRC8_POLY = 0x31;

uint8_t crc8_compute(uint8_t *message, uint8_t length) 
{
  uint8_t bit, byte;
  uint8_t crc = 0xFF;

  for (byte = 0; byte < length; ++byte) {
    crc ^= message[byte];
    for (bit = 8; bit > 0; --bit) {
      if (crc & (1 << 7))
        crc = (crc << 1) ^ CRC8_POLY;
      else
        crc = crc << 1;
    }
    return crc;
  }
}

/*Compare the computed checksum and received checksum*/ 
__INLINE bool checksum_validate(uint8_t * message, uint8_t length, uint8_t crc_received) 
{
  uint8_t crc = crc8_compute(message, length);
  return (crc == crc_received);
}

/*TWI initialization.*/
bool SHT3x_twi_init(void) 
{
    uint32_t err_code;
    const nrf_drv_twi_config_t twi_sht3x_config = 
	{
        .scl			= ARDUINO_SCL_PIN,  //29
        .sda		        = ARDUINO_SDA_PIN,  //28
        .frequency              = NRF_DRV_TWI_FREQ_400K,
        .interrupt_priority	= APP_IRQ_PRIORITY_HIGH,
        .clear_bus_init		 = false
	};
    err_code = nrf_drv_twi_init(&twi_sht3x, &twi_sht3x_config, NULL, NULL);
      if (err_code != NRF_SUCCESS) return err_code;

    nrf_drv_twi_enable(&twi_sht3x);

    return  NRF_SUCCESS;
}

void SHT3x_read_temperature(temp_values_t *temperature)
{     
        uint8_t cmd[2];

        cmd[0] = (uint8_t)(CMD_READ_SINGLE >> 8);
        cmd[1] = (uint8_t)(CMD_READ_SINGLE);
	
      /* Send command one byte at a time to set single shot reading */
      nrf_drv_twi_tx(&twi_sht3x, SHT3x_ADDR, cmd, sizeof(cmd), false);
	
      /* Read received sensor data into buffer */
      nrf_drv_twi_rx(&twi_sht3x, SHT3x_ADDR, rx_buf, sizeof(rx_buf)) ;

      /* Compute checksum for temperature sensor value */
      checksum_validate(&rx_buf[0], 2, rx_buf[2])  ;
       uint16_t temp_int = rx_buf[0] << 8 | rx_buf[1];
      *temperature = -45.0 + ((float)(175 * temp_int)) / (65535);
}

void SHT3x_read_humidity(humi_values_t *humidity)
{     
        uint8_t cmd[2];

        cmd[0] = (uint8_t)(CMD_READ_SINGLE >> 8);
        cmd[1] = (uint8_t)(CMD_READ_SINGLE);
	
      /* Send command one byte at a time to set single shot reading */
      nrf_drv_twi_tx(&twi_sht3x, SHT3x_ADDR, cmd, sizeof(cmd), false);
	
      /* Read received sensor data into buffer */
      nrf_drv_twi_rx(&twi_sht3x, SHT3x_ADDR, rx_buf, sizeof(rx_buf)) ;

      /* Compute checksum for temperature sensor value */
      checksum_validate(&rx_buf[0], 2, rx_buf[2])  ;
       uint16_t humi_int = rx_buf[0] << 8 | rx_buf[1];
      *humidity = (float)(100 * humi_int)/(65535);
}
/*lint --flb "Leave library region" */
