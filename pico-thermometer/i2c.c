#include <stdbool.h>

#include <libopencm3/stm32/i2c.h>

#include "i2c.h"

#define I2C_TIMEOUT_ADDRESS_US 1000
#define I2C_TIMEOUT_DATA_US 1000

int i2c_transfer(uint32_t i2c, uint8_t address, uint8_t *tx, uint8_t txlen, uint8_t *rx, uint8_t rxlen) {
	if (txlen) {
		uint32_t sr_flags;

		i2c_set_7bit_address(i2c, address);
		i2c_set_write_transfer_dir(i2c);
		i2c_set_bytes_to_transfer(i2c, txlen);
		if (rxlen) {
			i2c_disable_autoend(i2c);
		} else {
			i2c_enable_autoend(i2c);
		}
		I2C_ICR(i2c) |= I2C_ICR_NACKCF;
		i2c_send_start(i2c);
		while (txlen--) {
			sr_flags = os_wait_flag_timeout(&I2C_ISR(i2c), I2C_ISR_TXIS | I2C_ISR_NACKF, I2C_TIMEOUT_DATA_US);
			if (!sr_flags || (sr_flags & I2C_ICR_NACKCF)) {
				return -1;
			}
			i2c_send_data(i2c, *tx++);
		}
	}
	if (rxlen) {
		unsigned len = 0;
		uint32_t sr_flags;

		i2c_set_7bit_address(i2c, address);
		i2c_set_read_transfer_dir(i2c);
		i2c_set_bytes_to_transfer(i2c, rxlen);
		i2c_send_start(i2c);
		i2c_enable_autoend(i2c);
		I2C_ICR(i2c) |= I2C_ICR_NACKCF;
		while (rxlen--) {
			sr_flags = os_wait_flag_timeout(&I2C_ISR(i2c), I2C_ISR_RXNE | I2C_ISR_NACKF, I2C_TIMEOUT_DATA_US);
			if (!sr_flags) {
				return -1;
			}
			if (sr_flags & I2C_ICR_NACKCF) {
				return len;	
			}
			*rx++ = i2c_get_data(i2c);
			len++;
		}
	}

	return 0;
}
