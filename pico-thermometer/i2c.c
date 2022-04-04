#include <stdbool.h>

#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>

#include "i2c.h"
#include "os.h"

#define I2C_TIMEOUT_ADDRESS_US 1000
#define I2C_TIMEOUT_DATA_US 1000

static void i2c_sleep_inihibit(uint32_t i2c, bool inhibit) {
	switch (i2c) {
	case I2C1:
		if (inhibit) {
			RCC_APB1SMENR |= RCC_APB1SMENR_I2C1SMEN;
		} else {
			RCC_APB1SMENR &= ~RCC_APB1SMENR_I2C1SMEN;
		}
		os_inhibit_deep_sleep(inhibit);
		break;
	case I2C2:
		if (inhibit) {
			RCC_APB1SMENR |= RCC_APB1SMENR_I2C2SMEN;
		} else {
			RCC_APB1SMENR &= ~RCC_APB1SMENR_I2C2SMEN;
		}
		os_inhibit_deep_sleep(inhibit);
		break;
	case I2C3:
		if (inhibit) {
			RCC_APB1SMENR |= RCC_APB1SMENR_I2C3SMEN;
		} else {
			RCC_APB1SMENR &= ~RCC_APB1SMENR_I2C3SMEN;
		}
		os_inhibit_deep_sleep(inhibit);
		break;
	default:
		os_inhibit_sleep(inhibit);
	}
}

int i2c_transfer(uint32_t i2c, uint8_t address, uint8_t *tx, uint8_t txlen, uint8_t *rx, uint8_t rxlen) {
	i2c_sleep_inihibit(i2c, true);
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
				i2c_sleep_inihibit(i2c, false);
				return -1;
			}
			i2c_send_data(i2c, *tx++);
		}
		os_wait_flag_timeout(&I2C_ISR(i2c), I2C_ISR_TXIS | I2C_ISR_NACKF, I2C_TIMEOUT_DATA_US);
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
				i2c_sleep_inihibit(i2c, false);
				return len;	
			}
			*rx++ = i2c_get_data(i2c);
			len++;
		}
	}

	i2c_sleep_inihibit(i2c, false);
	return 0;
}
