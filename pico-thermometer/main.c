#include <stddef.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>

#include "ble.h"
#include "gpiod.h"
#include "i2c.h"
#include "os.h"
#include "os_time.h"
#include "sleep.h"
#include "strutil.h"
#include "util.h"

#define NRF_CMD_READ_REGISTER    0x00
#define NRF_CMD_WRITE_REGISTER   0x20
#define NRF_CMD_WRITE_TX_PAYLOAD 0xa0
#define NRF_CMD_REUSE_TX_PAYLOAD 0xe3
#define NRF_CMD_NOP              0xff

#define NRF_REG_CONFIG           0x00
#define NRF_REG_EN_AA            0x01
#define NRF_REG_EN_RXADDR        0x02
#define NRF_REG_SETUP_AW         0x03
#define NRF_REG_SETUP_RETR       0x04
#define NRF_REG_RF_CH            0x05
#define NRF_REG_RF_SETUP         0x06
#define NRF_REG_STATUS           0x07
#define NRF_REG_OBSERVE_TX       0x08
#define NRF_REG_RX_ADDR_P0       0x0A
#define NRF_REG_TX_ADDR          0x10
#define NRF_REG_RX_PW_P0         0x11
#define NRF_REG_FIFO_STATUS      0x17
#define NRF_REG_DYNPD            0x1C
#define NRF_REG_FEATURE          0x1D

#define NRF_STATUS_TX_DS         0x20
#define NRF_STATUS_TX_FULL       0x01

#define SHT_ADDRESS 0x40

#define SHT_CMD_MEASURE_TEMPERATURE 0xf3
#define SHT_CMD_MEASURE_HUMIDITY    0xf5

#define MEASURE_INTERVAL_MS 30000
#define BEACON_INTERVAL_MS 500

#define WATCHDOG_INTERVAL_MS 10000

const char *device_name = "";

static void clock_init(void) {
	rcc_osc_on(RCC_HSI16);
	rcc_wait_for_osc_ready(RCC_HSI16);
	rcc_set_sysclk_source(RCC_HSI16);
	RCC_CFGR |= RCC_CFGR_STOPWUCK_HSI16;
}

static void watchdog_init(void) {
	iwdg_set_period_ms(WATCHDOG_INTERVAL_MS);
	iwdg_start();
}

static void spi_init(void) {
	rcc_periph_clock_enable(RCC_SPI1);
	spi_reset(SPI1);
	spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_4,
	                SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
	                SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT,
	                SPI_CR1_MSBFIRST);
	spi_set_dff_8bit(SPI1);
//	spi_enable_rx_dma(SPI1);
//	spi_enable_tx_dma(SPI1);
	spi_enable(SPI1);
}

static void sht_on(void) {
	gpiod_set(GPIO_SHT_POWER, 1);
}

static void sht_off(void) {
	gpiod_set(GPIO_SHT_POWER, 0);
}

static void i2c_init(void) {
	rcc_periph_clock_enable(RCC_I2C1);

	i2c_reset(I2C1);
	i2c_peripheral_disable(I2C1);
	i2c_set_speed(I2C1, i2c_speed_sm_100k, rcc_apb1_frequency / MHZ(1));
	i2c_enable_interrupt(I2C1, I2C_CR1_ADDRIE | I2C_CR1_NACKIE | I2C_CR1_RXIE | I2C_CR1_TXIE);
	i2c_set_digital_filter(I2C1, 3);
	i2c_enable_analog_filter(I2C1);
	i2c_peripheral_enable(I2C1);
}

static int sht_cmd(uint8_t cmd) {
	return i2c_transfer(I2C1, SHT_ADDRESS, &cmd, 1, NULL, 0);
}

static int sht_read_temperature_mdeg(int32_t *ret) {
	uint16_t res;
	int err;

	sht_cmd(SHT_CMD_MEASURE_TEMPERATURE);
	os_delay(MS_TO_US(85));
	err = i2c_transfer(I2C1, SHT_ADDRESS, NULL, 0, (uint8_t*)&res, 2);
	if (err < 0) {
		return err;
	}
	res = ((res & 0xff) << 8) | ((res & 0xff00) >> 8);
	*ret = (175720LL * (int64_t)res) / 65536LL - 46850LL;
	return 0;
}

static int sht_read_humidity_m_perc(uint32_t *ret) {
	uint16_t res;
	int err;

	sht_cmd(SHT_CMD_MEASURE_HUMIDITY);
	os_delay(MS_TO_US(30));
	err = i2c_transfer(I2C1, SHT_ADDRESS, NULL, 0, (uint8_t*)&res, 2);
	if (err < 0) {
		return err;
	}
	res = ((res & 0xff) << 8) | ((res & 0xff00) >> 8);
	*ret = (125000ULL * (uint64_t)res) / 65536ULL - 6000ULL;
	return 0;
}

static void ce_lo(void) {
	gpiod_set(GPIO_NRF_CE, 0);
}

static void ce_hi(void) {
	gpiod_set(GPIO_NRF_CE, 1);
}

static void nrf_on(void) {
	gpiod_set(GPIO_SPI_NSS, 0);
	gpiod_set(GPIO_NRF_POWER, 1);
}

static void nrf_off(void) {
	ce_lo();
	gpiod_set(GPIO_NRF_POWER, 0);
	gpiod_set(GPIO_SPI_NSS, 1);
}

static void nrf_register_write(uint8_t reg, uint8_t val) {
	gpiod_set(GPIO_SPI_NSS, 1);
	spi_xfer(SPI1, NRF_CMD_WRITE_REGISTER | reg);
	spi_xfer(SPI1, val);
	gpiod_set(GPIO_SPI_NSS, 0);
}

static uint8_t nrf_register_read(uint8_t reg) {
	uint8_t val;

	gpiod_set(GPIO_SPI_NSS, 1);
	spi_xfer(SPI1, NRF_CMD_READ_REGISTER | reg);
	val = spi_xfer(SPI1, 0xff);
	gpiod_set(GPIO_SPI_NSS, 0);
	return val;
}

static void nrf_cmd_multibyte(uint8_t cmd, uint8_t *data, uint8_t len) {
	gpiod_set(GPIO_SPI_NSS, 1);
	spi_xfer(SPI1, cmd);
	while (len--) {
		spi_xfer(SPI1, *data++);
	}
	gpiod_set(GPIO_SPI_NSS, 0);
}

static void nrf_cmd(uint8_t cmd) {
	nrf_cmd_multibyte(cmd, NULL, 0);
}

static uint8_t nrf_get_status(void) {
	uint8_t status;

	gpiod_set(GPIO_SPI_NSS, 1);
	status = spi_xfer(SPI1, NRF_CMD_NOP);
	gpiod_set(GPIO_SPI_NSS, 0);
	
	return status;
}

static void nrf_ble_setup(void) {
	nrf_register_write(NRF_REG_EN_AA, 0x00); // Disable auto ack
	nrf_register_write(NRF_REG_RX_PW_P0, 0x20); // Enable RX pipe 0
	nrf_register_write(NRF_REG_EN_RXADDR, 0x01); // Enable RX 0
	nrf_register_write(NRF_REG_SETUP_AW, 0x02); // 4 byte addresses
	nrf_register_write(NRF_REG_SETUP_RETR, 0x00); // No retransmissions
	nrf_register_write(NRF_REG_RF_SETUP, 0x06); // 1Mbit/s, 0dBm tx power
	nrf_cmd_multibyte(NRF_CMD_WRITE_REGISTER | NRF_REG_RX_ADDR_P0, *ble_get_access_address(), sizeof(ble_access_address_t)); // Set RX address
	nrf_cmd_multibyte(NRF_CMD_WRITE_REGISTER | NRF_REG_TX_ADDR, *ble_get_access_address(), sizeof(ble_access_address_t)); // Set TX address
}

static uint8_t hdr = BLE_HEADER(BLE_PDU_TYPE_ADV_NONCONN_IND, 0, 1, 0);

static ble_mac_address_t mac_address = { 0x00, 0x00, 0x00, 0x41, 0x80, 0x00 };

int32_t mdeg_c = 0;
uint32_t m_perc_rh = 0;

os_task_t measure_task;
os_task_t tx_task;

static bool data_updated = false;

static void measure(void *ctx);
static void measure(void *ctx) {
	(void)ctx;

	os_schedule_task_relative(&measure_task, measure, MS_TO_US(MEASURE_INTERVAL_MS), NULL);

	sht_on();
	gpiod_set(GPIO_LED_SHT, 1);
	os_delay(MS_TO_US(1));
	gpiod_set(GPIO_LED_SHT, 0);
	os_delay(MS_TO_US(20));

	if (sht_read_temperature_mdeg(&mdeg_c) < 0) {
		goto out;
	}
	if (sht_read_humidity_m_perc(&m_perc_rh) < 0) {
		goto out;
	}

	data_updated = true;

out:
	sht_off();
}

static bool str_write(char **str, unsigned *len, const void *data, unsigned data_len) {
	if (*len < data_len) {
		return false;
	}

	memcpy(*str, data, data_len);
	*str += data_len;
	*len -= data_len;

	return true;
}

static bool str_write_str(char **str, unsigned *len, const char *data) {
	return str_write(str, len, data, strlen(data));
}

static int milli_to_decimal_str(char **str, unsigned *len, long val) {
	unsigned i;
	char *str_start = *str;

	i = long_to_str(*str, *len, val / 1000, 0);
	if (i > *len) {
		return -1;
	}
	*str += i;
	*len -= i;
	if (!*len) {
		return -1;
	}
	**str = '.';
	(*str)++;
	(*len)--;
	i = long_to_str(*str, *len, (val < 0 ? (1000 - val % 1000) : val % 1000) / 100, 2);
	if (i > *len) {
		return -1;
	}
	*str += i;
	*len -= i;

	return *str - str_start;
}

static int build_complete_local_name(char *str, unsigned len) {
	char *str_start = str;
	int i;

	if (!str_write_str(&str, &len, device_name)) {
		return -1;
	}
	if(milli_to_decimal_str(&str, &len, mdeg_c) < 0) {
		return -1;
	}
	if (!str_write_str(&str, &len, "°C ")) {
		return -1;
	}
	if(milli_to_decimal_str(&str, &len, m_perc_rh) < 0) {
		return -1;
	}
	if (!str_write_str(&str, &len, "%RH")) {
		return -1;
	}

	return str - str_start;
}

#define MAX_STATUS_POLLS 10

static uint8_t ble_data[32] = "  unitialized";
static unsigned int ble_data_len = 13;

static void ble_tx(void *ctx);
static void ble_tx(void *ctx) {
	uint8_t ble_frame[32];
	uint16_t ble_frame_len;

	ble_channel_t* channel;
	unsigned status_polls = 0;
	uint32_t i;

	(void)ctx;

	os_schedule_task_relative(&tx_task, ble_tx, MS_TO_US(BEACON_INTERVAL_MS), NULL);

	channel = ble_get_advertisement_channel();

	if (data_updated) {
		data_updated = false;
		ble_data[0] = build_complete_local_name((char*)ble_data + 2, sizeof(ble_data) - 2) + 1;
		ble_data[1] = 0x09;

	}

	ble_frame_len = ble_make_frame(ble_frame, sizeof(ble_frame), hdr, mac_address, ble_data, ble_data[0] + 2, channel);

	nrf_on();
	os_delay(MS_TO_US(100));

	nrf_ble_setup();
	nrf_register_write(NRF_REG_RF_CH, channel->frequency_mhz - 2400);
	nrf_cmd_multibyte(NRF_CMD_WRITE_TX_PAYLOAD, ble_frame, ble_frame_len);

	nrf_register_write(NRF_REG_CONFIG, 0x02); // crc disabled, powered up, ptx
	os_delay(1500);

	nrf_register_write(NRF_REG_STATUS, 0x70); // Clear flags

	ce_hi();
	gpiod_set(GPIO_LED_BLE, 1);
	for (i = 0; i < 100; i++) {
		__asm__ volatile("" : "+g" (i) : :);
	}
//	os_delay(MS_TO_US(1));
	gpiod_set(GPIO_LED_BLE, 0);
	ce_lo();

	while (!(nrf_get_status() & 0x20)) {
		status_polls++;
		if (status_polls >= MAX_STATUS_POLLS) {
			break;
		}
		os_delay(MS_TO_US(1));
	}
	nrf_off();
}

#define U_ID_BASE 0x1FF80050

#define U_ID1 (*(uint32_t*)(U_ID_BASE + 0x00))
#define U_ID2 (*(uint32_t*)(U_ID_BASE + 0x04))
#define U_ID3 (*(uint32_t*)(U_ID_BASE + 0x14))

static void mac_address_init(void) {
	uint32_t uid = U_ID3;

	memcpy(mac_address, &uid, 3);
}

int main(void) {
	watchdog_init();
	clock_init();
	gpiod_init();
	mac_address_init();
	/* Run sleep init before os_init to enable PWR clock domain */
	sleep_init();
	os_init();
	spi_init();
	ce_lo();
	sht_off();
	i2c_init();
	nrf_off();

	measure(NULL);
	ble_tx(NULL);

	while(1) {
		os_run();
		iwdg_reset();
	}

	return 0;
}
