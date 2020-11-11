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
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>

#include "ble.h"
#include "gpiod.h"
#include "os.h"
#include "os_time.h"
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
#define BEACON_INTERVAL_MS 1000

static const struct rcc_clock_scale clock_32MHz = {
	.pll_source = RCC_CFGR_PLLSRC_HSI16_CLK,
	.pll_mul = RCC_CFGR_PLLMUL_MUL4,
	.pll_div = RCC_CFGR_PLLDIV_DIV2,
	.flash_waitstates = FLASH_ACR_LATENCY_1WS,
	.voltage_scale = PWR_SCALE1,
	.hpre = RCC_CFGR_HPRE_NODIV,
	.ppre1 = RCC_CFGR_PPRE1_NODIV,
	.ppre2 = RCC_CFGR_PPRE2_NODIV,
	.ahb_frequency = MHZ(32),
	.apb1_frequency = MHZ(32),
	.apb2_frequency = MHZ(32),
	.msi_range = 0b101
};

static void clock_init(void) {
	rcc_clock_setup_pll(&clock_32MHz);
	RCC_CFGR |= RCC_CFGR_STOPWUCK_HSI16;
}

static void spi_init(void) {
	rcc_periph_clock_enable(RCC_SPI1);
	spi_reset(SPI1);
	spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_16,
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
	nvic_set_priority(NVIC_I2C1_IRQ, 0);
	nvic_enable_irq(NVIC_I2C1_IRQ);

	rcc_periph_clock_enable(RCC_I2C1);

	i2c_reset(I2C1);
	i2c_peripheral_disable(I2C1);
	i2c_set_speed(I2C1, i2c_speed_sm_100k, rcc_apb1_frequency / MHZ(1));
//	i2c_enable_interrupt(I2C1, I2C_CR1_STOPIE | I2C_CR1_ADDRIE);
	i2c_set_digital_filter(I2C1, 3);
	i2c_enable_analog_filter(I2C1);
	i2c_peripheral_enable(I2C1);
}

static void sht_cmd(uint8_t cmd) {
	i2c_transfer7(I2C1, SHT_ADDRESS, &cmd, 1, NULL, 0);	
}

static int32_t sht_read_temperature_mdeg(void) {
	uint16_t res;
	int32_t mdeg;

	sht_cmd(SHT_CMD_MEASURE_TEMPERATURE);
	os_delay(MS_TO_US(100));
	i2c_transfer7(I2C1, SHT_ADDRESS, NULL, 0, &res, 2);
	res = ((res & 0xff) << 8) | ((res & 0xff00) >> 8);
	mdeg = (175720LL * (int64_t)res) / 65536LL - 46850LL;
	return mdeg;
}

static uint32_t sht_read_humidity_m_perc(void) {
	uint16_t res;
	uint32_t m_perc_rh;

	sht_cmd(SHT_CMD_MEASURE_HUMIDITY);
	os_delay(MS_TO_US(100));
	i2c_transfer7(I2C1, SHT_ADDRESS, NULL, 0, &res, 2);
	res = ((res & 0xff) << 8) | ((res & 0xff00) >> 8);
	m_perc_rh = (125000ULL * (uint64_t)res) / 65536ULL - 6000ULL;
	return m_perc_rh;
}

static void nrf_on(void) {
	gpiod_set(GPIO_NRF_POWER, 1);
}

static void nrf_off(void) {
	gpiod_set(GPIO_NRF_POWER, 0);
}

static void ce_lo(void) {
	gpiod_set(GPIO_NRF_CE, 0);
}

static void ce_hi(void) {
	gpiod_set(GPIO_NRF_CE, 1);
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
	nrf_register_write(NRF_REG_CONFIG, 0x00); // crc disabled, powered down, ptx
	nrf_register_write(NRF_REG_EN_AA, 0x00); // Disable auto ack
	nrf_register_write(NRF_REG_RX_PW_P0, 0x20); // Enable RX pipe 0
	nrf_register_write(NRF_REG_EN_RXADDR, 0x01); // Enable RX 0
	nrf_register_write(NRF_REG_SETUP_AW, 0x02); // 4 byte addresses
	nrf_register_write(NRF_REG_SETUP_RETR, 0x00); // No retransmissions
	nrf_register_write(NRF_REG_RF_SETUP, 0x06); // 1Mbit/s, 0dBm tx power
	nrf_register_write(NRF_REG_FEATURE, 0x00); // Disable ShockBurst features
	nrf_register_write(NRF_REG_DYNPD, 0x00); // No dynamic payloads
	nrf_cmd_multibyte(NRF_CMD_WRITE_REGISTER | NRF_REG_RX_ADDR_P0, *ble_get_access_address(), sizeof(ble_access_address_t)); // Set RX address
	nrf_cmd_multibyte(NRF_CMD_WRITE_REGISTER | NRF_REG_TX_ADDR, *ble_get_access_address(), sizeof(ble_access_address_t)); // Set TX address
}

/*
static uint8_t msg[] = {
	0x07,
	0x09,
	'N',
	'R',
	'F',
	'2',
	'4',
	'L',
};
*/
static uint8_t hdr = BLE_HEADER(BLE_PDU_TYPE_ADV_NONCONN_IND, 0, 1, 0);

static ble_mac_address_t mac_address = { 0x37, 0x13, 0xb7, 0x41, 0x80, 0x00 };

int32_t mdeg_c;
uint32_t m_perc_rh;

os_task_t measure_task;
os_task_t tx_task;

static bool data_updated = false;

static void measure(void *ctx);
static void measure(void *ctx) {
	os_schedule_task_relative(&measure_task, measure, MS_TO_US(MEASURE_INTERVAL_MS), NULL);

	sht_on();
	os_delay(MS_TO_US(100));

	gpiod_set(GPIO_LED_SHT, 1);
	mdeg_c = sht_read_temperature_mdeg();
	m_perc_rh = sht_read_humidity_m_perc();
	gpiod_set(GPIO_LED_SHT, 0);

	sht_off();

	data_updated = true;
}

static unsigned milli_to_decimal_str(char *str, unsigned max_len, long val) {
	unsigned i, len = 0;

	i = long_to_str(str, max_len, val / 1000, 2);
	str += i;
	len += i;
	max_len -= i;
	*str++ = '.';
	len++;
	max_len--;
	len += long_to_str(str, max_len, (val < 0 ? (1000 - val % 1000) : val % 1000) / 10, 2);

	return len;
}

static unsigned build_complete_local_name(char *str, unsigned len) {
	char *str_start = str;
	int i;

	i = milli_to_decimal_str(str, len, mdeg_c);
	len -= i;
	str += i;
	strncpy(str, "°C ", len);
	len -= 3;
	str += 3;
	i = milli_to_decimal_str(str, len, m_perc_rh);
	len -= i;
	str += i;
	strncpy(str, "%RH", len);
	len -= 3;
	str += 3;

	return str - str_start;
}

static void ble_tx(void *ctx);
static void ble_tx(void *ctx) {
	uint8_t frame[32] = { 0 };
	char msg[32];
	ble_channel_t* channel;
	uint16_t len;

	os_schedule_task_relative(&tx_task, ble_tx, MS_TO_US(BEACON_INTERVAL_MS), NULL);

	nrf_register_write(NRF_REG_CONFIG, 0x02); // crc disabled, powered up, ptx
	os_delay(MS_TO_US(10));

	if (data_updated) {
		data_updated = false;
	//	msg[0] = snprintf(msg + 2, sizeof(msg) - 2, "%ld.%02ld°C %02lu.%02lu", mdeg_c / 1000, mdeg_c < 0 ? 1000 - mdeg_c % 1000 : mdeg_c % 1000, m_perc_rh / 1000, m_perc_rh % 1000) + 1;
	//	msg[0] = snprintf(msg + 2, sizeof(msg) - 2, "miau :3") + 1;
	//	msg[0] = snprintf(msg + 2, sizeof(msg) - 2, "%ld.%02ld°C %02u.%02u%%RH", mdeg_c / 1000, mdeg_c < 0 ? (1000 - mdeg_c % 1000) / 10 : (mdeg_c % 1000) / 10, m_perc_rh / 1000, (m_perc_rh % 1000) / 10) + 1;
		msg[0] = build_complete_local_name(msg + 2, sizeof(msg) - 2) + 1;
		msg[1] = 0x09;

		channel = ble_get_advertisement_channel();

		nrf_register_write(NRF_REG_RF_CH, channel->frequency_mhz - 2400);
		len = ble_frame(frame, sizeof(frame), hdr, mac_address, msg, msg[0] + 2, channel);

		nrf_register_write(NRF_REG_STATUS, 0x70); // Clear flags
		nrf_cmd_multibyte(NRF_CMD_WRITE_TX_PAYLOAD, frame, len);
	} else {
		nrf_cmd(NRF_CMD_REUSE_TX_PAYLOAD); // Reuse last tx payload
	}

	ce_hi();
	gpiod_set(GPIO_LED_BLE, 1);
	os_delay(MS_TO_US(1));
	gpiod_set(GPIO_LED_BLE, 0);
	ce_lo();

	while (!nrf_get_status() & 0x20) {
		os_delay(MS_TO_US(10));
	}
	nrf_register_write(NRF_REG_CONFIG, 0x00); // crc disabled, powered down, ptx
}

int main(void) {
	clock_init();
	gpiod_init();
	os_init();
	spi_init();
	ce_lo();
	nrf_off();
	sht_off();
	os_delay(MS_TO_US(1000));
	nrf_on();
	nrf_ble_setup();
	os_delay(MS_TO_US(100));
	i2c_init();

	os_delay(MS_TO_US(1000));

	measure(NULL);
	ble_tx(NULL);

	while(1) {
		os_run();
	}

	return 0;
}

