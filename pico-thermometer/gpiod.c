#include "gpiod.h"

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

#include "util.h"

#define GPIO_FLAG_INVERTED	BIT(0)
#define GPIO_FLAG_ON		BIT(1)
#define GPIO_FLAG_FORCE_OUT_OPT	BIT(2)

#define GPIO_AF_GPIO 0xff

static gpio_t gpios_g[] = {
	{ GPIOA, GPIO0,  GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, 0, GPIO_FLAG_INVERTED },  // NRF power
	{ GPIOA, GPIO1,  GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, 0, 0 },                   // SHT power
	{ GPIOA, GPIO2,  GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, 0, 0 },                   // LED BLE
	{ GPIOA, GPIO3,  GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, 0, 0 },                   // LED SHT
	{ GPIOA, GPIO4,  GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, 0, 0, 0, GPIO_FLAG_INVERTED },  // SPI1 NSS
	{ GPIOA, GPIO5,  GPIO_MODE_AF,     GPIO_PUPD_NONE, 0, 0, GPIO_AF0, 0 },            // SPI1 SCK
	{ GPIOA, GPIO6,  GPIO_MODE_AF,     GPIO_PUPD_NONE, 0, 0, GPIO_AF0, 0 },            // SPI1 MISO
	{ GPIOA, GPIO7,  GPIO_MODE_AF,     GPIO_PUPD_NONE, 0, 0, GPIO_AF0, 0 },            // SPI1 MOSI
	{ GPIOA, GPIO9,  GPIO_MODE_AF,     GPIO_PUPD_NONE, GPIO_OTYPE_OD, 0, GPIO_AF1, GPIO_FLAG_FORCE_OUT_OPT },            // I2C1 SCL
	{ GPIOA, GPIO10, GPIO_MODE_AF,     GPIO_PUPD_NONE, GPIO_OTYPE_OD, 0, GPIO_AF1, GPIO_FLAG_FORCE_OUT_OPT },            // I2C1 SDA
	{ GPIOB, GPIO1,  GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, 0, 0 },                   // NRF CE
};

void gpiod_init() {
	uint8_t i;

	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);

	for (i = 0; i < ARRAY_SIZE(gpios_g); i++) {
		const gpio_t *gpio = &gpios_g[i];

		gpio_mode_setup(gpio->port, gpio->mode, gpio->pullcfg, gpio->gpio);
		if (gpio->mode == GPIO_MODE_OUTPUT) {
			gpiod_set(i, 0);
		}
		if (gpio->mode == GPIO_MODE_OUTPUT || (gpio->flags & GPIO_FLAG_FORCE_OUT_OPT)) {
			gpio_set_output_options(gpio->port, gpio->otype, gpio->ospeed, gpio->gpio);
		}
		if (gpio->mode == GPIO_MODE_AF) {
			gpio_set_af(gpio->port, gpio->af, gpio->gpio);
		}
	}
}

void gpiod_set(uint8_t gpionum, uint8_t value) {
	gpio_t *gpio = &gpios_g[gpionum];

	if (!value == !(gpio->flags & GPIO_FLAG_INVERTED)) {
		if (gpio->flags & GPIO_FLAG_ON) {
			gpio_clear(gpio->port, gpio->gpio);
			gpio->flags &= ~GPIO_FLAG_ON;
		}
	} else {
		if (!(gpio->flags & GPIO_FLAG_ON)) {
			gpio_set(gpio->port, gpio->gpio);
			gpio->flags |= GPIO_FLAG_ON;
		}
	}
}

uint16_t gpiod_get(uint8_t gpionum) {
	gpio_t *gpio = &gpios_g[gpionum];

	if (gpio->flags & GPIO_FLAG_INVERTED) {
		return (~gpio_get(gpio->port, gpio->gpio)) & gpio->gpio;
	}
	return gpio_get(gpio->port, gpio->gpio);
}

void gpiod_toggle(uint8_t gpionum) {
	gpio_t *gpio = &gpios_g[gpionum];

	gpiod_set(gpionum, !(gpio->flags & GPIO_FLAG_ON));
}

uint32_t gpiod_get_port(uint8_t gpionum) {
	return gpios_g[gpionum].port;
}

uint16_t gpiod_get_gpio(uint8_t gpionum) {
	return gpios_g[gpionum].gpio;
}
