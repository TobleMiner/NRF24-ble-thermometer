#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencmsis/core_cm3.h>

#include "gpiod.h"
#include "sleep.h"
#include "util.h"

#define WFE() __asm__ volatile("wfe")
#define WFI() __asm__ volatile("wfi")

#define DBG_BASE	0x40015800
#define DBG_CR		*((uint32_t*)(DBG_BASE + 0x04))
#define DBG_SLEEP	BIT(0)
#define DBG_STOP	BIT(1)
#define DBG_STANDBY	BIT(2)

void sleep_init() {
	rcc_periph_reset_pulse(RST_PWR);
	rcc_periph_clock_enable(RCC_PWR);

	RCC_AHBSMENR = 0x00;
	RCC_APB2SMENR = 0x00;
	RCC_APB1SMENR = RCC_APB1SMENR_LPTIM1SMEN;
}

void sleep_enter(void) {
	SCB_SCR &= ~SCB_SCR_SLEEPDEEP;
	WFI();
}

void sleep_enter_event(void) {
	SCB_SCR &= ~SCB_SCR_SLEEPDEEP;
	WFE();
}

void sleep_enter_low_power(void) {
	sleep_enter();
}

void sleep_enter_stop(void) {
	rcc_periph_clock_enable(RCC_PWR);
	rcc_periph_reset_pulse(RST_PWR);
	SCB_SCR |= SCB_SCR_SLEEPDEEP;
	PWR_CR &= ~PWR_CR_PDDS;
	PWR_CR |= PWR_CR_ULP | PWR_CR_LPDS | PWR_CR_FWU;
	WFI();
}

void sleep_enter_standby(void) {

}
