#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencmsis/core_cm3.h>

#define WFE() __asm__ volatile("wfe")
#define WFI() __asm__ volatile("wfi")

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
	SCB_SCR |= SCB_SCR_SLEEPDEEP;
	PWR_CR &= ~PWR_CR_PDDS;
	PWR_CR |= PWR_CR_ULP | PWR_CR_LPDS | PWR_CR_FWU;
	PWR_CSR &= ~PWR_CSR_WUF;
	WFI();
}

void sleep_enter_standby(void) {

}
