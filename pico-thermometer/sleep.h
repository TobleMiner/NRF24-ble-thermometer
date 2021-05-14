#pragma once

static inline void __SEV(void) {
	__asm__ volatile("sev");
}

void sleep_init(void);
void sleep_enter(void);
void sleep_enter_event(void);
void sleep_enter_low_power(void);
void sleep_enter_stop(void);
void sleep_enter_standby(void);
