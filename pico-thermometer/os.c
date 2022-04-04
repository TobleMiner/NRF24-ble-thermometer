#include <stddef.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/lptimer.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencmsis/core_cm3.h>

#include "os.h"
#include "sleep.h"

#define OS_TIMER LPTIM1
#define OS_TIMER_RCC RCC_LPTIM1
#define OS_TIMER_RST RST_LPTIM1
#define OS_TIMER_TOP 0xFFFF
#define OS_TIMER_IRQ NVIC_LPTIM1_IRQ
#define OS_TIMER_IRQ_HANDLER lptim1_isr
#define OS_TIMER_TOP 0xFFFF

#define OS_SLEEP_THRESHOLD_LOW_POWER_US 1000
#define OS_SLEEP_THRESHOLD_STOP_US     10000

static uint32_t last_timer_counter_sync = 0;
static os_time_t last_os_time = { 0, 0 };
static uint32_t next_task_timer_counter = 0;
static uint32_t last_run_timer_counter = 0;

static os_task_t *os_tasks;
static os_task_t *next_task;

static bool inhibit_deep_sleep = false;
static bool inhibit_sleep = false;

void os_inhibit_deep_sleep(bool inhibit) {
	inhibit_deep_sleep = inhibit;
}

void os_inhibit_sleep(bool inhibit) {
	inhibit_sleep = inhibit;
}

static void do_sleep(uint32_t us) {
	if (inhibit_sleep) {
		return;
	}
	if (us >= OS_SLEEP_THRESHOLD_STOP_US && !inhibit_deep_sleep) {
		sleep_enter_stop();
/*
	} else if (us >= OS_SLEEP_THRESHOLD_LOW_POWER_US) {
		sleep_enter_low_power();
*/
	} else {
		sleep_enter();
	}
}

void os_init() {
	// 0.977ms resolution RTOS timer
	rcc_periph_clock_enable(OS_TIMER_RCC);
	rcc_periph_reset_pulse(OS_TIMER_RST);
	pwr_disable_backup_domain_write_protect();
	RCC_CSR |= RCC_CSR_LSEDRV_HIGHEST << RCC_CSR_LSEDRV_SHIFT;
	rcc_osc_on(RCC_LSE);
	pwr_enable_backup_domain_write_protect();
	rcc_wait_for_osc_ready(RCC_LSE);
	rcc_set_peripheral_clk_sel(LPTIM1, RCC_CCIPR_LPTIM1SEL_LSE);
	// 32.768kHz LSE / 32 = 1.024kHz
	lptimer_set_internal_clock_source(OS_TIMER);
	lptimer_set_prescaler(OS_TIMER, LPTIM_CFGR_PRESC_32);
	lptimer_enable(OS_TIMER);
	lptimer_set_period(OS_TIMER, OS_TIMER_TOP);
	lptimer_enable_irq(OS_TIMER, LPTIM_IER_ARRMIE | LPTIM_IER_CMPMIE);
	nvic_enable_irq(NVIC_LPTIM1_IRQ);
	lptimer_start_counter(OS_TIMER, LPTIM_CR_CNTSTRT);
}

static volatile bool timer_elapsed = false;

void OS_TIMER_IRQ_HANDLER(void) {
	if (lptimer_get_flag(OS_TIMER, LPTIM_ICR_ARRMCF)) {
		timer_elapsed = true;
		lptimer_clear_flag(OS_TIMER, LPTIM_ICR_ARRMCF);
	}
	if (lptimer_get_flag(OS_TIMER, LPTIM_ICR_CMPMCF)) {
		timer_elapsed = true;
		lptimer_clear_flag(OS_TIMER, LPTIM_ICR_CMPMCF);
	}
	if (lptimer_get_flag(OS_TIMER, LPTIM_ICR_CMPOKCF)) {
		lptimer_clear_flag(OS_TIMER, LPTIM_ICR_CMPOKCF);
		lptimer_disable_irq(OS_TIMER, LPTIM_ICR_CMPOKCF);
	}
}

static void os_sync_time(void) {
	uint32_t ticks_now = lptimer_get_counter(OS_TIMER);
	uint32_t ticks_delta;

	/*
	 * TODO : This is slightly flawed.
	 * if overflow happens and is checked too late
	 * it might not be caught. Add an additional
	 * overflow flag set from isr.
	 * Then check for the flag both, when overflow
	 * is detected and even if not. The check within
	 * the detection needs to have interrupts disabled
	 * and must set an additional overflow_handled flag
	 * to prevent races with the isr.
	 * The check outside is less critical. It can just
	 * check the flag and unset it if it was set. In theory
	 * there could be a race against the next overflow.
	 * However we assume that overflows are far enough apart
	 * for this routine to handle the first before another
	 * one happens
	 */
	if (ticks_now < last_timer_counter_sync) {
		// Overflow happened!
		ticks_delta = OS_TIMER_TOP;
		ticks_delta -= last_timer_counter_sync;
		ticks_delta += ticks_now;
	} else {
		ticks_delta = ticks_now - last_timer_counter_sync;
	}

	time_add_us(&last_os_time, TICKS_TO_US(ticks_delta));
	last_timer_counter_sync = ticks_now;
}

static void os_recalculate_next_deadline(void) {
	uint32_t ticks_now = lptimer_get_counter(OS_TIMER);
	os_time_t earliest;
	uint32_t delta_us, delta_ticks;

	if (!os_tasks) {
		next_task_timer_counter = OS_TIMER_TOP;
		return;
	}

	os_sync_time();
	// earliest deadline is guaranteed  to be list head
	earliest = os_tasks->deadline;

	if (TIME_GE(last_os_time, earliest)) {
		next_task_timer_counter = ticks_now;
		return;
	}

	delta_us = time_delta_us(&earliest, &last_os_time);
	delta_ticks = US_TO_TICKS(delta_us);
	if (OS_TIMER_TOP <= delta_ticks ||
	    OS_TIMER_TOP - delta_ticks < ticks_now) {
		next_task_timer_counter = OS_TIMER_TOP;
		return;
	}

	next_task_timer_counter = ticks_now + delta_ticks;
	next_task = os_tasks;
}

static void os_remove_task(os_task_t *removee) {
	if (os_tasks == removee) {
		os_tasks = removee->next;
	} else {
		os_task_t *task = os_tasks;
		while (task) {
			if (task->next == removee) {
				task->next = removee->next;
				break;
			}
			task = task->next;
		}
	}
	if (next_task == removee) {
		next_task = NULL;
	}
	removee->next = NULL;
}

static void os_add_task(os_task_t *insertee)  {
	insertee->next = NULL;
	// Adding a task twice would be fatal
	os_remove_task(insertee);
	if (os_tasks) {
		os_task_t *task = os_tasks;

		// Special case if earlier than current first task
		if (TIME_GE(os_tasks->deadline, insertee->deadline)) {
			insertee->next = os_tasks;
			os_tasks = insertee;
			return;
		}

		while (task->next) {
			if (TIME_GE(insertee->deadline, task->deadline) &&
			    TIME_GE(task->next->deadline, insertee->deadline)) {
				break;
			}
			task = task->next;
		}
		insertee->next = task->next;
		task->next = insertee;
	} else {
		os_tasks = insertee;
	}
}

void os_schedule_task_relative(os_task_t *task, os_task_f cb, uint32_t us, void *ctx) {
	os_time_t deadline;

	os_sync_time();
	deadline = last_os_time;
	time_add_us(&deadline, us);

	task->run = cb;
	task->deadline = deadline;
	task->ctx = ctx;

	os_add_task(task);
	os_recalculate_next_deadline();
}

/*
 * Setting up the timer compare match is a complicated procedure.
 * First of all writes to the compare register are async. They are synchronized
 * to the timer clock. Thus we need to wait before evaluating the counter
 * in relation to the compare value.
 * Additionally interrupts need to be disabled during evaluation to ensure we
 * did not miss the compare event.
 */
static void timer_set_compare(uint16_t val) {
	__disable_irq();
	lptimer_clear_flag(OS_TIMER, LPTIM_ICR_CMPOKCF);
	lptimer_enable_irq(OS_TIMER, LPTIM_ICR_CMPOKCF);
	lptimer_set_compare(OS_TIMER, val);
	while (!lptimer_get_flag(OS_TIMER, LPTIM_ICR_CMPOKCF)) {
		if (!inhibit_sleep) {
			if (inhibit_deep_sleep) {
				sleep_enter();
			} else {
				sleep_enter_stop();
			}
		}
	}
	lptimer_disable_irq(OS_TIMER, LPTIM_ICR_CMPOKCF);
	lptimer_clear_flag(OS_TIMER, LPTIM_ICR_CMPOKCF);
	timer_elapsed = lptimer_get_counter(OS_TIMER) >= val;
	__enable_irq();
}

void os_run() {
	uint32_t ticks_now = lptimer_get_counter(OS_TIMER);
	os_task_t *task = os_tasks;
	os_task_t *next;

	if (ticks_now >= last_run_timer_counter) {
		// Fast path
		if (ticks_now < next_task_timer_counter ||
		    next_task_timer_counter == OS_TIMER_TOP) {
			last_run_timer_counter = ticks_now;
			timer_set_compare(next_task_timer_counter);
			while (!timer_elapsed) {
				do_sleep(TICKS_TO_US(next_task_timer_counter - ticks_now));
			}
			return;
		} else {
			// Ensure minimum latency by executing deadline task first
			if (next_task) {
				os_remove_task(task);
				task->run(task->ctx);
			}
		}
	}
	// slow path

	os_sync_time();
	task = os_tasks;
	// There could be more tasks scheduled at the same time, find them!
	while (task) {
		next = task->next;
		if (TIME_GE(last_os_time, task->deadline)) {
			os_remove_task(task);
			task->run(task->ctx);
		}
		task = next;
	}
	os_recalculate_next_deadline();
	last_run_timer_counter = ticks_now;
}

static void os_delay_ticks(uint32_t ticks_now, uint32_t ticks) {
	uint32_t ticks_then = ticks_now + ticks;

	timer_set_compare(ticks_then);
	while (!timer_elapsed) {
		do_sleep(TICKS_TO_US(ticks));
	}
}

void os_delay(uint32_t us) {
	uint32_t ticks = US_TO_TICKS(us);

	while (ticks) {
		uint32_t ticks_now = lptimer_get_counter(OS_TIMER);
		uint32_t ticks_partial = ticks;

		if (ticks_partial >= OS_TIMER_TOP) {
			ticks_partial = OS_TIMER_TOP - 1;
		}
		if (OS_TIMER_TOP - ticks_partial < ticks_now) {
			ticks_partial = OS_TIMER_TOP - ticks_now;
		}

		os_delay_ticks(ticks_now, ticks_partial);

		if (ticks_now < last_timer_counter_sync) {
			os_sync_time();
		}

		ticks -= ticks_partial;
	}
}

uint32_t os_wait_flag_timeout(volatile uint32_t *mmio, uint32_t mask, uint32_t us) {
	uint32_t ticks = US_TO_TICKS(us);

	while (ticks) {
		uint32_t ticks_now = lptimer_get_counter(OS_TIMER);
		uint32_t ticks_partial = ticks;
		uint32_t ticks_then;
		uint32_t flags;

		if (ticks_partial >= OS_TIMER_TOP) {
			ticks_partial = OS_TIMER_TOP - 1;
		}
		if (OS_TIMER_TOP - ticks_partial < ticks_now) {
			ticks_partial =  OS_TIMER_TOP - ticks_now;
		}

		ticks_then = ticks_now + ticks_partial;

		timer_set_compare(ticks_then);
		while (!timer_elapsed) {
			sleep_enter_event();

			flags = *mmio & mask;
			if (flags) {
				return flags;
			}

		}

		if (ticks_now < last_timer_counter_sync) {
			os_sync_time();
		}

		flags = *mmio & mask;
		if (flags) {
			return flags;
		}

		ticks -= ticks_partial;
	}

	return 0;
}
