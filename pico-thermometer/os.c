#include <stddef.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/lptimer.h>
#include <libopencm3/stm32/rcc.h>

#include "os.h"

#define OS_TIMER LPTIM1
#define OS_TIMER_RCC RCC_LPTIM1
#define OS_TIMER_RST RST_LPTIM1
#define OS_TIMER_TOP 0xFFFF
#define OS_TIMER_IRQ NVIC_LPTIM1_IRQ
#define OS_TIMER_IRQ_HANDLER lptim1_isr
#define OS_TIMER_TOP 0xFFFF

static uint32_t last_timer_counter_sync = 0;
static os_time_t last_os_time = { 0, 0 };
static uint32_t next_task_timer_counter = 0;
static uint32_t last_run_timer_counter = 0;

static os_task_t *os_tasks;

void os_init() {
	// 1ms resolution RTOS timer
	rcc_periph_clock_enable(OS_TIMER_RCC);
	rcc_periph_reset_pulse(OS_TIMER_RST);
	rcc_osc_on(RCC_LSI);
	rcc_set_peripheral_clk_sel(LPTIM1, RCC_CCIPR_LPTIM1SEL_LSI);
	// 32kHz LSI / 32 = 1kHz
	lptimer_set_internal_clock_source(OS_TIMER);
	lptimer_set_prescaler(OS_TIMER, LPTIM_CFGR_PRESC_32);
	lptimer_enable(OS_TIMER);
	lptimer_set_period(OS_TIMER, OS_TIMER_TOP);
//	lptimer_enable_irq(OS_TIMER, LPTIM_IER_ARRMIE | LPTIM_IER_CMPMIE);
//	nvic_enable_irq(NVIC_LPTIM1_IRQ);
	lptimer_start_counter(OS_TIMER, LPTIM_CR_CNTSTRT);
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

void os_run() {
	uint32_t ticks_now = lptimer_get_counter(OS_TIMER);
	os_task_t *task = os_tasks;
	os_task_t *next;

	if (ticks_now > last_run_timer_counter) {
		// Fast path
		if (ticks_now < next_task_timer_counter) {
			last_run_timer_counter = ticks_now;
			return;
		} else {
			// Ensure minimum latency by executing deadline task first
			if (task) {
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

static void os_delay_slowpath(uint32_t us) {
	os_time_t deadline;

	os_sync_time();
	deadline = last_os_time;
	time_add_us(&deadline, us);
	while (TIME_GE(deadline, last_os_time)) {
		os_sync_time();
	}
}

void os_delay(uint32_t us) {
	uint32_t ticks_now = lptimer_get_counter(OS_TIMER);
	uint32_t ticks = US_TO_TICKS(us);
	if (OS_TIMER_TOP <= ticks ||
	    OS_TIMER_TOP - ticks < ticks_now) {
		os_delay_slowpath(us);
	} else {
		uint32_t ticks_then = ticks_now + ticks;
		while (lptimer_get_counter(OS_TIMER) < ticks_then) {
			// depend on ticks_then to make absolutely sure this not optimzed out
			__asm__ volatile("" : "+g" (ticks_then) : :);
		}
	}
}
