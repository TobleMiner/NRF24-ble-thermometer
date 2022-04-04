#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "os_time.h"
#include "util.h"

#define US_TO_TICKS(us) (DIV_ROUND_UP(us, 977))
#define MS_TO_TICKS(ms) US_TO_TICKS((ms) * 1000)
#define TICKS_TO_US(ticks) ((ticks) * 977)

typedef void (*os_task_f)(void *ctx);

struct os_task;

struct os_task {
	struct os_task *next;
	void *ctx;
	os_time_t deadline;
	os_task_f run;
};

typedef struct os_task os_task_t;

void os_init(void);
void os_schedule_task_relative(os_task_t *task, os_task_f cb, uint32_t us, void *ctx);
void os_run(void);
void os_delay(uint32_t us);
uint32_t os_wait_flag_timeout(volatile uint32_t *mmio, uint32_t mask, uint32_t us);
void os_inhibit_deep_sleep(bool inhibit);
void os_inhibit_sleep(bool inhibit);
