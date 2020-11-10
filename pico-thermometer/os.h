#pragma once

#include <stdint.h>

#include "os_time.h"
#include "util.h"

#define US_TO_TICKS(us) (DIV_ROUND_UP(us, 1000))
#define MS_TO_TICKS(ms) US_TO_TICKS((ms) * 1000)
#define TICKS_TO_US(ticks) ((ticks) * 1000)

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