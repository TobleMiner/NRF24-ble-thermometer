#pragma once

#include <stdint.h>

#define MS_TO_US(ms) ((ms) * 1000)
#define SEC_TO_US(s) (MS_TO_US((s) * 1000))

#define TIME_GE(a, b) \
	((a).s > (b).s || \
	((a).s == (b).s && (a).us >= (b).us))

typedef struct {
	uint32_t s;
	uint32_t us;
} os_time_t;

uint32_t time_delta_us(os_time_t *later, os_time_t *earlier);

static inline void time_normalize(os_time_t *time) {
	while (time->us >= SEC_TO_US(1)) {
		time->us -= SEC_TO_US(1);
		time->s++;
	}
}

static inline void time_add_us(os_time_t *time, uint32_t us) {
	time->us += us;
	time_normalize(time);
}
