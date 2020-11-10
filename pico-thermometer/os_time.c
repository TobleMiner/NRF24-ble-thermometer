#include "os_time.h"

uint32_t time_delta_us(os_time_t *later, os_time_t *earlier) {
	uint32_t delta_s = later->s - earlier->s;
	uint32_t delta_us = 0;

	if (earlier->us > later->us) {
		// This will underflow of later < earlier!
		delta_s--;
		delta_us = SEC_TO_US(1);
	}
	delta_us += delta_s * SEC_TO_US(1);
	delta_us += later->us;
	delta_us -= earlier->us;

	return delta_us;
}
