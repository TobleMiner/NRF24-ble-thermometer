#pragma once

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(*(arr)))

#define paddr__(x) ((uint32_t)(x))

#define BIT(x) (1 << (x))

#define KHZ(x) ((x) * 1000UL)
#define MHZ(x) (KHZ((x)) * 1000UL)

#define SWAP(a, b) \
	do { \
		__typeof__(a) tmp_ = b; \
		b = a; \
		a = tmp_; \
	} while(0)

#define MIN(a, b) ((a) < (b) ? (a) : (b))

#define DIV_ROUND_UP(u, v) (((u) + ((v) - 1)) / (v))

#define BITSWAP_U8(x) \
	((((x) & 0x80) >> 7) | \
	(((x) & 0x40) >> 5) | \
	(((x) & 0x20) >> 3) | \
	(((x) & 0x10) >> 1) | \
	(((x) & 0x08) << 1) | \
	(((x) & 0x04) << 3) | \
	(((x) & 0x02) << 5) | \
	(((x) & 0x01) << 7))
