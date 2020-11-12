#include <string.h>

#include "ble.h"
#include "util.h"

// X^24 + X^10 + X^9 + X^6 + X^4 + X^3 + X + 1
#define CRC_POLY ((BITSWAP_U8(0x5B) << 16) | (BITSWAP_U8(0x06) << 8))

ble_channel_t ble_advertisement_channels[] = {
	{ 37, 2402 }, { 38, 2426 }, { 39, 2480 }
};

// Efficient LSB first BLE crc
static uint32_t ble_crc(uint8_t *data, uint16_t len) {
	uint32_t crc = 0xaaaaaa;

	while (len--) {
		uint8_t byte = *data++;
		uint8_t bit;

		for (bit = 0; bit < 8; bit++) {
			if ((crc & 0x01) ^ (byte & 0x01)) {
				crc = (crc >> 1) ^ CRC_POLY;
			} else {
				crc >>= 1;
			}
			byte >>= 1;
		}
	}

	return crc;
}

static void ble_whiten(uint8_t *data, uint16_t len, uint8_t channel) {
	uint8_t mask;
	uint8_t coeff = BITSWAP_U8(channel) | 0x02;

	while (len--) {
		for (mask = 1; mask; mask <<= 1) {
			if (coeff & 0x80) {
				coeff ^= 0x11;
				*data ^= mask;
			}
			coeff <<= 1;
		}
		data++;
	}
}

uint16_t ble_frame(uint8_t *dst, uint16_t dst_len, ble_header_t hdr, ble_mac_address_t mac_addr, uint8_t *payload, uint8_t payload_len, ble_channel_t *channel) {
	uint32_t crc;
	uint8_t *dst_start = dst;

	if (dst_len < sizeof(hdr) + 1 + sizeof(ble_mac_address_t) + payload_len + 3) {
		return 0;
	}

	*dst++ = hdr;
	*dst++ = payload_len + sizeof(ble_mac_address_t);
	memcpy(dst, mac_addr, sizeof(ble_mac_address_t));
	dst += 6;
	memcpy(dst, payload, payload_len);
	dst += payload_len;

	crc = ble_crc(dst_start, dst - dst_start);
	*dst++ = crc & 0xff;
	*dst++ = (crc >> 8) & 0xff;
	*dst++ = (crc >> 16) & 0xff;
	ble_whiten(dst_start, dst - dst_start, channel->channel);
	dst_len = dst - dst_start;
	while (dst-- > dst_start) {
		*dst = BITSWAP_U8(*dst);
	}
	return dst_len;
}

static int adv_idx = 0;

ble_channel_t *ble_get_advertisement_channel() {
	adv_idx %= ARRAY_SIZE(ble_advertisement_channels);
	return &ble_advertisement_channels[adv_idx++];
}

static ble_access_address_t advertisement_access_address = { BITSWAP_U8(0x8E), BITSWAP_U8(0x89), BITSWAP_U8(0xBE), BITSWAP_U8(0xD6) };

ble_access_address_t *ble_get_access_address() {
	return &advertisement_access_address;
}
