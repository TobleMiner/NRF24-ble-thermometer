#pragma once

#include <stdint.h>

typedef struct {
	uint8_t channel;
	unsigned frequency_mhz;
} ble_channel_t;

typedef uint8_t ble_mac_address_t[6];
typedef uint8_t ble_access_address_t[4];
typedef uint8_t ble_header_t;

/* BLE header structure */
#define BLE_HEADER(pdu_type, ch_sel, tx_add, rx_add) \
        ((pdu_type) | (ch_sel) << 5 | (tx_add) << 6 | (rx_add) << 7)

/* BLE 5.0 PDU types */
#define BLE_PDU_TYPE_ADV_IND         0b0000
#define BLE_PDU_TYPE_ADV_DIRECT_IND  0b0001
#define BLE_PDU_TYPE_ADV_NONCONN_IND 0b0010
#define BLE_PDU_TYPE_SCAN_REQ        0b0011
#define BLE_PDU_TYPE_AUX_SCAN_REQ    0b0011
#define BLE_PDU_TYPE_SCAN_RSP        0b0100
#define BLE_PDU_TYPE_CONNECT_REQ     0b0101
#define BLE_PDU_TYPE_AUX_CONNECT_REQ 0b0101
#define BLE_PDU_TYPE_ADV_SCAN_IND    0b0110
#define BLE_PDU_TYPE_ADV_EXT_IND     0b0111
#define BLE_PDU_TYPE_AUX_ADV_IND     0b0111
#define BLE_PDU_TYPE_AUX_SCAN_RSP    0b0111
#define BLE_PDU_TYPE_AUX_SYNC_IND    0b0111
#define BLE_PDU_TYPE_AUX_CHAIN_IND   0b0111
#define BLE_PDU_TYPE_AUX_CONNECT_RSP 0b1000

uint16_t ble_frame(uint8_t *dst, uint16_t dst_len, ble_header_t hdr, ble_mac_address_t mac_addr, uint8_t *payload, uint8_t payload_len, ble_channel_t *channel);
ble_channel_t *ble_get_advertisement_channel(void);
ble_access_address_t *ble_get_access_address(void);
