#pragma once

int i2c_transfer(uint32_t i2c, uint8_t address, uint8_t *tx, uint8_t txlen, uint8_t *rx, uint8_t rxlen);
