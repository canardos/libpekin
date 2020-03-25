#include <cstdint>
#include <libpekin.h>
#include <display/ili9481_cmd_defs.h>
#include "display/lcd_bus8_bit.h"

LcdBus8Bit::LcdBus8Bit() {
    // TODO: PB3->7 hard coded
    CONTROL_GPIO_PORT->CRL = (CONTROL_GPIO_PORT->CRL & 0x00000FFF) | 0x33333000;
    SET_CS_IDLE;
    SET_WR_IDLE;
    SET_RD_IDLE;
    // Keep in data mode
    // keep in write mode
    setBusModeWrite();
}

LcdBus8Bit::~LcdBus8Bit() { }

void LcdBus8Bit::setBusModeRead(void) {
    // lower 8-bits
    DATA_GPIO_PORT->CRL = 0x88888888;
}

void LcdBus8Bit::setBusModeWrite(void) {
    // lower 8-bits
    DATA_GPIO_PORT->CRL = 0x33333333;
}

void LcdBus8Bit::rawWrite8(uint8_t data) {
#if DATA_GPIO_PORT_USE_WHOLE_PORT
    // we're not using the upper 8 bits so faster to write the whole port
    DATA_GPIO_PORT->ODR = data;
#else
    // Faster than shifting reset bits up (i.e. ((~c)<<16) | c)
    // Tested on STM32F103
    DATA_GPIO_PORT->BSRR = c;
    DATA_GPIO_PORT->BRR = ~c;
#endif
    SET_WR_ACTIVE;
    SET_WR_IDLE;
}

void inline LcdBus8Bit::rawWrite16(uint16_t data) {
    // MSB first
    rawWrite8(data >> 8);
    rawWrite8(data & 0xff);
}


uint8_t LcdBus8Bit::rawRead8() {
    SET_RD_ACTIVE;

    // TODO: this is supposed to be a generic bus - add timing params

    // wait for module (ILI9481 read access time max = 340ns)
    Libp::delayMs(1);
    // Lower 8-bits
    uint8_t temp = (DATA_GPIO_PORT->IDR & 0x00ff);
    SET_RD_IDLE;
    return temp;
}


uint16_t LcdBus8Bit::rawRead16() {
    // MSB first
    uint8_t hi = rawRead8();
    uint8_t lo = rawRead8();
    return (hi << 8) | lo;
}


void LcdBus8Bit::write8(uint8_t data) {
    SET_CS_ACTIVE;
    rawWrite8(data);
    SET_CS_IDLE;
}

void LcdBus8Bit::write16(uint16_t data) {
    SET_CS_ACTIVE;
    rawWrite8(data >> 8);
    rawWrite8(data & 0xff);
    SET_CS_IDLE;
}

void LcdBus8Bit::write8Repeat(uint8_t data, uint32_t n) {
    SET_CS_ACTIVE;
    while (n-- > 0) {
        rawWrite8(data);
    }
    SET_CS_IDLE;
}

void LcdBus8Bit::write16Repeat(uint16_t data, uint32_t n) {
    SET_CS_ACTIVE;
    while (n-- > 0) {
        rawWrite16(data);
    }
    SET_CS_IDLE;
}

void LcdBus8Bit::write8Array(const uint8_t* data, uint32_t len) {
    SET_CS_ACTIVE;
    for (uint32_t i = 0; i < len; i++) {
        rawWrite8(data[i]);
    }
    SET_CS_IDLE;
}
void LcdBus8Bit::write16Array(const uint16_t* data, uint32_t len) {
    SET_CS_ACTIVE;
    for (uint32_t i = 0; i < len; i++) {
        rawWrite16(data[i]);
    }
    SET_CS_IDLE;
}

void LcdBus8Bit::write8Cmd(uint8_t cmd) {
    SET_RS_REG;
    SET_CS_ACTIVE;
    rawWrite8(cmd);
    SET_CS_IDLE;
    SET_RS_DATA;
}

uint8_t LcdBus8Bit::read8() {
    setBusModeRead();
    SET_CS_ACTIVE;
    uint8_t data = rawRead8();
    SET_CS_IDLE;
    setBusModeWrite();
    return data;
}

uint16_t LcdBus8Bit::read16() {
    setBusModeRead();
    SET_CS_ACTIVE;
    uint16_t data = rawRead16();
    SET_CS_IDLE;
    setBusModeWrite();
    return data;
}
