#ifndef SRC_DISPLAY_LCD_BUS8_BIT_H_
#define SRC_DISPLAY_LCD_BUS8_BIT_H_

#include <cstdint>
#include <libpekin_stm32_hal.h>
#include "bus/i_basic_bus_16.h"

//TODO: Migrate to use GpioPort

//*************************************************
// GPIO Settings - adjust as required
//*************************************************

// Port to use for data (bits 0-7)
#define DATA_GPIO_PORT                GPIOA

// Set to 1 for faster writes if nothing else
// is connected to bits 8-15 on the data port.
#define DATA_GPIO_PORT_USE_WHOLE_PORT 1

#define CONTROL_GPIO_PORT             GPIOB

#define CONTROL_PORT_RS_BIT           2
#define CONTROL_PORT_RD_BIT           3
#define CONTROL_PORT_WR_BIT           4
#define CONTROL_PORT_CS_BIT           6
#define CONTROL_PORT_RST_BIT          5

//*************************************************

#define SET_RS_REG     CONTROL_GPIO_PORT->BRR  = (1<<CONTROL_PORT_RS_BIT)
#define SET_RS_DATA    CONTROL_GPIO_PORT->BSRR = (1<<CONTROL_PORT_RS_BIT)
#define SET_RD_ACTIVE  CONTROL_GPIO_PORT->BRR  = (1<<CONTROL_PORT_RD_BIT)
#define SET_RD_IDLE    CONTROL_GPIO_PORT->BSRR = (1<<CONTROL_PORT_RD_BIT)
#define SET_WR_ACTIVE  CONTROL_GPIO_PORT->BRR  = (1<<CONTROL_PORT_WR_BIT)
#define SET_WR_IDLE    CONTROL_GPIO_PORT->BSRR = (1<<CONTROL_PORT_WR_BIT)
#define SET_CS_ACTIVE  CONTROL_GPIO_PORT->BRR  = (1<<CONTROL_PORT_CS_BIT)
#define SET_CS_IDLE    CONTROL_GPIO_PORT->BSRR = (1<<CONTROL_PORT_CS_BIT)

#define SET_RST_ACTIVE GPIOC->BRR  = (1<<CONTROL_PORT_RST_BIT)
#define SET_RST_IDLE   GPIOC->BSRR = (1<<CONTROL_PORT_RST_BIT)

/**
 * Implementation of the IBasicBus16 interface for STM32 using 8-bit 8080 style
 * over GPIO.
 *
 * See IBasicBus16 for member function documentation.
 */
class LcdBus8Bit : public Libp::IBasicBus16 {
public:
    LcdBus8Bit();
    virtual ~LcdBus8Bit();

    void write8(uint8_t data);
    void write16(uint16_t data);
    void write8Cmd(uint8_t cmd);

    void write8Repeat(uint8_t data, uint32_t n);
    void write16Repeat(uint16_t data, uint32_t n);

    void write8Array(const uint8_t* data, uint32_t len);
    void write16Array(const uint16_t* data, uint32_t len);

    uint8_t read8();
    uint16_t read16();

private:
    void setBusModeRead();
    void setBusModeWrite();

    /**
     * This function does not alter the CS or the D/C line.
     *
     * @param data
     */
    void rawWrite8(uint8_t data);
    /**
     * This function does not alter the CS or the D/C line.
     *
     * @param data
     */

    void rawWrite16(uint16_t data);

    /**
     * This function does not alter the CS or the D/C line.
     *
     * @return
     */
    uint8_t rawRead8();

    /**
     * This function does not alter the CS or the D/C line.
     *
     * @return
     */
    uint16_t rawRead16();
};

#endif /* SRC_DISPLAY_LCD_BUS8_BIT_H_ */
