#ifndef LIB_LIBPEKIN_STM32_FLASH_STM32F1XX_H_
#define LIB_LIBPEKIN_STM32_FLASH_STM32F1XX_H_

#include <cstdint>
#include "libpekin.h"
#include "bits.h"
#include "flash_stm32f1xx_stm.h"

namespace LibpStm32 {

/**
 * Functions to erase/write flash.
 *
 * Warning: Largely untested
 */
class Flash {
private:
    static inline FLASH_TypeDef* const flash_ = reinterpret_cast<FLASH_TypeDef*>(FLASH_R_BASE);
    static constexpr uint32_t rdprt_key = 0x00A5;
    static constexpr uint32_t key1 = 0x45670123;
    static constexpr uint32_t key2 = 0xCDEF89AB;

public:
    /**
     * Return the flash size in bytes.
     *
     * @return
     */
    static uint32_t size()
    {
        return endAddr() - startAddr();
    }

    /**
     * Return the flash starting address.
     *
     * @return
     */
    static constexpr uint32_t startAddr()
    {
        return FLASH_BASE;
    }

    /**
     * Return the flash ending address.
     *
     * @return
     */
    static uint32_t endAddr()
    {

        // TODO: Can we detect the flash size from macro defs without reading the SIZE reg and make this constexpr?

    /* Low Density */
    #if (defined(STM32F101x6) || defined(STM32F102x6) || defined(STM32F103x6))

        return ((*((uint16_t *)FLASH_SIZE_DATA_REGISTER)) == 0x20U)
                ? FLASH_BANK1_END
                : 0x08003FFFU;

    #endif /* STM32F101x6 || STM32F102x6 || STM32F103x6 */

    /* Medium Density */
    #if (defined(STM32F100xB) || defined(STM32F101xB) || defined(STM32F102xB) || defined(STM32F103xB))

        return ((*((uint16_t *)FLASH_SIZE_DATA_REGISTER)) == 0x80U)
                ? FLASH_BANK1_END
                : ((*((uint16_t *)FLASH_SIZE_DATA_REGISTER)) == 0x40U)
                        ? 0x0800FFFFU
                        : ((*((uint16_t *)FLASH_SIZE_DATA_REGISTER)) == 0x20U)
                                ? 0x08007FFFU
                                : 0x08003FFFU;

    #endif /* STM32F100xB || STM32F101xB || STM32F102xB || STM32F103xB*/

    /* High Density */
    #if (defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F103xE))

        return ((*((uint16_t *)FLASH_SIZE_DATA_REGISTER)) == 0x200U)
                ? FLASH_BANK1_END
                : ((*((uint16_t *)FLASH_SIZE_DATA_REGISTER)) == 0x180U)
                        ? 0x0805FFFFU
                        : 0x0803FFFFU;

    #endif /* STM32F100xE || STM32F101xE || STM32F103xE */

    /* XL Density */
    #if defined(FLASH_BANK2_END)

        return ((*((uint16_t *)FLASH_SIZE_DATA_REGISTER)) == 0x400U)
                ? FLASH_BANK2_END
                : 0x080BFFFFU;

    #endif /* FLASH_BANK2_END */

    /* Connectivity Line */
    #if (defined(STM32F105xC) || defined(STM32F107xC))

        return ((*((uint16_t *)FLASH_SIZE_DATA_REGISTER)) == 0x100U)
                ? FLASH_BANK1_END
                : ((*((uint16_t *)FLASH_SIZE_DATA_REGISTER)) == 0x80U)
                      ? 0x0801FFFFU
                      : 0x0800FFFFU;

    #endif /* STM32F105xC || STM32F107xC */
    }

    /**
     * Return the flash page size in bytes.
     *
     * @return
     */
    static constexpr uint32_t pageSize()
    {
    /*
     32 1kb low den
     128 1kb med
     256 2kb high
     128 2kb connectivity
     */
    #if (defined(STM32F101x6) || defined(STM32F102x6) || defined(STM32F103x6) || defined(STM32F100xB) || defined(STM32F101xB) || defined(STM32F102xB) || defined(STM32F103xB))
        return 0x400U;
    #endif /* STM32F101x6 || STM32F102x6 || STM32F103x6 */
           /* STM32F100xB || STM32F101xB || STM32F102xB || STM32F103xB */

    #if (defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F103xE) || defined(STM32F101xG) || defined(STM32F103xG) || defined(STM32F105xC) || defined(STM32F107xC))
        return 0x800U;
    #endif /* STM32F100xB || STM32F101xB || STM32F102xB || STM32F103xB */
           /* STM32F101xG || STM32F103xG */
           /* STM32F105xC || STM32F107xC */
    }


    /**
     * Unlock the flash for erasing/writing.
     *
     * @return true on sucess
     */
    static bool unlock()
    {
        flash_->KEYR = key1;
        flash_->KEYR = key2;
#if defined(FLASH_BANK2_END)
        flash_->KEYR2 = key1;
        flash_->KEYR2 = key2;
        return !((FLASH->CR2 & FLASH_CR2_LOCK) | (FLASH->CR & FLASH_CR_LOCK));
#endif
        return !(FLASH->CR & FLASH_CR_LOCK);
    }

    /**
     * Lock the flash after erasing/writing to prevent accidental changes.
     */
    static void lock()
    {
        flash_->CR |= FLASH_CR_LOCK;
#if defined(FLASH_BANK2_END)
        flash_->CR2 |= FLASH_CR2_LOCK;
#endif
    }

    enum class Bank : uint8_t {
        one, two, both
    };

    /**
     * Erase the full flash bank(s).
     *
     * Not all devices have a second bank. On a device with only one bank,
     * specifying bank two is a NOOP and specifying both will erase bank one.
     *
     * @param bank
     */
    static void eraseBank(Bank bank = Bank::one)
    {
        // Bank1 erase
        if (bank != Bank::two) {
            flash_->CR |= FLASH_CR_MER;
            flash_->CR |= FLASH_CR_STRT;
        }
#if defined(FLASH_BANK2_END)
        if (bank != bank::one) {
            flash_->CR2 |= FLASH_CR2_MER;   // Bank2 mass erase
            flash_->CR2 |= FLASH_CR2_STRT;  // Bank2 start
            waitForFinishBank2();
            flash_->CR2 &= ~FLASH_CR2_MER;
        }
#endif
        waitForFinishBank1();
        flash_->CR &= ~FLASH_CR_MER;
    }

    /**
     *
     * @param start_addr
     * @param ng_pages
     *
     * @return
     */
    // TODO: is addr start of page only? need to validate?
    static bool erasePages(uint32_t start_addr, uint16_t nb_pages)
    {
        if ( !isValidFlashProgAddr(start_addr) || !isValidFlashPage(start_addr, nb_pages) )
            return false;

        while (nb_pages-- > 0) {
            if (!erasePage(start_addr))
                return false;
            start_addr += pageSize();
        }
        return true;
    }


    /**
     * Program must occur on 16-bit boundaries
     *
     * @param addr destination address
     * @param data
     * @param len array length (i.e. a len of 2 == 4 bytes)
     *
     * @return
     */
    static bool program(uint32_t addr, const uint16_t* data, uint32_t len)
    {
        const uint32_t end_addr =  addr + len * 2 - 1;

        if ( !isValidFlashProgAddr(addr) || !isValidFlashProgAddr(end_addr) )
            return false;

        waitForFinishBank1();
        while (addr <= FLASH_BANK1_END && addr < end_addr) {
            uint16_t* pFlash = reinterpret_cast<uint16_t*>(addr);
            flash_->CR |= FLASH_CR_PG;
            *pFlash = *data;
            if (!waitForFinishBank1()) {
                // TODO: clear PG?
                return false;
            }
            if (*pFlash != *data)
                return false;
            data++;
            addr += 2;
        }
#if defined(FLASH_BANK2_END)
        waitForFinishBank2();
        while (addr < end_addr) {
            uint16_t* pFlash = reinterpret_cast<uint16_t*>(addr);
            flash_->CR2 |= FLASH_CR2_PG;
            *pFlash = *data;
            if (!waitForFinishBank2()) {
                // TODO: clear PG?
                return false;
            }
            if (*pFlash != *data)
                return false;
            data++;
            addr += 2;
        }
#endif
        return true;
    }

private:
    inline __attribute__((always_inline))
    static bool errorCondition()
    {
        return (flash_->SR  & (FLASH_SR_PGERR | FLASH_SR_WRPRTERR))
            || (flash_->OBR & FLASH_OBR_OPTERR);
    }

    /**
     * Waits until the busy flag is clear, clears end of op flag
     *
     * @return
     */
    inline __attribute__((always_inline))
    static bool waitForFinishBank1()
    {
        while (flash_->SR & FLASH_SR_BSY)
            ;
        // Clear end of operation flag
        if (flash_->SR & FLASH_SR_EOP)
            flash_->SR |= FLASH_SR_EOP;

        return !errorCondition();
    }

#if defined(FLASH_BANK2_END)
    inline __attribute__((always_inline))
    static void waitForFinishBank2()
    {
        while (flash_->SR2 & FLASH_SR2_BSY)
            ;
        // Clear end of operation flag
        if (flash_->SR2 & FLASH_SR2_EOP)
            flash_->SR2 |= FLASH_SR2_EOP;

        return !errorCondition();
    }
#endif


    /**
     * Erase a single page. Assumes page address is valid
     *
     * @param page_addr
     *
     * @return true on success, false on error
     */
    inline __attribute__((always_inline))
    static bool erasePage(uint32_t page_addr)
    {
        // It's not clear from the docs whether page_addr needs to be the start
        // address or any address within the page to erase. So we assume it's
        // correct and don't validate here.
        // Validation to ensure it's already within the flash address range has
        // already been done.
#if defined(FLASH_BANK2_END)
        if (page_addr > FLASH_BANK1_END) {
            flash_->CR2 |= FLASH_CR2_PER;   // Bank2 page erase
            flash_->AR2 = page_addr;        // Set address
            flash_->CR2 |= FLASH_CR2_STRT;  // Bank2 start
            if (!waitForFinishBank2())
                return false;
            flash_->CR2 &= ~FLASH_CR2_PER;
        }
        else {
#endif
            flash_->CR |= FLASH_CR_PER;     // Bank1 page erase
            flash_->AR = page_addr;         // Set address
            flash_->CR |= FLASH_CR_STRT;    // Bank1 start
            if (!waitForFinishBank1())
                return false;
            flash_->CR &= ~FLASH_CR_PER;
#if defined(FLASH_BANK2_END)
        }
#endif
        return true;
    }
};

} // namespace LibpStm32 {

#endif /* LIB_LIBPEKIN_STM32_FLASH_STM32F1XX_H_ */
