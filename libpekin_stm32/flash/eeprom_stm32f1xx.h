/**
 * Relies on STM32 HAL drivers
 */
#ifndef SRC_STM32_EEPROM_RAW_H_
#define SRC_STM32_EEPROM_RAW_H_

#include <cstdint>
#include "libpekin_stm32_hal.h"
#include "flash_stm32f1xx.h"

namespace LibpStm32 {

// TODO: Hide this class
/**
 * Do not use this class directly. Use @p Stm32Eeprom
 */
// TODO: rename
class EepromRaw {
    //template <typename T, uint16_t signature>
    //friend class Stm32Eeprom;
public:
    /**
     * 2 sequential flash pages are required.
     *
     * @param flash_base_addr base address of first page to use for storage.
     *                        This page and the following page will be used.
     * @param data_len length in bytes of object to be stored. Must be a
     *                 multiple of 2 and <= [ (Flash::pageSize() - sizeof(Header)
     *                 - sizeof(signature)*2) ]
     * @param signature should be unique for the data type being stored.
     *                  may not be 0xFFFF or 0x0000.
     */
    EepromRaw(
            uint32_t flash_base_addr, uint16_t data_len, uint16_t signature) :
                addr_page0_  { flash_base_addr},
                addr_page1_  { flash_base_addr + Flash::pageSize() },
                data_len_    { data_len },
                storage_len_ { static_cast<uint16_t>(data_len + sizeof(signature) * 2) },
                signature_   { signature }
    {
        // TODO: we need to be able to throw an error here. Should not do on construction
        verifyAndInit();
    }
    /**
     * @param data length must match that provided on construction
     * @return
     */
    bool write(const uint16_t* data);
    /**
     * @return @p nullptr if no data is present
     */
    const uint16_t* get();

private:
    static constexpr uint16_t page_state_inuse = 0xffff;
    static constexpr uint16_t page_state_full = 0x0000;
    struct Header {
        /// Used to identify that this page has previously
        /// been setup to store this type of data
        uint16_t signature_; // must be first
        uint16_t state;
    };

    const uint32_t addr_page0_;
    const uint32_t addr_page1_;
    /// block length in bytes excluding leading/trailing signature
    const uint16_t data_len_;
    /// block length in bytes including leading/trailing signature
    const uint16_t storage_len_;
    /// unique number used to identify start/end of data storage and start of
    /// page
    const uint16_t signature_;
    /// Offset from page start to start of last
    /// storage location (before the page is full)
    const uint16_t last_slot_page_offs_ =
            sizeof(Header) + ((Flash::pageSize() - sizeof(Header)) / storage_len_ * storage_len_) - storage_len_;

    static Header* header(uint32_t addr);

    static bool pageInUse(uint32_t page_base_addr);

    //static uint16_t state(uint32_t page_base_addr);

    void verifyAndInit();

    void fixIncompleteWrite();

    uint32_t getCurrentPageAddr();

    uint32_t getOtherPageAddr(uint32_t addr);

    bool formatPage(uint32_t base_addr);

    void setPageStatusFull(uint32_t base_addr);

    uint32_t findFreeBlock(uint32_t page_base_addr);

    uint32_t findStoredBlock(uint32_t page_base_addr);

    bool writeBlock(uint32_t addr, const uint16_t* data);
};

// Thin template wrapper providing type safety and static param checks
/**
 * EEPROM emulation class using flash storage for STM32 devices.
 *
 * Two flash pages are used for storage to provide for recovery of prior data
 * if power is lost during writes.
 *
 * `signature` is used to identify previously written data and should be
 * unique if multiple `Stm32Eeprom` objects are being instantiated for
 * different data types.
 *
 * @tparam T data type to be stored. Must be 16-bit aligned and size must be
 *           <= (Flash::pageSize() - 8)
 * @tparam base_addr flash storage location base address. Must be the start of
 *                   a page. Two pages starting at this location will be used.
 *                   This should usually be the start of the last 2 pages.
 * @tparam signature unique number for this type/location. May not be 0x0000 or
 *                   0xffff.
 */
// TODO: can we make base_addr default to last 2 pages? - we don't know end addr at compile time
template <typename T, uint32_t base_addr, uint16_t signature>
// TODO: rename
class Eeprom {
public:
    static_assert(sizeof(T) % 2 == 0, "T must be 16-bit aligned");
    static_assert(sizeof(T) + sizeof(EepromRaw::Header::signature_)*2 <= Flash::pageSize() - sizeof(EepromRaw::Header), "T is too large" );
    static_assert(base_addr % Flash::pageSize() == 0, "base_addr must be the beginning of a page");
    static_assert(signature != 0xFFFF && signature != 0x0000, "signature may not be 0x0000 or 0xFFFF");
    /**
     * Write data.
     *
     * @param data data to be written
     * @return true if data is successfully written, false otherwise.
     */
    inline bool write(const T* data)
    {
        return eeprom.write(reinterpret_cast<const uint16_t*>(data));
    }
    /**
     * Return a pointer to the most recently written object.
     *
     * @return @p nullptr if not found
     */
    inline const T* get()
    {
        return reinterpret_cast<const T*>(eeprom.get());
    }
private:
    EepromRaw eeprom = EepromRaw(base_addr, sizeof(T), signature);
};

} // namespace LibpStm32

#endif /* SRC_STM32_EEPROM_RAW_H_ */
