#include <flash/eeprom_stm32f1xx.h>
#include "stddef.h"

using namespace LibpStm32;

/// Flash must be unlocked
/// length in 16-bit half words
/*static bool flashWrite(uint32_t addr, const uint16_t* data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, data[i]) != HAL_OK)
            return false;
        addr += sizeof(uint16_t);
    }
    return true;
}*/

inline
static uint16_t addrToContent(uint32_t addr)
{
    return *reinterpret_cast<uint16_t*>(addr);
}

inline
static uint16_t* addrToPtr(uint32_t addr)
{
    return reinterpret_cast<uint16_t*>(addr);
}

inline
EepromRaw::Header* EepromRaw::header(uint32_t page_base_addr)
{
    return reinterpret_cast<Header*>(page_base_addr);
}

/*inline
uint16_t EepromRaw::state(uint32_t page_base_addr)
{
    return reinterpret_cast<Header*>(page_base_addr)->state;
}*/

inline
bool EepromRaw::pageInUse(uint32_t page_base_addr)
{
    return reinterpret_cast<Header*>(page_base_addr)->state == page_state_inuse;
}

/// Prepare pages for first run / correct errors caused by incomplete writes
void EepromRaw::verifyAndInit()
{
    // TODO: use formatPage return values

    // A valid page is one that has been previously initialized
    const bool page0_valid = header(addr_page0_)->signature_ == signature_;
    const bool page1_valid = header(addr_page1_)->signature_ == signature_;

    if (page0_valid) {
        if (page1_valid) {
            // 2 valid pages, but no in use pages: should not be possible
            // Format page0 for use
            if (!pageInUse(addr_page0_) && !pageInUse(addr_page1_)) {
                formatPage(addr_page0_);
            }
            // 2 valid in use pages: must have lost power during page status switch
            // Roll back page switch - last save will be lost
            else if (pageInUse(addr_page0_) && pageInUse(addr_page1_)) {
                bool page0_full = addrToContent(addr_page0_ + last_slot_page_offs_) == signature_;
                setPageStatusFull(page0_full ? addr_page1_ : addr_page0_);
            }
        }
        else {
            // Only page0 valid, but not in use: should not be possible
            // Format page0 for use
            if (!pageInUse(addr_page0_)) {
                formatPage(addr_page0_);
            }
        }
    }
    // Page 0 invalid
    else {
        if (page1_valid) {
            // Only page1 valid, but not in use: should not be possible
            // Format page0 for use
            if (!pageInUse(addr_page1_)) {
                formatPage(addr_page0_);
            }
        }
        // No valid pages: first run with this type
        // Format page0 for use
        else {
            formatPage(addr_page0_);
        }
    }
    // Check and fix incomplete data writes
    uint32_t addr = findStoredBlock(getCurrentPageAddr());
    if (addr != 0 && addrToContent(addr + storage_len_ - sizeof(signature_)) != signature_)
        fixIncompleteWrite();
}

void EepromRaw::fixIncompleteWrite()
{
    uint32_t page_addr = getCurrentPageAddr();
    uint32_t addr = findStoredBlock(page_addr);
    if (addr == page_addr + sizeof(Header)) {
        // This is the first slot, but incomplete write on page switch already
        // rolled back in verifyAndInit, so this must be the first write ever.
        // Nothing to roll back to, so reformat page
        formatPage(page_addr);
        return;
    }
    if (addr + storage_len_ <= last_slot_page_offs_) {
        // There is free space - copy valid block to block after the incomplete write
        writeBlock(addr + storage_len_, addrToPtr(addr - storage_len_));
    }
    else {
        // No space remaining - switch to new page, copy, and invalidate this page
        uint32_t new_page_addr = getOtherPageAddr(page_addr);
        formatPage(new_page_addr);
        writeBlock(new_page_addr + sizeof(Header), addrToPtr(addr - storage_len_));
        setPageStatusFull(page_addr);
    }
}


bool EepromRaw::write(const uint16_t* data)
{
    uint32_t page_addr = getCurrentPageAddr();
    uint32_t free_addr = findFreeBlock(page_addr);
    if (free_addr == 0) {
        // Page is full, switch pages
        uint32_t new_page_addr = getOtherPageAddr(page_addr);
        formatPage(new_page_addr);
        free_addr = findFreeBlock(new_page_addr);
        bool success = writeBlock(free_addr, data);
        setPageStatusFull(page_addr);
        return success;
    }
    return writeBlock(free_addr, data);
}

const uint16_t* EepromRaw::get() {
    uint32_t page_addr = getCurrentPageAddr();
    uint32_t addr = findStoredBlock(page_addr);
    return addr == 0
            ? nullptr
            // return ptr to data, not signature
            : addrToPtr(addr + sizeof(signature_));
}

// Private

/// Return the base address of the page currently
/// in use for storage (may be full)
uint32_t EepromRaw::getCurrentPageAddr()
{
    // Do no reorder
    if (pageInUse(addr_page0_))
        return addr_page0_;
    else
        return addr_page1_;
}


/// Return the base address of the page that is not the page address provided
inline uint32_t EepromRaw::getOtherPageAddr(uint32_t addr)
{
    return (addr == addr_page0_) ? addr_page1_ : addr_page0_;
}


//// Erase page, write signature, set status to in_use
bool EepromRaw::formatPage(uint32_t base_addr)
{
    //FLASH_EraseInitTypeDef erase_info = { 0 };
    //erase_info.TypeErase = FLASH_TYPEERASE_PAGES;
    //erase_info.Banks = FLASH_BANK_1 // not used
    //erase_info.PageAddress = base_addr;
    //erase_info.NbPages = 1;
    //HAL_FLASH_Unlock();
    Flash::unlock();
    //uint32_t result;
    //HAL_FLASHEx_Erase(&erase_info, &result);
    //bool success = result == 0xFFFFFFFF;
    bool success = Flash::erasePages(base_addr, 1);
    if (success)
        //success = success && HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, base_addr, signature_) == HAL_OK;
        success = success && Flash::program(base_addr, &signature_, 1);
    //HAL_FLASH_Lock();
    Flash::lock();
    return success;
}


//// Change page status to full
void EepromRaw::setPageStatusFull(uint32_t base_addr)
{
    //HAL_FLASH_Unlock();
    Flash::unlock();
    /*HAL_FLASH_Program(
            FLASH_TYPEPROGRAM_HALFWORD,
            base_addr + sizeof(Header::signature_),
            static_cast<uint16_t>(PageState::full));*/
    Flash::program(base_addr + sizeof(Header::signature_), &page_state_full, 1);
    //HAL_FLASH_Lock();
    Flash::lock();
}


/// Return the address of the stored data or 0 if page is empty
/// Address returned is start of the storage block, including leading signature
uint32_t EepromRaw::findStoredBlock(uint32_t page_base_addr)
{
    uint32_t addr = page_base_addr + last_slot_page_offs_;

    while (addr >= (page_base_addr + sizeof(Header))) {
        if (addrToContent(addr) == signature_)
            return addr;
        addr -= storage_len_;
    }
    // Page is empty
    return 0;
}


/// Returns the address of the next free block to store T
/// If the page is full, 0 is returned
uint32_t EepromRaw::findFreeBlock(uint32_t page_base_addr)
{
    uint32_t addr = findStoredBlock(page_base_addr);
    if (addr == 0)
        return page_base_addr + sizeof(Header);
    return (addr == page_base_addr + last_slot_page_offs_)
            ? 0
            : addr + storage_len_;
}


/// Write data with a signature prefix and suffix
bool EepromRaw::writeBlock(uint32_t addr, const uint16_t* data)
{
    // TODO: check return vals

    //HAL_FLASH_Unlock();
    Flash::unlock();

    //HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, signature_);
    Flash::program(addr, &signature_, 1);

    //flashWrite(addr+=2, data, data_len_ >> 1);
    Flash::program(addr+=2, data, data_len_ >> 1);

    //HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr+=data_len_, signature_);
    Flash::program(addr+=data_len_, &signature_, 1);

    //HAL_FLASH_Lock();
    Flash::lock();
    return true;
}
