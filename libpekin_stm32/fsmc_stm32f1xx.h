#ifndef LIB_LIBPEKIN_STM32_FSMC_STM32F1XX_H_
#define LIB_LIBPEKIN_STM32_FSMC_STM32F1XX_H_

#include <cstdint>
#include "bits.h"
#include "libpekin.h"

namespace LibpStm32::Fsmc {


// AN2784 - Using the high-density STM32F10xxx FSMCperipheral to drive external memories

/*
 * AHB clock cycle @ 72MHz = 13.88ns
 */
/// FSMC Access Mode
/*enum class AccessMode : uint32_t {
    // (EXTMOD: bit is set in  the FSMC_BCRx register
    // Mode 1 is the default mode when SRAM/CRAM memory type is selected ( Bits 3:2 MTYP = 0x0 or 0x01 in the FSMC_BCRx register)
    // Mode 2 is the default mode when NOR memory type is selected ( Bits 3:2 MTYP = 0x10 in the FSMC_BCRx register)
    mode_1, // SRAM/CRAM
    mode_2, // NOR
    extended_a = 0x00000000U, // SRAM/CRAM
    extended_b = static_cast<uint32_t>(FSMC_BTRx_ACCMOD_0), // NOR
    extended_c = static_cast<uint32_t>(FSMC_BTRx_ACCMOD_1), // NOR
    extended_d = static_cast<uint32_t>(FSMC_BTRx_ACCMOD_0 | FSMC_BTRx_ACCMOD_1) // NOR

};*/

/*
         (++) Static random access memory (SRAM).
         (++) NOR Flash memory.
         (++) PSRAM (4 memory banks).
         (++) 16-bit PC Card compatible devices.
         (++) Two banks of NAND Flash memory with ECC hardware to check up to 8 Kbytes of
              data.



Bank1 NOR/PSRAM
Bank3 NAND
*/

/**
 * Disconnect FSMC NADV output signal - use pin as GPIO/other function.
 */
inline __attribute__((always_inline))
void nadvDisconnected()
{
    Libp::Bits::setBit(AFIO->MAPR2, AFIO_MAPR2_FSMC_NADV_REMAP_Pos);
}

/**
 * Connect FSMC NADV output signal for usage.
 */
inline __attribute__((always_inline))
void nadvConnected()
{
    Libp::Bits::clearBit(AFIO->MAPR2, AFIO_MAPR2_FSMC_NADV_REMAP_Pos);
}


struct SramNorCfg {
    uint32_t bcr;
    uint32_t btr;
};

struct Config {

    enum class MemType {
        sram  = 0b00 << FSMC_BCRx_MTYP_Pos,
        psram = 0b01 << FSMC_BCRx_MTYP_Pos,
        nor   = 0b11 << FSMC_BCRx_MTYP_Pos
    };
    enum class SyncWaitCfg : uint32_t {
        one_cycle_before = 0,                 // NWAIT signal is active one data cycle before wait state
        during_wait_state = FSMC_BCRx_WAITCFG // NWAIT signal is active during wait state (not used for PRAM).
    };
    enum class CramPageSize : uint32_t {
        no_burst_split = 0b000 << 16,
        bytes_128      = 0b001 << 16,
        bytes_256      = 0b010 << 16,
        bytes_512      = 0b011 << 16,
        bytes_1024     = 0b100 << 16,
    };
    enum class WaitPolarity : uint32_t {
        active_low = 0,
        active_high = FSMC_BCRx_WAITPOL
    };
    enum class DataWidth : uint32_t {
        bits_8 = 0,
        bits_16 = 0b01 << FSMC_BCRx_MWID_Pos
    };


    // Descending register bit position order
    bool cram_burst_en          = false; // enable sync burst protocol for write ops for PSRAM
    CramPageSize cram_pg_size   = CramPageSize::no_burst_split;
    bool async_wait_en          = false;
    bool ext_mode_en            = false;
    bool wait_en                = true; /// sync
    bool write_en               = true;
    SyncWaitCfg sync_wait_cfg   = SyncWaitCfg::one_cycle_before;
    bool direct_wrap_burst_en   = false;
    WaitPolarity wait_polarity  = WaitPolarity::active_low;
    bool burst_en               = false;
    bool flash_access_en        = true;
    DataWidth data_width         = DataWidth::bits_16;
    MemType mem_type            = MemType::nor;
    bool addr_data_mux          = true;

    enum class AccessMode : uint32_t {
        mode_a = 0b00 << FSMC_BTRx_ACCMOD_Pos,
        mode_b = 0b01 << FSMC_BTRx_ACCMOD_Pos,
        mode_c = 0b10 << FSMC_BTRx_ACCMOD_Pos,
        mode_d = 0b11 << FSMC_BTRx_ACCMOD_Pos
    };

    /// All values should be actual cycles.
    /// Do not adjust to values expected by registers
    struct Timing {
        AccessMode access_mode = AccessMode::mode_a;
        // must be 2 for PSRAM(CRAM)
        uint8_t  data_latency   = 0b1111;      ///< [2..17 FSMC_CLK cycles] For synchronous NOR flash with burst mode enabled
        uint8_t  clk_div        = 0b1111;      ///< [2..16 HCLK cycles] Synchronous memory only.
        uint8_t  bus_turn       = 0b1111;      ///< [1..16 HCLK cycles]
        uint16_t data_phase_dur = 0b1111'1111; ///< [2..256]
        uint8_t  addr_hold      = 0b1111;      ///< [2..16 HCLK cycles]
        uint8_t  addr_setup     = 0b1111;      ///< [1..16 HCLK cycles]
    };

    Timing timing;


    constexpr SramNorCfg makeConfig() const
    {
        uint32_t bcr =
                  cram_burst_en                 << FSMC_BCRx_CBURSTRW_Pos
                | addr_data_mux                 << FSMC_BCRx_MUXEN_Pos
                | Libp::enumBaseT(mem_type)
                | Libp::enumBaseT(data_width)
                | flash_access_en               << FSMC_BCRx_FACCEN_Pos
                | burst_en                      << FSMC_BCRx_BURSTEN_Pos
                | Libp::enumBaseT(wait_polarity)
                | direct_wrap_burst_en          << FSMC_BCRx_WRAPMOD_Pos
                | Libp::enumBaseT(sync_wait_cfg)
                | write_en                      << FSMC_BCRx_WREN_Pos
                | wait_en                       << FSMC_BCRx_WAITEN_Pos
                | ext_mode_en                   << FSMC_BCRx_EXTMOD_Pos
                | async_wait_en                 << FSMC_BCRx_ASYNCWAIT_Pos
                | Libp::enumBaseT(cram_pg_size)
                | cram_burst_en                 << FSMC_BCRx_CBURSTRW_Pos;

        uint32_t btr =
                  ( (timing.data_latency - 2) & 0b1111) << FSMC_BTRx_DATLAT_Pos
                | ( (timing.clk_div - 1)      & 0b1111) << FSMC_BTRx_CLKDIV_Pos
                | ( (timing.bus_turn - 1)     & 0b1111) << FSMC_BTRx_BUSTURN_Pos
                | (timing.data_phase_dur - 1)           << FSMC_BTRx_DATAST_Pos
                | ( (timing.addr_hold - 1) & 0b1111)    << FSMC_BTRx_ADDHLD_Pos
                | ( (timing.addr_setup - 1) & 0b1111)   << FSMC_BTRx_ADDSET_Pos;

        SramNorCfg config = { bcr, btr };
        return config;
    }

};

/**
 * Very basic configuration helper for FSMC SRAM.
 *
 * Does not support extended mode.
 *
 * @tparam bank_ 1..4
 */
template <uint8_t bank_>
class NorSram {
private:
    //FSMC_Bank1
    static inline FSMC_Bank1_TypeDef* const fsmc_ = reinterpret_cast<FSMC_Bank1_TypeDef*>(FSMC_BANK1_R_BASE);
    static inline volatile uint32_t& bcr_ = fsmc_->BTCR[(bank_ - 1)*2];
    static inline volatile uint32_t& btr_ = fsmc_->BTCR[(bank_ - 1)*2 + 1];

public:

    inline __attribute__((always_inline))
    static void disable()
    {
        Libp::Bits::clearMask(bcr_, FSMC_BCRx_MBKEN);
    }

    inline __attribute__((always_inline))
    static void enable()
    {
        Libp::Bits::setMask(bcr_, FSMC_BCRx_MBKEN);
    }

    /// FSMC clock must be enabled prior to calling this function
    static void init(const SramNorCfg cfg)
    {
        disable();
        constexpr uint32_t mask = ~(1 << 7); // bit 7 must be preserved
        Libp::Bits::setBits(bcr_, mask, cfg.bcr);
        btr_ = cfg.btr;
        // TODO: extended timing regs
        enable();
    }
};

// do transfer via:
//   (++) HAL_SRAM_Read()/HAL_SRAM_Write() for polling read/write access
//   (++) HAL_SRAM_Read_DMA()/HAL_SRAM_Write_DMA() for DMA read/write transfer
// do control via:
//   HAL_SRAM_WriteOperation_Enable()/
//   HAL_SRAM_WriteOperation_Disable() to respectively enable/disable the SRAM write operation
// monitor via:
//   HAL_SRAM_GetState

// When the extended mode is enabled, the FSMC_BTR register is used for read operations and the FSMC_BWR register is used for write operations.


} // namespace LibpStm32::Fsmc

#endif /* LIB_LIBPEKIN_STM32_FSMC_STM32F1XX_H_ */
