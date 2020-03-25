#ifndef LIB_LIBPEKIN_STM32_DMA_STM32F1XX_H_
#define LIB_LIBPEKIN_STM32_DMA_STM32F1XX_H_

#include <cstdint>
#include "libpekin.h"
#include "bits.h"

#ifndef DMA1_BASE
static_assert(false, "STM32 CMSIS header must be included before this file");
#endif

namespace LibpStm32::Dma {

/*
 * The two DMA controllers have 12 channels in total (7 for DMA1 and 5 for DMA2
 *
 * In summary, each DMA transfer consists of three operations:
 *
 * - The loading of data from the peripheral data register or a location in
 *   memory addressed through an internal current peripheral/memory address
 *   register.
 *   The start address used for the first transfer is the base peripheral/
 *   memory address programmed in the DMA_CPARx or DMA_CMARx register.
 *
 * - The storage of the data loaded to the peripheral data register or a
 *   location in memory addressed through an internal current peripheral/
 *   memory address register.
 *   The start address used for the first transfer is the base peripheral/
 *   memory address programmed in the DMA_CPARx or DMA_CMARx register.
 *
 * - The post-decrementing of the DMA_CNDTRx register, which contains the
 *   number of transactions that have still to be performed.
 *
 * Each channel can handle DMA transfer between a peripheral register located
 * at a fixed address and a memory address. The amount of data to be
 * transferred (up to 65535) is programmable. The register which contains the
 * amount of data items to be transferred is decremented after each transaction
 */

/**
 * Designed to produce a compile-time constant for use as a configuration
 * value.
 *
 * Defaults:
 * - mem/periph size = 8-bit
 * - priority = low
 * - circular mode = disabled
 * - interrupts = disabled
 *
 * \code
 * Usage:
 *
 * using namespace LibpStm32::Dma
 *
 * constexpr uint32_t dma_cfg = CfgBuilder::create(Mode::periph_to_mem, IncMode::mem_only)
 *     .priority(ChPriority::high)
 *     .bitWidth(MemSize::bits16, PeriphSize::bits16)
 *     .enableInts(true, false, true)
 *     .build();
 *
 * DmaDevice<1>::configure(dma_cfg);
 *
 * \endcode
 */
class CfgBuilder {
private:
    uint32_t ccr_reg; // implicit const
    constexpr CfgBuilder() : ccr_reg(0) { }
    constexpr CfgBuilder(uint32_t reg) : ccr_reg(reg) { }
public:
    enum class ChPriority : uint32_t {
        low       = 0b00,
        medium    = 0b01 << DMA_CCR_PL_Pos,
        high      = 0b10 << DMA_CCR_PL_Pos,
        very_high = 0b11 << DMA_CCR_PL_Pos
    };
    enum class MemSize : uint32_t {
        bits8  = 0b00,
        bits16 = 0b01 << DMA_CCR_MSIZE_Pos,
        bits32 = 0b10 << DMA_CCR_MSIZE_Pos
    };
    enum class PeriphSize : uint32_t {
        bits8  = 0b00,
        bits16 = 0b01 << DMA_CCR_PSIZE_Pos,
        bits32 = 0b10 << DMA_CCR_PSIZE_Pos
    };
    enum class IncMode : uint32_t {
        none        = 0b00,
        periph_only = 0b01 << DMA_CCR_PINC_Pos,
        mem_only    = 0b10 << DMA_CCR_PINC_Pos,
        both        = 0b11 << DMA_CCR_PINC_Pos
    };
    enum class Mode : uint32_t {
        periph_to_mem = 0,
        mem_to_periph = 0,
        mem_to_mem = DMA_CCR_MEM2MEM
    };

    static constexpr CfgBuilder create(Mode mode, IncMode inc_mode = IncMode::none)
    {
        uint32_t reg = Libp::enumBaseT(inc_mode);
        if (mode == Mode::mem_to_mem)
            reg |= DMA_CCR_MEM2MEM;
        else if (mode == Mode::mem_to_periph)
            reg |= DMA_CCR_DIR;

        return CfgBuilder(reg);
    }
    /// Default is low
    constexpr CfgBuilder priority(ChPriority priority) const
    {
        uint32_t reg = ccr_reg;
        Libp::Bits::setBits(reg, DMA_CCR_PL_Msk, Libp::enumBaseT(priority));
        return CfgBuilder(reg);
    }
    /// Default is 8-bit
    constexpr CfgBuilder bitWidth(MemSize mem_size, PeriphSize periph_size = PeriphSize::bits8) const
    {
        uint32_t reg = ccr_reg;
        Libp::Bits::setBits(reg,
                DMA_CCR_MSIZE_Msk | DMA_CCR_PSIZE_Msk,
                Libp::enumBaseT(mem_size) | Libp::enumBaseT(periph_size));
        return CfgBuilder(reg);
    }
    /// Default is disabled
    constexpr CfgBuilder circularMode(bool enable) const
    {
        uint32_t reg = ccr_reg;
        Libp::Bits::updBit(reg, DMA_CCR_CIRC_Pos, enable);
        return CfgBuilder(reg);
    }
    /// Default is all disabled
    constexpr CfgBuilder enableInts(bool error, bool half_transfer, bool transfer_complete) const
    {
        uint32_t reg = ccr_reg;
        Libp::Bits::setBits(reg,
                DMA_CCR_TEIE_Msk | DMA_CCR_HTIE_Msk | DMA_CCR_TCIE_Msk,
                (error << DMA_CCR_TEIE_Pos) | (half_transfer << DMA_CCR_HTIE_Pos) | (transfer_complete << DMA_CCR_TCIE_Pos) );
        return CfgBuilder(reg);
    }
    constexpr uint32_t build()
    {
        return ccr_reg;
    }
};


template <uint8_t channel_, uint32_t base_addr_ = DMA1_BASE>
class DmaDevice {
private:
    static inline DMA_TypeDef* const dma_ = reinterpret_cast<DMA_TypeDef*>(base_addr_);
    static inline DMA_Channel_TypeDef* const ch_
        = reinterpret_cast<DMA_Channel_TypeDef*>(base_addr_ + 0x08 + 20 * (channel_ - 1));
public:

    inline __attribute__((always_inline))
    static bool intGlobal()
    {
        return dma_->ISR & ((1 << 0) << ((channel_ - 1) * 4));
    }
    inline __attribute__((always_inline))
    static bool intComplete()
    {
        return dma_->ISR & ((1 << 1) << ((channel_ - 1) * 4));
    }
    inline __attribute__((always_inline))
    static bool intHalfComplete()
    {
        return dma_->ISR & ((1 << 2) << ((channel_ - 1) * 4));
    }
    inline __attribute__((always_inline))
    static bool intErr()
    {
        return dma_->ISR & ((1 << 3) << ((channel_ - 1) * 4));
    }
    inline __attribute__((always_inline))
    static void clearIntGlobal()
    {
        dma_->IFCR |= ((1 << 0) << ((channel_ - 1) * 4));
    }
    inline __attribute__((always_inline))
    static void clearIntComplete()
    {
        dma_->IFCR |= ((1 << 1) << ((channel_ - 1) * 4));
    }
    inline __attribute__((always_inline))
    static void clearIntHalfComplete()
    {
        dma_->IFCR |= ((1 << 2) << ((channel_ - 1) * 4));
    }
    inline __attribute__((always_inline))
    static void clearIntErr()
    {
        dma_->IFCR |= ((1 << 3) << ((channel_ - 1) * 4));
    }

    /**
     * Does not disable peripheral clock
     */
    inline __attribute__((always_inline))
    static void disable()
    {
        ch_->CCR &= ~DMA_CCR_EN;
    }

    /**
     * Assumes peripheral clock is already enabled
     */
    inline __attribute__((always_inline))
    static void enable()
    {
        ch_->CCR |= DMA_CCR_EN;
    }

    /**
     * Disable the channel/interrupts and clear any pending interrupts
     */
    static void abort()
    {
        // Disable DMA IT
        ch_->CCR &= ~(DMA_CCR_TCIE | DMA_CCR_HTIE | DMA_CCR_TEIE);
        disable();
        // Clear int flags (global flag clears all others)
        dma_->IFCR = (DMA_ISR_GIF1 << ((channel_ - 1) * 4) ); // bits 0, 4, 8 etc for ch1,2,3
    }

    /**
     *
     * @param cfg DMA_CCRx register value - use CfgBuilder to generate.
     */
    static void configure(uint32_t cfg)
    {
        // Can't update regs when enabled
        disable();
        ch_->CCR = cfg;
    }

    /**
     * En/disable transfer status interrupts.
     *
     * @param error
     * @param half_transfer
     * @param transfer_complete
     */
    static void enableInts(bool error, bool half_transfer, bool transfer_complete)
    {
        constexpr uint32_t mask = DMA_CCR_TEIE | DMA_CCR_HTIE | DMA_CCR_TCIE;
        Libp::Bits::setBits(ch_->CCR, mask,
                  (error << DMA_CCR_TEIE_Pos)
                | (half_transfer << DMA_CCR_HTIE_Pos)
                | (transfer_complete << DMA_CCR_TCIE_Pos) );
    }

    /**
     *
     * Transfer will start immediately in Mem2Mem mode. In normal mode, the
     * peripheral connected to the channel must trigger the transfer.
     *
     * @param periph_addr
     * @param mem_addr
     * @param len
     */
    static void start(uint32_t periph_addr, uint32_t mem_addr, uint16_t len)
    {
        disable();
        // Set peripheral address
        ch_->CPAR = periph_addr;
        // Set memory address
        ch_->CMAR = mem_addr;
        ch_->CNDTR = len;
        enable();
    }
};

} // namespace LibpStm32::Dma

#endif /* LIB_LIBPEKIN_STM32_DMA_STM32F1XX_H_ */
