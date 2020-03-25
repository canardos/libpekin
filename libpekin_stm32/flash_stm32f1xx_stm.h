#ifndef LIB_LIBPEKIN_STM32_FLASH_STM32F1XX_STM_H_
#define LIB_LIBPEKIN_STM32_FLASH_STM32F1XX_STM_H_

/******************************************************************************
 *
 * The below functions and defines are adapted from the macros in file
 * stm32f1xx_hal_flash_ex.h of the STM32 HAL library.
 *
 * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#define FLASH_SIZE_DATA_REGISTER     0x1FFFF7E0U

#if (defined(STM32F101x6) || defined(STM32F102x6) || defined(STM32F103x6) || defined(STM32F100xB) || defined(STM32F101xB) || defined(STM32F102xB) || defined(STM32F103xB))
#define FLASH_PAGE_SIZE          0x400U
#endif /* STM32F101x6 || STM32F102x6 || STM32F103x6 */
       /* STM32F100xB || STM32F101xB || STM32F102xB || STM32F103xB */

#if (defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F103xE) || defined(STM32F101xG) || defined(STM32F103xG) || defined(STM32F105xC) || defined(STM32F107xC))
#define FLASH_PAGE_SIZE          0x800U
#endif /* STM32F100xB || STM32F101xB || STM32F102xB || STM32F103xB */


    static constexpr bool isValidFlashProgAddr(uint32_t addr)
    {
/* Low Density */
#if (defined(STM32F101x6) || defined(STM32F102x6) || defined(STM32F103x6))
        return (((addr) >= FLASH_BASE) && (((*((uint16_t *)FLASH_SIZE_DATA_REGISTER)) == 0x20U) ?
                ((addr) <= FLASH_BANK1_END) :  ((addr) <= 0x08003FFFU)));

#endif /* STM32F101x6 || STM32F102x6 || STM32F103x6 */

/* Medium Density */
#if (defined(STM32F100xB) || defined(STM32F101xB) || defined(STM32F102xB) || defined(STM32F103xB))
        return (((addr) >= FLASH_BASE) && (((*((uint16_t *)FLASH_SIZE_DATA_REGISTER)) == 0x80U) ?
                ((addr) <= FLASH_BANK1_END) :  (((*((uint16_t *)FLASH_SIZE_DATA_REGISTER)) == 0x40U) ?
                ((addr) <= 0x0800FFFF) :  (((*((uint16_t *)FLASH_SIZE_DATA_REGISTER)) == 0x20U) ?
                ((addr) <= 0x08007FFF) :  ((addr) <= 0x08003FFFU)))));

#endif /* STM32F100xB || STM32F101xB || STM32F102xB || STM32F103xB*/

/* High Density */
#if (defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F103xE))
        return (((addr) >= FLASH_BASE) && (((*((uint16_t *)FLASH_SIZE_DATA_REGISTER)) == 0x200U) ?
                ((addr) <= FLASH_BANK1_END) :  (((*((uint16_t *)FLASH_SIZE_DATA_REGISTER)) == 0x180U) ?
                ((addr) <= 0x0805FFFFU) :  ((addr) <= 0x0803FFFFU))));

#endif /* STM32F100xE || STM32F101xE || STM32F103xE */

/* XL Density */
#if defined(FLASH_BANK2_END)
        return  (addr) (((addr) >= FLASH_BASE) && (((*((uint16_t *)FLASH_SIZE_DATA_REGISTER)) == 0x400U) ?
               ((addr) <= FLASH_BANK2_END) :  ((addr) <= 0x080BFFFFU)));

#endif /* FLASH_BANK2_END */

/* Connectivity Line */
#if (defined(STM32F105xC) || defined(STM32F107xC))
        return (((addr) >= FLASH_BASE) && (((*((uint16_t *)FLASH_SIZE_DATA_REGISTER)) == 0x100U) ?
                ((addr) <= FLASH_BANK1_END) :  (((*((uint16_t *)FLASH_SIZE_DATA_REGISTER)) == 0x80U) ?
                ((addr) <= 0x0801FFFFU) :  ((addr) <= 0x0800FFFFU))));

#endif /* STM32F105xC || STM32F107xC */
    }


    static constexpr bool isValidFlashPage(uint32_t addr, uint16_t nb_pages)
    {
/* Low Density */
#if (defined(STM32F101x6) || defined(STM32F102x6) || defined(STM32F103x6))
        return (((*((uint16_t *)FLASH_SIZE_DATA_REGISTER)) == 0x20U) ? ((addr)+((nb_pages)*FLASH_PAGE_SIZE)- 1 <= 0x08007FFFU) :
               ((addr)+((nb_pages)*FLASH_PAGE_SIZE)- 1 <= 0x08003FFFU));

#endif /* STM32F101x6 || STM32F102x6 || STM32F103x6 */

/* Medium Density */
#if (defined(STM32F100xB) || defined(STM32F101xB) || defined(STM32F102xB) || defined(STM32F103xB))
        return (((*((uint16_t *)FLASH_SIZE_DATA_REGISTER)) == 0x80U) ? ((addr)+((nb_pages)*FLASH_PAGE_SIZE)-1 <= 0x0801FFFFU) :
               (((*((uint16_t *)FLASH_SIZE_DATA_REGISTER)) == 0x40U) ? ((addr)+((nb_pages)*FLASH_PAGE_SIZE)-1 <= 0x0800FFFFU) :
               (((*((uint16_t *)FLASH_SIZE_DATA_REGISTER)) == 0x20U) ? ((addr)+((nb_pages)*FLASH_PAGE_SIZE)-1 <= 0x08007FFFU) :
               ((addr)+((nb_pages)*FLASH_PAGE_SIZE)-1 <= 0x08003FFFU))));

#endif /* STM32F100xB || STM32F101xB || STM32F102xB || STM32F103xB*/

/* High Density */
#if (defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F103xE))
        return (((*((uint16_t *)FLASH_SIZE_DATA_REGISTER)) == 0x200U) ? ((addr)+((nb_pages)*FLASH_PAGE_SIZE)-1 <= 0x0807FFFFU) :
               (((*((uint16_t *)FLASH_SIZE_DATA_REGISTER)) == 0x180U) ? ((addr)+((nb_pages)*FLASH_PAGE_SIZE)-1 <= 0x0805FFFFU) :
               ((addr)+((nb_pages)*FLASH_PAGE_SIZE)-1 <= 0x0803FFFFU)));

#endif /* STM32F100xE || STM32F101xE || STM32F103xE */

/* XL Density */
#if defined(FLASH_BANK2_END)
        return (((*((uint16_t *)FLASH_SIZE_DATA_REGISTER)) == 0x400U) ? ((addr)+((nb_pages)*FLASH_PAGE_SIZE)-1 <= 0x080FFFFFU) :
               ((addr)+((nb_pages)*FLASH_PAGE_SIZE)-1 <= 0x080BFFFFU));

#endif /* FLASH_BANK2_END */

/* Connectivity Line */
#if (defined(STM32F105xC) || defined(STM32F107xC))
       return (((*((uint16_t *)FLASH_SIZE_DATA_REGISTER)) == 0x100U) ? ((addr)+((nb_pages)*FLASH_PAGE_SIZE)-1 <= 0x0803FFFFU) :
              (((*((uint16_t *)FLASH_SIZE_DATA_REGISTER)) ==  0x80U) ? ((addr)+((nb_pages)*FLASH_PAGE_SIZE)-1 <= 0x0801FFFFU) :
              ((addr)+((nb_pages)*FLASH_PAGE_SIZE)-1 <= 0x0800FFFFU)));

#endif /* STM32F105xC || STM32F107xC */
    }

#endif /* LIB_LIBPEKIN_STM32_FLASH_STM32F1XX_STM_H_ */
