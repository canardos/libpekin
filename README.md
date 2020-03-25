# Libpekin MCU Library

Libpekin is a collection of C/C++ code targeting light-weight embedded applications on 32-bit MCUs, primarily the STM32 platform.

It is the result of attempts to reuse code in my [various embedded projects](https://www.duk.io/blog/electronics-projects/). The different parts of the library have been developed on-demand based on the requirements of those projects. Consequently, functionality is quite scattered with very limited testing. Some areas are reasonably developed, while others have only the minimum code needed to accomplish a specific task for a specific application.

**It almost certainly contains more than a few bugs. Be sure to read, understand, and test any code from here.**

The library consists of a core, platform independent set of helper functions, classes, drivers and "interfaces" (`/libpekin`), and hardware specific implementations (e.g. `/libpekin_stm32`)

Abstractions are a mix of abstract classes (AKA interfaces) and C++20 concepts.

## Build

There are no build files here, as it's mostly a header-only library and everything needs to be statically linked in the end anyway.

The library has predominantly been used in embedded projects built using [PlatformIO](https://platformio.org/). For those projects, putting the `Libpekin` and `Libpekin_[architecture]` folders in the library folder (usually `lib`) should be all that is required.

**Notes:**

- Very limited testing and only with GCC.
- Makes use of C++2a concepts to define interfaces (`-std=c++2a -fconcepts` required).
- C99 VLAs used in several places - make sure you know your stack requirements.

**Dependencies**

- STM32 CMSIS headers for `libpekin_stm32`

## Folder structure

```
libpekin [platform independent]
|
+--audio      : Audio related constants/data structures
+--bus        : Bus and GPIO 'interfaces'
+--display    : Some display drivers
+--drivers    : MCU independent drivers for various semiconductor devices
              : and displays
+--graphics   : Functionality to draw text and graphics
+--pid        : PID algo
+--serial     : Serial I/O
+--touch      : Touch screen
```

```
libpeken_stm32 [STM32 (primarily F1) platform]
|
+--display    : Parallel display buses
+--examples   :
+--flash      : EEPROM emulation
+--serial     : Serial I/O
+--touch      : Touch screen
|
\adc_stm32f1xx.h       : Basic STM32f1xx ADC functions
\clock_stm32f1xx.h     : Basic STM32f1xx clock configuration
                       : functionality
\dac_stm32f1xx.h       : Very limited STM32f1xx DAC functions
\dma_stm32f1xx.h       : Very limited STM32f1xx DMA configuration
                       : functionality
\flash_stm32f1xx.h     : STM32F1xx Flash storage functionality
\flash_stm32f1xx_stm.h : STM32F1xx Flash constants
\fsmc_stm32f1xx.h      : Very limited STM32f1xx FSMC functionality
\pins_stm32f1xx.h      : Low level individual GPIO pin operations
                       : helpers for STM32f1xx MCUs.
\power_stm32f1xx.h     : Some sleep, stop, standby operation
                       : functionality for STM32f1xx MCUs.
\timer_stm32f1xx.h     : Limited STM32F1xx timer configuration
                       : functionality
                       :
\libpekin_stm32_hal.*  : Required libpekin function implementations
                       : for STM32
```
