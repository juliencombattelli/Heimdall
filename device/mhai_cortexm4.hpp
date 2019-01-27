//=============================================================================
// Name        : mhai_cortexm4.hpp
// Author      : Julien Combattelli
// EMail       : julien.combattelli@gmail.com
// Date        : Jan 27, 2019
// Version     : 1.0.0
// Copyright   : This file is part of the Modern Hardware Abstraction Interface
//               (MHAI) project which is released under MIT license.
//               See file LICENSE.txt for full license details.
//=============================================================================

#ifndef MHAI_DEVICE_MHAI_CM4_HPP_
#define MHAI_DEVICE_MHAI_CM4_HPP_

#include "mhai.hpp"

#include <cstdint>

#if defined(__CC_ARM)
#    define __ASM __asm       /*!< asm keyword for ARM Compiler */
#    define __INLINE __inline /*!< inline keyword for ARM Compiler */
#    define __STATIC_INLINE static __inline

#elif defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
#    define __ASM __asm       /*!< asm keyword for ARM Compiler */
#    define __INLINE __inline /*!< inline keyword for ARM Compiler */
#    define __STATIC_INLINE static __inline

#elif defined(__GNUC__)
#    define __ASM __asm     /*!< asm keyword for GNU Compiler */
#    define __INLINE inline /*!< inline keyword for GNU Compiler */
#    define __STATIC_INLINE static inline

#elif defined(__ICCARM__)
#    define __ASM __asm     /*!< asm keyword for IAR Compiler */
#    define __INLINE inline /*!< inline keyword for IAR Compiler. Only available in High optimization mode! */
#    define __STATIC_INLINE static inline

#elif defined(__TMS470__)
#    define __ASM __asm /*!< asm keyword for TI CCS Compiler */
#    define __STATIC_INLINE static inline

#elif defined(__TASKING__)
#    define __ASM __asm     /*!< asm keyword for TASKING Compiler */
#    define __INLINE inline /*!< inline keyword for TASKING Compiler */
#    define __STATIC_INLINE static inline

#elif defined(__CSMC__)
#    define __packed
#    define __ASM _asm      /*!< asm keyword for COSMIC Compiler */
#    define __INLINE inline /*!< inline keyword for COSMIC Compiler. Use -pc99 on compile line */
#    define __STATIC_INLINE static inline

#else
#    error Unknown compiler
#endif

// TODO add FPU defines

///////////////////////////////////////////////////////////////////////////////
// IO definitions (access restrictions to peripheral registers)
///////////////////////////////////////////////////////////////////////////////

#define __I volatile  // Defines 'read only' permissions
#define __O volatile  // Defines 'write only' permissions
#define __IO volatile // Defines 'read / write' permissions

// following defines should be used for structure members
#define __IM volatile const // Defines 'read only' structure member permissions
#define __OM volatile       // Defines 'write only' structure member permissions
#define __IOM volatile      // Defines 'read / write' structure member permissions

namespace mhai
{
namespace cortexm4
{
constexpr std::uint32_t CortexM = 0x4; // Cortex-M Core

///////////////////////////////////////////////////////////////////////////////
// Base addresses for memory mapping
///////////////////////////////////////////////////////////////////////////////

constexpr std::uint32_t SCS_base = 0xE000E000;
constexpr std::uint32_t ITM_base = 0xE0000000;
constexpr std::uint32_t DWT_base = 0xE0001000;
constexpr std::uint32_t TPI_base = 0xE0040000;
constexpr std::uint32_t CoreDebug_base = 0xE000EDF0;
constexpr std::uint32_t SysTick_base = SCS_base + 0x0010;
constexpr std::uint32_t NVIC_base = SCS_base + 0x0100;
constexpr std::uint32_t SCB_base = SCS_base + 0x0D00;

///////////////////////////////////////////////////////////////////////////////
// Peripherals registers offset
///////////////////////////////////////////////////////////////////////////////

// TODO add SysTick registers offset

// TODO add NVIC registers offset

// TODO add DWT registers offset

///////////////////////////////////////////////////////////////////////////////
// Peripherals, registers and operations definitions
///////////////////////////////////////////////////////////////////////////////

// TODO add SysTick periph def

// TODO add NVIC periph def

// TODO add DWT periph def

} // namespace cortexm4
} // namespace mhai

#endif // MHAI_DEVICE_MHAI_CM4_HPP_
