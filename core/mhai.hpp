//=============================================================================
// Name        : mhai.hpp
// Author      : Julien Combattelli
// EMail       : julien.combattelli@gmail.com
// Date        : Jan 26, 2019
// Version     : 1.0.0
// Copyright   : This file is part of the Modern Hardware Abstraction Interface
//               (MHAI) project which is released under MIT license.
//               See file LICENSE.txt for full license details.
//=============================================================================

#ifndef MHAI_CORE_HPP_
#define MHAI_CORE_HPP_

#include <cstdint>

namespace mhai
{

template<std::uint32_t periph_addr>
struct Periph
{
    template<std::uint32_t reg_offset>
    struct Register
    {
        static constexpr std::uint32_t reg_addr = periph_addr + reg_offset;

        template<std::uint32_t ... bit>
        static constexpr void set_bit() noexcept
        {
            *(volatile std::uint32_t*)(reg_addr) |= (bit | ...);
        }

        template<std::uint32_t ... bit>
        static constexpr void clear_bit() noexcept
        {
            *(volatile std::uint32_t*)(reg_addr) &= ~(bit | ...);
        }

        template<std::uint32_t ... bit>
        static constexpr uint32_t get_bit() noexcept
        {
            return *(volatile std::uint32_t*)(reg_addr) & (bit | ...);
        }

        static constexpr void set(std::uint32_t val) noexcept
        {
            *(volatile std::uint32_t*) (reg_addr) = val;
        }

        static constexpr void clear() noexcept
        {
            *(volatile std::uint32_t*)(reg_addr) = 0x0u;
        }

        static constexpr std::uint32_t get() noexcept
        {
            return *(volatile std::uint32_t*) (reg_addr);
        }

        static constexpr void modify(std::uint32_t clearmask, std::uint32_t setmask) noexcept
        {
            set((get() & ~clearmask) | setmask);
        }

    }; // struct Register
}; // struct Periph

} // namespace mhai

#endif // MHAI_CORE_HPP_
