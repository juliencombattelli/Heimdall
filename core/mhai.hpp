//=============================================================================
// Name        : mhai.hpp
// Author      : Julien Combattelli
// EMail       : julien.combattelli@gmail.com
// Date        : Jan 26, 2019
// Version     : 1.1.0
// Copyright   : This file is part of the Modern Hardware Abstraction Interface
//               (MHAI) project which is released under MIT license.
//               See file LICENSE.txt for full license details.
//=============================================================================

#ifndef MHAI_CORE_HPP_
#define MHAI_CORE_HPP_

#include <cstdint>
#include <type_traits>

namespace mhai
{
using addr_t = std::size_t;

template<addr_t periph_addr>
struct Periph
{
    template<typename Integral, addr_t reg_offset>
    struct Register
    {
        static_assert(std::is_unsigned<Integral>::value,
                      "Integral must be one of std::uint8_t, std::uint16_t, std::uint32_t, std::uint64_t");

        static constexpr addr_t reg_addr = periph_addr + reg_offset;

        template<Integral... bit>
        static constexpr void set_bit() noexcept
        {
            *(volatile Integral*)(reg_addr) |= (bit | ...);
        }

        template<Integral... bit>
        static constexpr void clear_bit() noexcept
        {
            *(volatile Integral*)(reg_addr) &= ~(bit | ...);
        }

        template<Integral... bit>
        static constexpr Integral get_bit() noexcept
        {
            return *(volatile Integral*)(reg_addr) & (bit | ...);
        }

        static constexpr void set(Integral val) noexcept
        {
            *(volatile Integral*)(reg_addr) = val;
        }

        static constexpr void clear() noexcept
        {
            *(volatile Integral*)(reg_addr) = 0x0u;
        }

        static constexpr Integral get() noexcept
        {
            return *(volatile Integral*)(reg_addr);
        }

        static constexpr void modify(Integral clearmask, Integral setmask) noexcept
        {
            set((get() & ~clearmask) | setmask);
        }

    }; // struct Register
}; // struct Periph

} // namespace mhai

#endif // MHAI_CORE_HPP_
