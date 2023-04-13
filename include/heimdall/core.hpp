#ifndef HEIMDALL_CORE_HPP_
#define HEIMDALL_CORE_HPP_

#include <concepts>
#include <cstdint>

namespace mdal {

using address_type = std::uintptr_t;
using index_type = std::size_t;

template <typename T>
concept readable = requires {
    true;
};

template <typename T>
concept writeable = requires {
    true;
};

template <typename T>
concept readable_writeable = requires {
    true;
};

template <typename T>
concept register_traits = requires {
    // Type T::value_type exists and it is an integral type
    requires std::integral<typename T::value_type>;
    // Type T::access_policy exists
    typename T::access_policy;
    // Value T::endianness exists and is of type std::hardware::hw_base::byte_order
    {
        T::endianness
    } -> std::convertible_to<std::hardware::hw_base::byte_order>;
    // Value T::device_bus_width exists and is of type std::hardware::hw_base::device_bus
    {
        T::device_bus_width
    } -> std::convertible_to<std::hardware::hw_base::device_bus>;
    // Value T::address_offset exists and is of type std::hardware::hw_base::address_type
    // @note Use 0 if the offset has no real meaning and the register address is specified only
    // using register_access<>.
    {
        T::address_offset
    } -> std::convertible_to<address_type>;
};

} // namespace mdal

#endif // HEIMDALL_CORE_HPP_
