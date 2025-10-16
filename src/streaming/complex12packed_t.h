#ifndef LIME_COMPLEX12PACKED_H
#define LIME_COMPLEX12PACKED_H

#include <cstdint>
#include <type_traits>

namespace lime {

/** @brief Structure to hold a 12 bit packed integer complex number.
    Used only for data transferring to hardware.
*/
struct complex12packed_t {
    constexpr complex12packed_t()
        : complex12packed_t(0, 0)
    {
    }

    /// @brief Constructs the 12 bit compressed complex number.
    /// @param i The I value of the number.
    /// @param q The Q value of the number.
    constexpr complex12packed_t(int16_t i, int16_t q)
        : data{ uint8_t(i), uint8_t((q << 4) | ((i >> 8) & 0x0F)), uint8_t(q >> 4) }
    {
    }

    /// Set the I (real) value of the complex number.
    constexpr int16_t real() const
    {
        int16_t value = data[0];
        value |= (data[1] << 8);
        // shifting to fill sign
        value <<= 4;
        value >>= 4;
        return value;
    }

    /// @copydoc POD_complex_t::real(T)
    constexpr void real(int16_t value)
    {
        data[0] = value;
        data[1] = (data[1] & 0xF0) | ((value >> 8) & 0x0F);
    }

    /// Return the Q (imaginary) value of the complex number.
    constexpr int16_t imag() const
    {
        int16_t value = data[1];
        value |= (data[2] << 8);
        value >>= 4;
        return value;
    }

    /// Set the Q (imaginary) value of the complex number.
    constexpr void imag(int16_t value)
    {
        data[1] = (value << 4) | (data[1] & 0x0F);
        data[2] = value >> 4;
    }

  private:
    uint8_t data[3];
};
static_assert(std::is_trivially_copyable<complex12packed_t>::value == true, "complex12packed_t is not trivially copyable");

} // namespace lime

#endif