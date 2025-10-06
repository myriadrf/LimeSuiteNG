#ifndef LIME_COMPLEX_H
#define LIME_COMPLEX_H

#include <cstdint>
#include <complex>
#include <type_traits>

namespace lime {

/** @brief Structure to hold a 16 bit integer complex number. */
using complex16_t = std::complex<int16_t>;
static_assert(std::is_trivially_copyable<complex16_t>::value == true, "complex16_t is not trivially copyable");

/** @brief Structure to hold a 12 bit integer complex number.
    Stored as 16 bit, but the actual used values range should be of 12bits
*/
// Inheriting complex16_t instead of using alias to differentiate types for templates
struct complex12_t : public complex16_t {
    /// @brief Constructs the 16 bit integer complex number.
    /// @param real The I (real) component of the number.
    /// @param imag The Q (imaginary) component of the number.
    constexpr complex12_t(int16_t real, int16_t imag)
        : complex16_t(real, imag){};
};
static_assert(std::is_trivially_copyable<complex12_t>::value == true, "complex12_t is not trivially copyable");

/** @brief Structure to hold a 32 bit float complex number. */
using complex32f_t = std::complex<float>;
static_assert(std::is_trivially_copyable<complex32f_t>::value == true, "complex32f_t is not trivially copyable");

/** @brief Structure to hold a 64 bit float complex number. */
using complex64f_t = std::complex<double>;
static_assert(std::is_trivially_copyable<complex64f_t>::value == true, "complex64f_t is not trivially copyable");

} // namespace lime

#endif
