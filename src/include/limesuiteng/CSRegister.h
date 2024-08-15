#ifndef LIME_CSREGISTER_H
#define LIME_CSREGISTER_H

#include "limesuiteng/Register.h"

namespace lime {

/// @brief Generic control status register that has default value and specifies binary number format
struct CSRegister : public lime::Register {
    constexpr CSRegister()
        : lime::Register(0, 15, 0)
        , defaultValue(0)
        , twoComplement(false){};

    /// @copydoc lime::Register::Register(uint16_t,uint8_t,uint8_t)
    /// @param defaultValue The default value of the register.
    /// @param twoComplement Whether the register is represented in a Two's Complement way.
    constexpr CSRegister(uint16_t address, uint8_t msb, uint8_t lsb, uint16_t defaultValue, bool twoComplement)
        : lime::Register(address, msb, lsb)
        , defaultValue(defaultValue)
        , twoComplement(twoComplement){};

    uint16_t defaultValue; ///< The default value of the register.
    bool twoComplement; ///< Indicates if the register is represented in a Two's Complement way.
};

} // namespace lime
#endif // LIME_CSREGISTER_H
