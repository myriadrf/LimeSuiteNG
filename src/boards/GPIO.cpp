#include "GPIO.h"

#include "limesuiteng/SDRDevice.h"

#include <assert.h>
#include <stdint.h>
#include <string>

namespace lime {

GPIO_Interface::GPIO_Interface(lime::SDRDevice* parent, const std::vector<GPIO>& pins)
    : parent(parent)
    , pins(pins)
{
    uint32_t maxIndex = 0;
    for (const auto& pin : pins)
    {
        if (pin.id > maxIndex)
            maxIndex = pin.id;
    }
    values.resize(maxIndex / 8 + (maxIndex % 8 ? 1 : 0));
}

void GPIO_Interface::SetValue(uint32_t id, uint32_t value)
{
    uint8_t mask = 1 << (id % 8);
    values[id / 8] &= ~mask;
    values[id / 8] |= (value != 0 ? mask : 0);
}

uint32_t GPIO_Interface::GetValue(uint32_t id)
{
    return (values[id / 8] >> (id % 8)) & 1;
}

std::vector<GPIO> GPIO_Interface::GetPins() const
{
    return pins;
}

OpStatus GPIO_Interface::WriteAll()
{
    assert(parent);
    return parent->GPIOWrite(values.data(), values.size());
}

OpStatus GPIO_Interface::ReadAll()
{
    assert(parent);
    return parent->GPIORead(values.data(), values.size());
}

} // namespace lime
