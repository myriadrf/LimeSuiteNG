#ifndef LIME_GPIO_H
#define LIME_GPIO_H

#include <assert.h>
#include <stdint.h>
#include <string>
#include <vector>

#include "limesuiteng/config.h"
#include "limesuiteng/OpStatus.h"

namespace lime {

class SDRDevice;

class LIME_API GPIO
{
  public:
    GPIO(uint32_t id, const std::string& name)
        : id(id)
        , name(name)
    {
    }
    uint32_t id;
    std::string name;
};

class LIME_API GPIO_Interface
{
  public:
    GPIO_Interface(lime::SDRDevice* parent, const std::vector<GPIO>& pins);
    void SetValue(uint32_t id, uint32_t value);
    uint32_t GetValue(uint32_t id);
    std::vector<GPIO> GetPins() const;
    OpStatus WriteAll();
    OpStatus ReadAll();

  private:
    lime::SDRDevice* parent;
    std::vector<GPIO> pins;
    std::vector<uint8_t> values;
};

} // namespace lime

#endif // LIME_GPIO_H
