#ifndef LMS8001_REGISTERS_MAP_H
#define LMS8001_REGISTERS_MAP_H

#include <vector>
#include <map>
#include <stdint.h>

namespace lime {
struct LMS8Parameter;

class LMS8001_RegistersMap
{
  public:
    struct Register {
        uint16_t value;
        uint16_t defaultValue;
        uint16_t mask;
    };

    LMS8001_RegistersMap();
    ~LMS8001_RegistersMap();

    uint16_t GetValue(uint16_t address) const;
    void SetValue(const uint16_t address, const uint16_t value);

    void InitializeDefaultValues(const std::vector<const LMS8Parameter*> parameterList);
    uint16_t GetDefaultValue(uint16_t address) const;
    std::vector<uint16_t> GetUsedAddresses() const;

  protected:
    std::map<uint16_t, Register> mRegMap;
};

} // namespace lime
#endif
