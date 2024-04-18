#ifndef LIME_FX3MOCK_H
#define LIME_FX3MOCK_H

#include <gmock/gmock.h>

#include "comms/USB/FX3/FX3.h"

namespace lime::testing {

class FX3Mock : public FX3
{
  public:
    MOCK_METHOD(bool, Connect, (uint16_t vid, uint16_t pid, const std::string_view serial), (override));
    MOCK_METHOD(void, Disconnect, (), (override));

    MOCK_METHOD(int32_t, BulkTransfer, (uint8_t endPoint, uint8_t* data, int length, int32_t timeout_ms), (override));
    MOCK_METHOD(int32_t,
        ControlTransfer,
        (int requestType, int request, int value, int index, uint8_t* data, uint32_t length, int32_t timeout_ms),
        (override));

#ifdef __unix__
    MOCK_METHOD(void, HandleLibusbEvents, (), (override));
#endif
};

} // namespace lime::testing

#endif // LIME_FX3MOCK_H
