#include "WriteRegistersBatch.h"

#ifndef NDEBUG
    #define ASSERT_WARNING(cond, message) \
        if (!cond) \
        lime::warning("%s (%s)\n", message, #cond)
#else
    #define ASSERT_WARNING(cond, message)
#endif

using namespace lime;

/// @brief Constructor for the batch.
/// @param fpga The FPGA this batch belongs to.
WriteRegistersBatch::WriteRegistersBatch(FPGA* fpga)
    : owner(fpga){};

WriteRegistersBatch::~WriteRegistersBatch()
{
    ASSERT_WARNING(addrs.size() == 0, "FPGA WriteRegistersBatch not flushed");
}

/// @brief Writes the modified values into the FPGA.
/// @return The operation status.
OpStatus WriteRegistersBatch::Flush()
{
    OpStatus status = owner->WriteRegisters(addrs.data(), values.data(), addrs.size());
    addrs.clear();
    values.clear();
    return status;
}

/// @brief Sets an address value pair to write into the FPGA on flushing.
/// @param addr The address to write to.
/// @param value The value to write.
void WriteRegistersBatch::WriteRegister(uint16_t addr, uint16_t value)
{
    addrs.push_back(addr);
    values.push_back(value);
}
