#include "PPSDO_FPGA.h"

#include "comms/ICSR.h"
#include "limesuiteng/Logger.h"

#include <cinttypes>
#include <cmath>
#include <limits>

namespace lime {

namespace {

// Register offsets from the PPSDO block base address.
enum Register : uint32_t {
    CONTROL = 0x00,
    CONFIG_ONE_S_TARGET = 0x04,
    CONFIG_ONE_S_TOL = 0x08,
    CONFIG_TEN_S_TARGET = 0x0C,
    CONFIG_TEN_S_TOL = 0x10,
    CONFIG_HUNDRED_S_TARGET = 0x14,
    CONFIG_HUNDRED_S_TOL = 0x18,
    STATUS_ONE_S_ERROR = 0x1C,
    STATUS_TEN_S_ERROR = 0x20,
    STATUS_HUNDRED_S_ERROR = 0x24,
    STATUS_DAC_TUNED_VAL = 0x28,
    STATUS_ACCURACY = 0x2C,
    STATUS_PPS_ACTIVE = 0x30,
    STATUS_STATE = 0x34,
};

//  Control bit fields
constexpr int CONTROL_EN_OFFSET = 0;
constexpr int CONTROL_EN_SIZE = 1;
constexpr int CONTROL_CLK_SEL_OFFSET = 1;
constexpr int CONTROL_CLK_SEL_SIZE = 1;

uint64_t setField(uint64_t currRegValue, uint64_t newBitValue, int bitOffset, int size)
{
    uint64_t mask = ((1ULL << size) - 1ULL) << bitOffset;
    return ((currRegValue & ~mask) | ((newBitValue << bitOffset) & mask));
}

} // namespace

PPSDO_FPGA::PPSDO_FPGA(std::shared_ptr<ICSR> csr, uint64_t baseAddress)
    : csr(csr)
    , baseAddress(baseAddress)
{
}

uint64_t PPSDO_FPGA::ReadRegister(uint32_t offset, OpStatus& status)
{
    const uint64_t address = baseAddress + offset;
    uint64_t value = csr->ioRead64(address, &status);
    // lime::debug("PPSDO: Read 0x%016" PRIx64 " from register 0x%016" PRIx64, value, address);
    return value;
}

OpStatus PPSDO_FPGA::WriteRegister(uint32_t offset, uint64_t value)
{
    const uint64_t address = baseAddress + offset;
    lime::debug("PPSDO: Writing 0x%016" PRIx64 " value to register 0x%016" PRIx64, value, address);
    return csr->ioWrite64(address, value);
}

OpStatus PPSDO_FPGA::Configure(const PPSDO::Config& config)
{
    const double freq = config.clockFrequency;
    const double ppm = config.tolerancePPM;
    if (freq <= 0 || ppm < 0 || freq > std::numeric_limits<uint32_t>::max())
    {
        lime::error("PPSDO: invalid configuration, clock %g Hz, tolerance %g ppm", freq, ppm);
        return OpStatus::InvalidValue;
    }

    const uint64_t target_1s = static_cast<uint64_t>(freq);
    const uint64_t target_10s = static_cast<uint64_t>(freq * 10);
    const uint64_t target_100s = static_cast<uint64_t>(freq * 100);
    const uint64_t tol_1s_hz = static_cast<uint64_t>(std::ceil(freq * ppm / 1e6));
    const uint64_t tol_10s_hz = static_cast<uint64_t>(std::ceil(freq * ppm / 1e5));
    const uint64_t tol_100s_hz = static_cast<uint64_t>(std::ceil(freq * ppm / 1e4));
    lime::info("PPSDO: targets 1s:%" PRIu64 " 10s:%" PRIu64 " 100s:%" PRIu64 ", tolerances 1s:%" PRIu64 " 10s:%" PRIu64
               " 100s:%" PRIu64 " Hz",
        target_1s,
        target_10s,
        target_100s,
        tol_1s_hz,
        tol_10s_hz,
        tol_100s_hz);

    const std::pair<Register, uint64_t> values[] = {
        { CONFIG_ONE_S_TARGET, target_1s },
        { CONFIG_ONE_S_TOL, tol_1s_hz },
        { CONFIG_TEN_S_TARGET, target_10s },
        { CONFIG_TEN_S_TOL, tol_10s_hz },
        { CONFIG_HUNDRED_S_TARGET, target_100s },
        { CONFIG_HUNDRED_S_TOL, tol_100s_hz },
    };
    for (const auto& [reg, value] : values)
    {
        OpStatus status = WriteRegister(reg, value);
        if (status != OpStatus::Success)
            return status;
    }

    OpStatus status = OpStatus::Success;
    uint64_t control = ReadRegister(CONTROL, status);
    if (status != OpStatus::Success)
        return status;
    // TODO: Update this later with the new revision of data sheet, for now defaulting to clk_sel 0
    // (ceil(clk) == 10.0 || floor(clk) == 10.0 ) ? 1ULL : 0ULL;
    const uint64_t clk_sel = 0;
    control = setField(control, clk_sel, CONTROL_CLK_SEL_OFFSET, CONTROL_CLK_SEL_SIZE);
    return WriteRegister(CONTROL, control);
}

OpStatus PPSDO_FPGA::Enable(bool enable)
{
    OpStatus status = OpStatus::Success;
    uint64_t control = ReadRegister(CONTROL, status);
    if (status != OpStatus::Success)
        return status;

    control = setField(control, static_cast<uint64_t>(enable), CONTROL_EN_OFFSET, CONTROL_EN_SIZE);
    return WriteRegister(CONTROL, control);
}

OpStatus PPSDO_FPGA::GetStatus(PPSDO::Status& out)
{
    OpStatus status = OpStatus::Success;
    PPSDO::Status result;

    result.enabled = ReadRegister(CONTROL, status) & (1ULL << CONTROL_EN_OFFSET);
    if (status == OpStatus::Success)
        result.error1s = static_cast<int32_t>(ReadRegister(STATUS_ONE_S_ERROR, status));
    if (status == OpStatus::Success)
        result.error10s = static_cast<int32_t>(ReadRegister(STATUS_TEN_S_ERROR, status));
    if (status == OpStatus::Success)
        result.error100s = static_cast<int32_t>(ReadRegister(STATUS_HUNDRED_S_ERROR, status));
    if (status == OpStatus::Success)
        result.dacValue = static_cast<uint32_t>(ReadRegister(STATUS_DAC_TUNED_VAL, status));
    if (status == OpStatus::Success)
        result.accuracy = static_cast<uint8_t>(ReadRegister(STATUS_ACCURACY, status));
    if (status == OpStatus::Success)
        result.ppsActive = ReadRegister(STATUS_PPS_ACTIVE, status) != 0;

    uint64_t state = 0;
    if (status == OpStatus::Success)
        state = ReadRegister(STATUS_STATE, status);
    if (status != OpStatus::Success)
        return status;

    if (state == 0)
        result.state = TuneState::CoarseTune;
    else if (state == 1)
        result.state = TuneState::FineTune;
    else
        result.state = TuneState::Unknown;

    out = result;
    return OpStatus::Success;
}

} // namespace lime
