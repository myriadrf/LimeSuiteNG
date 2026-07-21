#include <gtest/gtest.h>

#include <cstring>
#include <memory>
#include <vector>

#include "boards/LMS7002M_SDRDevice.h"
#include "comms/SPI/ISPI.h"
#include "limesuiteng/LMS7002M.h"
#include "limesuiteng/OpStatus.h"
#include "limesuiteng/RFStream.h"

using namespace lime;

namespace {

// Answers reads with zeroes and records every register write
class RecordingSPIStub : public ISPI
{
  public:
    struct Write {
        uint16_t address;
        uint16_t value;
    };

    OpStatus Transact(const uint32_t* MOSI, uint32_t* MISO, uint32_t count) override
    {
        for (uint32_t i = 0; i < count; ++i)
        {
            if (MOSI[i] & (1u << 31)) // msbit 1=SPI write
                writes.push_back({ static_cast<uint16_t>((MOSI[i] >> 16) & 0x7FFF), static_cast<uint16_t>(MOSI[i] & 0xFFFF) });
        }
        if (MISO != nullptr)
            std::memset(MISO, 0, count * sizeof(uint32_t));
        return OpStatus::Success;
    }

    std::vector<Write> writes;
};

// Bare LMS7002M_SDRDevice with a single stub-backed chip
class TestDevice : public LMS7002M_SDRDevice
{
  public:
    explicit TestDevice(std::shared_ptr<ISPI> spi) { mLMSChips.push_back(std::make_unique<LMS7002M>(spi)); }

    OpStatus Configure(const SDRConfig& config, uint8_t moduleIndex) override { return OpStatus::Success; }
    OpStatus Init() override { return OpStatus::Success; }
    double GetClockFreq(uint8_t clk_id, uint8_t channel) override { return 0.0; }
    OpStatus SetClockFreq(uint8_t clk_id, double freq, uint8_t channel) override { return OpStatus::Success; }
    OpStatus SetSampleRate(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double sampleRate, uint8_t oversample) override
    {
        return OpStatus::Success;
    }
    std::unique_ptr<RFStream> StreamCreate(const StreamConfig& config, uint8_t moduleIndex) override { return nullptr; }
};

constexpr uint16_t MAC_ADDR = 0x0020;

bool WroteMACValue(const std::vector<RecordingSPIStub::Write>& writes, uint16_t macBits)
{
    for (const auto& write : writes)
    {
        if (write.address == MAC_ADDR && (write.value & 0x3) == macBits)
            return true;
    }
    return false;
}

} // namespace

namespace lime::testing {

TEST(LMS7002M_SDRDevice_GFIR, SetCoefficientsSelectsRequestedChannel)
{
    auto spi = std::make_shared<RecordingSPIStub>();
    TestDevice device(spi);

    spi->writes.clear();
    // stub registers read back as oversample(2), which limits GFIR1 to 10 coefficients
    const std::vector<double> coefficients(8, 0.5);
    ASSERT_EQ(device.SetGFIRCoefficients(0, TRXDir::Rx, 1, 0, coefficients), OpStatus::Success);
    EXPECT_TRUE(WroteMACValue(spi->writes, 0x2)) << "channel B was never selected";
}

TEST(LMS7002M_SDRDevice_GFIR, GetCoefficientsSelectsRequestedChannel)
{
    auto spi = std::make_shared<RecordingSPIStub>();
    TestDevice device(spi);

    spi->writes.clear();
    device.GetGFIRCoefficients(0, TRXDir::Rx, 1, 0);
    EXPECT_TRUE(WroteMACValue(spi->writes, 0x2)) << "channel B was never selected";
}

TEST(LMS7002M_SDRDevice_GFIR, SetCoefficientsRejectsOversizedCount)
{
    auto spi = std::make_shared<RecordingSPIStub>();
    TestDevice device(spi);

    // 256 used to wrap to a count of 0 and report success while writing nothing
    EXPECT_EQ(device.SetGFIRCoefficients(0, TRXDir::Rx, 0, 0, std::vector<double>(256, 0.0)), OpStatus::OutOfRange);
    EXPECT_EQ(device.SetGFIRCoefficients(0, TRXDir::Rx, 0, 0, std::vector<double>(41, 0.0)), OpStatus::OutOfRange);
    EXPECT_EQ(device.SetGFIRCoefficients(0, TRXDir::Tx, 0, 2, std::vector<double>(121, 0.0)), OpStatus::OutOfRange);
    EXPECT_EQ(device.SetGFIRCoefficients(0, TRXDir::Rx, 0, 3, std::vector<double>(10, 0.0)), OpStatus::OutOfRange);
}

TEST(LMS7002M_SDRDevice_GFIR, GetCoefficientsReportsFailure)
{
    auto spi = std::make_shared<RecordingSPIStub>();
    TestDevice device(spi);

    // an invalid filter index used to return a zero-filled buffer as success
    EXPECT_TRUE(device.GetGFIRCoefficients(0, TRXDir::Rx, 0, 3).empty());
}

} // namespace lime::testing
