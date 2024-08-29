#include "lms7002m_embedded_tests.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "limesuiteng/embedded/lms7002m/lms7002m.h"
#include "lms7002m/csr_data.h"
#include "lms7002m/spi.h"

// namespace lime {
// namespace testing {
using namespace lime::testing;

struct SetExpectPair {
    double input;
    double expectedOutput;
    double referenceClk;

    friend std::ostream& operator<<(std::ostream& os, const SetExpectPair& obj)
    {
        char str[64];
        double integer;
        int64_t fractional = std::modf(obj.input, &integer) * 1e9;
        snprintf(str, sizeof(str), "%lip%li", int64_t(integer), fractional);
        os << "set_" << str;
        fractional = std::modf(obj.expectedOutput, &integer) * 1e9;
        snprintf(str, sizeof(str), "%lip%li", int64_t(integer), fractional);
        os << "_expect_" << str;
        fractional = std::modf(obj.referenceClk, &integer) * 1e9;
        snprintf(str, sizeof(str), "%lip%li", int64_t(integer), fractional);
        os << "_ref_" << str;
        return os;
    }
};

class lms7002mEmbeddedLOprecision : public ::testing::TestWithParam<SetExpectPair>
{
  protected:
    lms7002mEmbeddedLOprecision(){};

    void SetUp() override
    {
        lms7002m_hooks hooks{};
        memset(&hooks, 0, sizeof(hooks));

        hooks.spi16_userData = &spi_stub;
        hooks.spi16_transact = LMS7002M_SPI_STUB::spi16_transact;

        chip = lms7002m_create(&hooks);
        ASSERT_NE(chip, nullptr);
    }

    void TearDown() override { lms7002m_destroy(chip); }

    LMS7002M_SPI_STUB spi_stub;

  public:
    lms7002m_context* chip;
};

TEST_P(lms7002mEmbeddedLOprecision, SetGetValueMatch)
{
    const SetExpectPair& params = GetParam();

    spi_stub.Set(0, { 0x0123, 13, 12 }, 2); // force tune success

    // uint64_t expectedFreq = 1400000000;
    constexpr bool isTx = false; // math is the same for Rx/Tx
    lms7002m_set_reference_clock(chip, params.referenceClk);
    struct lms7002m_fraction frac_in {
        int64_t(params.input * 1000000), 1000000
    };
    ASSERT_EQ(lms7002m_set_frequency_sx_fractional(chip, isTx, frac_in), lime_Result_Success);
    struct lms7002m_fraction frac_out = lms7002m_get_frequency_sx_fractional(chip, isTx);
    constexpr double epsilon{ 0.001 };
    const double output = double(frac_out.num) / frac_out.den;
    EXPECT_NEAR(output, params.expectedOutput, epsilon);
}

INSTANTIATE_TEST_SUITE_P(precisiontest,
    lms7002mEmbeddedLOprecision,
    ::testing::Values(
        // TODO: add edge cases, where internally fractional part rounding up/down would need integer part change
        SetExpectPair{ 868000000, 868000000.9537, 26000000 },
        SetExpectPair{ 868000000, 867999997.5586, 30720000 },
        SetExpectPair{ 1000000000, 999999997.5586, 30720000 },
        SetExpectPair{ 1500000000, 1500000014.6484, 30720000 },
        SetExpectPair{ 2500000000, 2500000004.8828, 30720000 },
        SetExpectPair{ 3500000000, 3500000009.7656, 30720000 }),
    ::testing::PrintToStringParamName());

// }
// }
