#include "lms7002m_embedded_tests.h"

#include "limesuiteng/embedded/lms7002m/lms7002m.h"

#include "lms7002m/csr_data.h"

using namespace lime::testing;

lms7002m_embedded::lms7002m_embedded()
{
}

void lms7002m_embedded::SetUp()
{
    lms7002m_hooks hooks{};
    memset(&hooks, 0, sizeof(hooks));

    hooks.spi16_userData = &spi_stub;
    hooks.spi16_transact = LMS7002M_SPI_STUB::spi16_transact;

    chip = lms7002m_create(&hooks);
    ASSERT_NE(chip, nullptr);
}

void lms7002m_embedded::TearDown()
{
    lms7002m_destroy(chip);
}

TEST_F(lms7002m_embedded, lms7002m_create_DoesNotModifyRegisters)
{
    ASSERT_EQ(spi_stub.writeCount, 0);
    ASSERT_EQ(spi_stub.readCount, 0);
}

TEST_F(lms7002m_embedded, lms7002m_destroy_DoesNotModifyRegisters)
{
    lms7002m_destroy(chip);
    chip = nullptr;
    ASSERT_EQ(spi_stub.writeCount, 0);
    ASSERT_EQ(spi_stub.readCount, 0);
}

TEST_F(lms7002m_embedded, ReferenceClock_SetGet_Matches)
{
    uint64_t testValue = 10e6;
    lime_Result result = lms7002m_set_reference_clock(chip, testValue);
    ASSERT_EQ(result, lime_Result_Success);
    uint64_t getValue = lms7002m_get_reference_clock(chip);
    ASSERT_EQ(getValue, testValue);
}

TEST_F(lms7002m_embedded, SetActiveChannel_ReportCorrectValue)
{
    ASSERT_EQ(lms7002m_set_active_channel(chip, LMS7002M_CHANNEL_A), lime_Result_Success);
    ASSERT_EQ(spi_stub.Get(0, LMS7002M_MAC), 1);
    ASSERT_EQ(lms7002m_set_active_channel(chip, LMS7002M_CHANNEL_B), lime_Result_Success);
    ASSERT_EQ(spi_stub.Get(0, LMS7002M_MAC), 2);
    ASSERT_EQ(lms7002m_set_active_channel(chip, LMS7002M_CHANNEL_AB), lime_Result_Success);
    ASSERT_EQ(spi_stub.Get(0, LMS7002M_MAC), 3);
    ASSERT_EQ(lms7002m_set_active_channel(chip, LMS7002M_CHANNEL_SXR), lime_Result_Success);
    ASSERT_EQ(spi_stub.Get(0, LMS7002M_MAC), 1);
    ASSERT_EQ(lms7002m_set_active_channel(chip, LMS7002M_CHANNEL_SXT), lime_Result_Success);
    ASSERT_EQ(spi_stub.Get(0, LMS7002M_MAC), 2);
}

TEST_F(lms7002m_embedded, GetActiveChannel_ReportCorrectValue)
{
    spi_stub.Set(0, LMS7002M_MAC, 1);
    ASSERT_EQ(lms7002m_get_active_channel(chip), LMS7002M_CHANNEL_A);
    spi_stub.Set(0, LMS7002M_MAC, 2);
    ASSERT_EQ(lms7002m_get_active_channel(chip), LMS7002M_CHANNEL_B);
    spi_stub.Set(0, LMS7002M_MAC, 3);
    ASSERT_EQ(lms7002m_get_active_channel(chip), LMS7002M_CHANNEL_AB);
    spi_stub.Set(0, LMS7002M_MAC, 1);
    ASSERT_EQ(lms7002m_get_active_channel(chip), LMS7002M_CHANNEL_SXR);
    spi_stub.Set(0, LMS7002M_MAC, 2);
    ASSERT_EQ(lms7002m_get_active_channel(chip), LMS7002M_CHANNEL_SXT);
}

TEST_F(lms7002m_embedded, lms7002m_tune_cgen_vco_ComparatorFail_ReportError)
{
    // Setting LMS7002M_VCO_CMPHO_CGEN, LMS7002M_VCO_CMPLO_CGEN values
    spi_stub.Set(0, { 0x008C, 13, 12 }, 0);
    ASSERT_NE(lms7002m_tune_cgen_vco(chip), lime_Result_Success);

    spi_stub.Set(0, { 0x008C, 13, 12 }, 1);
    ASSERT_NE(lms7002m_tune_cgen_vco(chip), lime_Result_Success);

    spi_stub.Set(0, { 0x008C, 13, 12 }, 3);
    ASSERT_NE(lms7002m_tune_cgen_vco(chip), lime_Result_Success);
}

TEST_F(lms7002m_embedded, lms7002m_tune_cgen_vco_ComparatorOk_ReportSuccess)
{
    spi_stub.Set(0, { 0x008C, 13, 12 }, 2);
    ASSERT_EQ(lms7002m_tune_cgen_vco(chip), lime_Result_Success);
}

TEST_F(lms7002m_embedded, lms7002m_set_frequency_cgen_SetInvalidFreq_ReportError)
{
    ASSERT_EQ(lms7002m_set_frequency_cgen(chip, 0), lime_Result_InvalidValue);
    ASSERT_EQ(lms7002m_set_frequency_cgen(chip, 641e6), lime_Result_OutOfRange);
}

TEST_F(lms7002m_embedded, lms7002m_set_frequency_cgen_SetValidFreq_ReportSuccess)
{
    spi_stub.Set(0, { 0x008C, 13, 12 }, 2); // allow tune success
    ASSERT_EQ(lms7002m_set_frequency_cgen(chip, 122.88e6), lime_Result_Success);
}

TEST_F(lms7002m_embedded, lms7002m_set_frequency_cgen_SetValidFreq_TuneFails_ReportError)
{
    spi_stub.Set(0, { 0x008C, 13, 12 }, 0); // force tune failure
    ASSERT_EQ(lms7002m_set_frequency_cgen(chip, 122.88e6), lime_Result_Error);
}