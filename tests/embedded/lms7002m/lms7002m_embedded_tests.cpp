#include "lms7002m_embedded_tests.h"

#include "limesuiteng/embedded/lms7002m/lms7002m.h"

#include "lms7002m/csr_data.h"
#include "lms7002m/spi.h"

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

TEST_F(lms7002m_embedded, lms7002m_spi_write_ValueIsCorrect)
{
    const uint16_t writeAddr = 0x0020;
    const uint16_t writeValue = 0xABCD;
    spi_stub.registers[0][writeAddr] = 0xFFFF;
    lms7002m_spi_write(chip, writeAddr, writeValue);
    ASSERT_EQ(spi_stub.registers[0][writeAddr], writeValue);
}

TEST_F(lms7002m_embedded, lms7002m_spi_read_ValueIsCorrect)
{
    const uint16_t readAddr = 0x0020;
    spi_stub.registers[0][readAddr] = 0xFFFD;
    const uint16_t readValue = lms7002m_spi_read(chip, readAddr);
    ASSERT_EQ(readValue, spi_stub.registers[0][readAddr]);
}

TEST_F(lms7002m_embedded, lms7002m_spi_modify_ValueIsCorrect)
{
    const uint16_t addr = 0x0020;
    spi_stub.registers[0][addr] = 0x00FD;
    lms7002m_spi_modify(chip, addr, 12, 8, 0xA);
    ASSERT_EQ(spi_stub.registers[0][addr], 0x0AFD);
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

TEST_F(lms7002m_embedded, lms7002m_set_frequency_cgen_SetGet_ValueMatch)
{
    spi_stub.Set(0, { 0x008C, 13, 12 }, 2); // force tune success
    uint64_t expectedFreq = 122880000;
    ASSERT_EQ(lms7002m_set_frequency_cgen(chip, expectedFreq), lime_Result_Success);
    EXPECT_NEAR(lms7002m_get_frequency_cgen(chip), expectedFreq, 10);
}

TEST_F(lms7002m_embedded, lms7002m_set_nco_frequencies_SetGet_ValuesMatch)
{
    std::array<uint32_t, 16> freqs;
    for (size_t i = 0; i < freqs.size(); ++i)
        freqs[i] = i * 1e3;

    bool isTx = true;
    const int16_t phaseOffset = 123;
    ASSERT_EQ(lms7002m_set_active_channel(chip, LMS7002M_CHANNEL_A), lime_Result_Success);
    ASSERT_EQ(lms7002m_set_nco_frequencies(chip, isTx, freqs.data(), freqs.size(), phaseOffset), lime_Result_Success);

    std::array<uint32_t, 16> freqs_readback{};
    int16_t phaseOffset_readback{ 0 };
    ASSERT_EQ(lms7002m_get_nco_frequencies(chip, isTx, freqs_readback.data(), freqs_readback.size(), &phaseOffset_readback),
        lime_Result_Success);

    ASSERT_EQ(freqs_readback, freqs);
    ASSERT_EQ(phaseOffset_readback, phaseOffset);
}

TEST_F(lms7002m_embedded, lms7002m_set_nco_phases_SetGet_ValuesMatch)
{
    std::array<int16_t, 16> pho;
    for (size_t i = 0; i < pho.size(); ++i)
        pho[i] = -32768 + i * (65535 / 16);

    bool isTx = true;
    const uint32_t freqOffset = 123456;
    ASSERT_EQ(lms7002m_set_active_channel(chip, LMS7002M_CHANNEL_A), lime_Result_Success);
    ASSERT_EQ(lms7002m_set_nco_phases(chip, isTx, pho.data(), pho.size(), freqOffset), lime_Result_Success);

    std::array<int16_t, 16> pho_readback{};
    uint32_t freqOffset_readback{ 0 };
    ASSERT_EQ(
        lms7002m_get_nco_phases(chip, isTx, pho_readback.data(), pho_readback.size(), &freqOffset_readback), lime_Result_Success);

    ASSERT_EQ(pho_readback, pho);
    ASSERT_EQ(freqOffset, freqOffset);
}

TEST_F(lms7002m_embedded, lms7002m_set_gfir_coefficients_SetGet_ValuesMatch)
{
    std::array<int16_t, 120> coefs;
    for (size_t i = 0; i < coefs.size(); ++i)
        coefs[i] = -32768 + i * (65536 / 120);

    bool isTx = true;
    ASSERT_EQ(lms7002m_set_active_channel(chip, LMS7002M_CHANNEL_A), lime_Result_Success);
    ASSERT_EQ(lms7002m_set_gfir_coefficients(chip, isTx, 2, coefs.data(), coefs.size()), lime_Result_Success);

    std::array<int16_t, 120> coefs_readback{};
    ASSERT_EQ(lms7002m_get_gfir_coefficients(chip, isTx, 2, coefs_readback.data(), coefs_readback.size()), lime_Result_Success);

    ASSERT_EQ(coefs_readback, coefs);
}

TEST_F(lms7002m_embedded, lms7002m_set_frequency_sx_SetGet_ValueMatch)
{
    spi_stub.Set(0, { 0x0123, 13, 12 }, 2); // force tune success
    uint64_t expectedFreq = 1400000000;
    bool isTx = false;
    ASSERT_EQ(lms7002m_set_frequency_sx(chip, isTx, expectedFreq), lime_Result_Success);
    EXPECT_NEAR(lms7002m_get_frequency_sx(chip, isTx), expectedFreq, 10);
}