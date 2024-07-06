#include "lms7002m_embedded_tests.h"

#include "limesuiteng/embedded/lms7002m/lms7002m.h"

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

TEST_F(lms7002m_embedded, SetActiveChannel_IsCorrectValue)
{
    ASSERT_EQ(lms7002m_set_active_channel(chip, LMS7002M_CHANNEL_A), lime_Result_Success);
    ASSERT_EQ(spi_stub.registers[0].at(0x0020) & 0x3, 1);
    ASSERT_EQ(lms7002m_set_active_channel(chip, LMS7002M_CHANNEL_B), lime_Result_Success);
    ASSERT_EQ(spi_stub.registers[0].at(0x0020) & 0x3, 2);
    ASSERT_EQ(lms7002m_set_active_channel(chip, LMS7002M_CHANNEL_AB), lime_Result_Success);
    ASSERT_EQ(spi_stub.registers[0].at(0x0020) & 0x3, 3);
    ASSERT_EQ(lms7002m_set_active_channel(chip, LMS7002M_CHANNEL_SXR), lime_Result_Success);
    ASSERT_EQ(spi_stub.registers[0].at(0x0020) & 0x3, 1);
    ASSERT_EQ(lms7002m_set_active_channel(chip, LMS7002M_CHANNEL_SXT), lime_Result_Success);
    ASSERT_EQ(spi_stub.registers[0].at(0x0020) & 0x3, 2);
}

TEST_F(lms7002m_embedded, GetActiveChannel_IsCorrectValue)
{
    spi_stub.registers[0].at(0x0020) = 0x0001;
    ASSERT_EQ(lms7002m_get_active_channel(chip), LMS7002M_CHANNEL_A);
    spi_stub.registers[0].at(0x0020) = 0x0002;
    ASSERT_EQ(lms7002m_get_active_channel(chip), LMS7002M_CHANNEL_B);
    spi_stub.registers[0].at(0x0020) = 0x0003;
    ASSERT_EQ(lms7002m_get_active_channel(chip), LMS7002M_CHANNEL_AB);
    spi_stub.registers[0].at(0x0020) = 0x0001;
    ASSERT_EQ(lms7002m_get_active_channel(chip), LMS7002M_CHANNEL_SXR);
    spi_stub.registers[0].at(0x0020) = 0x0002;
    ASSERT_EQ(lms7002m_get_active_channel(chip), LMS7002M_CHANNEL_SXT);
}
