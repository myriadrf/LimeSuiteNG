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
