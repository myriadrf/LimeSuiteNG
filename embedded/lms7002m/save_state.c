#include "privates.h"

struct lms7002m_context;

static const uint16_t chipStateAddr[][2] = {
    { 0x0021, 0x002F }, // LimeLight
    { 0x0081, 0x0082 }, // EN_DIR Configuration + AFE
    { 0x0084, 0x0084 }, // BIAS
    { 0x0085, 0x0085 }, // XBUF
    { 0x0086, 0x008C }, // CGEN
    { 0x0092, 0x00A7 }, // LDO
    { 0x00A8, 0x00A8 }, // BIST
    { 0x00AD, 0x00AE }, // CDS
    { 0x0100, 0x0104 }, // TRF
    { 0x0105, 0x010A }, // TBB
    { 0x010C, 0x0114 }, // RFE
    { 0x0115, 0x011A }, // RBB
    { 0x0200, 0x020C }, // TxTSP
    { 0x0240, 0x0261 }, // TxNCO
    { 0x0400, 0x040F }, // RxTSP
    { 0x0440, 0x0461 }, // RxNCO
    { 0x0500, 0x05A7 }, // GFIR3
    { 0x05C0, 0x05C0 }, // DC Calibration Configuration
};
static uint16_t x0020state;
static uint16_t chipStateData[359]; // UPDATE IF SOMETHING CHANGES

void lms7002m_save_chip_state(struct lms7002m_context* self, bool wr)
{
    uint16_t dest = 0;
    const uint16_t ch = lms7002m_spi_read(self, 0x0020);
    if (!wr)
        x0020state = ch;

    for (uint8_t i = 0; i < sizeof(chipStateAddr) / sizeof(chipStateAddr[0]); ++i)
    {
        for (uint16_t addr = chipStateAddr[i][0]; addr <= chipStateAddr[i][1]; ++addr)
        {
            if (wr)
                lms7002m_spi_write(self, addr, chipStateData[dest]);
            else
                chipStateData[dest] = lms7002m_spi_read(self, addr);
            ++dest;
        }
    }

    // sxr and sxt
    for (uint8_t i = 0; i < 2; ++i)
    {
        lms7002m_spi_write(self, 0x0020, 0xFFFD + i);
        for (uint16_t addr = 0x011C; addr <= 0x0123; ++addr)
        {
            if (wr)
                lms7002m_spi_write(self, addr, chipStateData[dest]);
            else
                chipStateData[dest] = lms7002m_spi_read(self, addr);
            ++dest;
        }
    }

    if (wr)
    {
        //MCLK2 toggle
        uint16_t reg = lms7002m_spi_read(self, 0x002B);
        lms7002m_spi_write(self, 0x002B, reg ^ (1 << 9));
        lms7002m_spi_write(self, 0x002B, reg);

        //TSP logic reset
        reg = lms7002m_spi_read(self, 0x0020);
        lms7002m_spi_write(self, 0x0020, reg & ~0xAA00);
        lms7002m_spi_write(self, 0x0020, reg);
        lms7002m_spi_write(self, 0x0020, x0020state);
    }
    else
    {
        lms7002m_spi_write(self, 0x0020, ch);
    }
}
