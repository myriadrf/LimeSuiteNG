#include <ciso646>
#include <stdint.h>
#include "limesuiteng/LMS7002M.h"
#include "limesuiteng/types.h"
#include "LMS7002MCSR_Data.h"

static lime::LMS7002M* serPort;

extern "C" float bandwidthRF;
extern "C" float RefClk;

void SetupCalibrations(lime::LMS7002M* chip, double BW)
{
    serPort = chip;
    bandwidthRF = BW;
    RefClk = chip->GetReferenceClk_SX(lime::TRXDir::Rx);
}

#include <vector>
#include <string>
#include <fstream>
using namespace std;
using namespace lime::LMS7002MCSR_Data;

//spiAddrReg might not have SPI write bit, add it here if necessary
void SPI_write(unsigned short spiAddrReg, unsigned short spiDataReg)
{
    serPort->SPI_write(spiAddrReg, spiDataReg);
}

unsigned short SPI_read(unsigned short spiAddrReg)
{
    return serPort->SPI_read(spiAddrReg);
}

void Modify_SPI_Reg_bits_WrOnly(
    const uint16_t address, const uint8_t msb, const uint8_t lsb, const uint16_t value, const uint16_t spiDataReg)
{
    uint16_t spiMask = (~(~0u << (msb - lsb + 1))) << (lsb); // creates bit mask
    // spiDataReg = (spiDataReg & (~spiMask)) | ((value << lsb) & spiMask); //clear bits
    return SPI_write(address, (spiDataReg & (~spiMask)) | ((value << lsb) & spiMask)); //write modified data back to SPI reg
}

void Modify_SPI_Reg_bits(const uint16_t address, const uint8_t msb, const uint8_t lsb, const uint16_t value)
{
    uint16_t spiDataReg = SPI_read(address); //read current SPI reg data
    uint16_t spiMask = (~(~0u << (msb - lsb + 1))) << (lsb); // creates bit mask
    spiDataReg = (spiDataReg & (~spiMask)) | ((value << lsb) & spiMask); //clear bits
    return SPI_write(address, spiDataReg); //write modified data back to SPI reg
}

void Modify_SPI_Reg_bits(const CSRegister& param, const uint16_t value)
{
    return Modify_SPI_Reg_bits(param.address, param.msb, param.lsb, value);
}

uint16_t Get_SPI_Reg_bits(uint16_t address, uint8_t msb, uint8_t lsb)
{
    return (SPI_read(address) & (~(~0u << (msb + 1)))) >> lsb; //shift bits to LSB
}

uint16_t Get_SPI_Reg_bits(const CSRegister& reg)
{
    return Get_SPI_Reg_bits(reg.address, reg.msb, reg.lsb);
}

void SPI_read_batch(const uint16_t* addr, uint16_t* values, uint8_t cnt)
{
    serPort->SPI_read_batch(addr, values, cnt);
}
void SPI_write_batch(const uint16_t* addr, const uint16_t* values, uint8_t cnt)
{
    serPort->SPI_write_batch(addr, values, cnt);
}

void Modify_SPI_Reg_mask(const uint16_t* addr, const uint16_t* values, const uint16_t* masks, uint8_t cnt)
{
    uint8_t i;
    for (i = 0; i < cnt; ++i)
        SPI_write(addr[i], (SPI_read(addr[i]) & ~masks[i]) | (values[i] & masks[i]));
}
