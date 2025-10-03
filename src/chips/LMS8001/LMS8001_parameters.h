#ifndef LIME_LMS8001_PARAMETERS_H
#define LIME_LMS8001_PARAMETERS_H

#include <stdint.h>
#include <vector>

namespace lime {

struct LMS8Parameter {
    uint16_t address;
    uint8_t msb;
    uint8_t lsb;
    uint16_t defaultValue;
    const char* name;
    const char* tooltip;
    inline bool operator==(const LMS8Parameter& obj)
    {
        if (address == obj.address && msb == obj.msb && lsb == obj.lsb)
            return true;
        return false;
    };
};

static const struct LMS8Parameter SPI_SDIO_DS = { 0x0000, 6, 6, 0, "SPI_SDIO_DS", "Driver strength of SDIO pad" };
static const struct LMS8Parameter SPI_SDO_DS = { 0x0000, 5, 5, 0, "SPI_SDO_DS", "Driver strength of SDO pad" };
static const struct LMS8Parameter SPI_SDIO_PE = { 0x0000, 4, 4, 1, "SPI_SDIO_PE", "Pull up control of SDIO pad" };
static const struct LMS8Parameter SPI_SDO_PE = { 0x0000, 3, 3, 1, "SPI_SDO_PE", "Pull up control of SDO pad" };
static const struct LMS8Parameter SPI_SCLK_PE = { 0x0000, 2, 2, 1, "SPI_SCLK_PE", "Pull up control of SCLK pad" };
static const struct LMS8Parameter SPI_SEN_PE = { 0x0000, 1, 1, 1, "SPI_SEN_PE", "Pull up control of SEN pad" };
static const struct LMS8Parameter SPIMODE = { 0x0000, 0, 0, 1, "SPIMODE", "SPI mode" };
static const struct LMS8Parameter GPIO_OUT_SPI = { 0x0004, 8, 0, 0, "GPIO_OUT_SPI", "Output data for GPIO pads from SPI" };
static const struct LMS8Parameter GPIO_OUT_SPI_0 = { 0x0004, 0, 0, 0, "GPIO_OUT_SPI_0", "Output data for GPIO0 pad from SPI" };
static const struct LMS8Parameter GPIO_OUT_SPI_1 = { 0x0004, 1, 1, 0, "GPIO_OUT_SPI_1", "Output data for GPIO1 pad from SPI" };
static const struct LMS8Parameter GPIO_OUT_SPI_2 = { 0x0004, 2, 2, 0, "GPIO_OUT_SPI_2", "Output data for GPIO2 pad from SPI" };
static const struct LMS8Parameter GPIO_OUT_SPI_3 = { 0x0004, 3, 3, 0, "GPIO_OUT_SPI_3", "Output data for GPIO3 pad from SPI" };
static const struct LMS8Parameter GPIO_OUT_SPI_4 = { 0x0004, 4, 4, 0, "GPIO_OUT_SPI_4", "Output data for GPIO4 pad from SPI" };
static const struct LMS8Parameter GPIO_OUT_SPI_5 = { 0x0004, 5, 5, 0, "GPIO_OUT_SPI_5", "Output data for GPIO5 pad from SPI" };
static const struct LMS8Parameter GPIO_OUT_SPI_6 = { 0x0004, 6, 6, 0, "GPIO_OUT_SPI_6", "Output data for GPIO6 pad from SPI" };
static const struct LMS8Parameter GPIO_OUT_SPI_7 = { 0x0004, 7, 7, 0, "GPIO_OUT_SPI_7", "Output data for GPIO7 pad from SPI" };
static const struct LMS8Parameter GPIO_OUT_SPI_8 = { 0x0004, 8, 8, 0, "GPIO_OUT_SPI_8", "Output data for GPIO8 pad from SPI" };
static const struct LMS8Parameter GPIO0_SEL = { 0x0005, 2, 0, 0, "GPIO0_SEL", "Selects the source for GPIO 0" };
static const struct LMS8Parameter GPIO1_SEL = { 0x0005, 5, 3, 0, "GPIO1_SEL", "Selects the source for GPIO 1" };
static const struct LMS8Parameter GPIO2_SEL = { 0x0005, 8, 6, 0, "GPIO2_SEL", "Selects the source for GPIO 2" };
static const struct LMS8Parameter GPIO3_SEL = { 0x0005, 11, 9, 0, "GPIO3_SEL", "Selects the source for GPIO 3" };
static const struct LMS8Parameter GPIO4_SEL = { 0x0005, 14, 12, 0, "GPIO4_SEL", "Selects the source for GPIO 4" };
static const struct LMS8Parameter GPIO5_SEL = { 0x0006, 2, 0, 0, "GPIO5_SEL", "Selects the source for GPIO 5" };
static const struct LMS8Parameter GPIO6_SEL = { 0x0006, 5, 3, 0, "GPIO6_SEL", "Selects the source for GPIO 6" };
static const struct LMS8Parameter GPIO7_SEL = { 0x0006, 8, 6, 0, "GPIO7_SEL", "Selects the source for GPIO 7" };
static const struct LMS8Parameter GPIO8_SEL = { 0x0006, 11, 9, 0, "GPIO8_SEL", "Selects the source for GPIO 8" };
static const struct LMS8Parameter GPIO_IN = { 0x0008, 8, 0, 0, "GPIO_IN", "Data read from GPIO pads" };
static const struct LMS8Parameter GPIO_IN_0 = { 0x0008, 0, 0, 0, "GPIO_IN_0", "Data read from GPIO pads" };
static const struct LMS8Parameter GPIO_IN_1 = { 0x0008, 1, 1, 0, "GPIO_IN_1", "Data read from GPIO pads" };
static const struct LMS8Parameter GPIO_IN_2 = { 0x0008, 2, 2, 0, "GPIO_IN_2", "Data read from GPIO pads" };
static const struct LMS8Parameter GPIO_IN_3 = { 0x0008, 3, 3, 0, "GPIO_IN_3", "Data read from GPIO pads" };
static const struct LMS8Parameter GPIO_IN_4 = { 0x0008, 4, 4, 0, "GPIO_IN_4", "Data read from GPIO pads" };
static const struct LMS8Parameter GPIO_IN_5 = { 0x0008, 5, 5, 0, "GPIO_IN_5", "Data read from GPIO pads" };
static const struct LMS8Parameter GPIO_IN_6 = { 0x0008, 6, 6, 0, "GPIO_IN_6", "Data read from GPIO pad" };
static const struct LMS8Parameter GPIO_IN_7 = { 0x0008, 7, 7, 0, "GPIO_IN_7", "Data read from GPIO pad" };
static const struct LMS8Parameter GPIO_IN_8 = { 0x0008, 8, 8, 0, "GPIO_IN_8", "Data read from GPIO pad" };
static const struct LMS8Parameter GPIO_PE = { 0x0009, 8, 0, 1, "GPIO_PE", "Pull up control of GPIO pads" };
static const struct LMS8Parameter GPIO_PE_0 = { 0x0009, 0, 0, 1, "GPIO_PE_0", "Pull up control of GPIO0 pad" };
static const struct LMS8Parameter GPIO_PE_1 = { 0x0009, 1, 1, 1, "GPIO_PE_1", "Pull up control of GPIO1 pad" };
static const struct LMS8Parameter GPIO_PE_2 = { 0x0009, 2, 2, 1, "GPIO_PE_2", "Pull up control of GPIO2 pad" };
static const struct LMS8Parameter GPIO_PE_3 = { 0x0009, 3, 3, 1, "GPIO_PE_3", "Pull up control of GPIO3 pad" };
static const struct LMS8Parameter GPIO_PE_4 = { 0x0009, 4, 4, 1, "GPIO_PE_4", "Pull up control of GPIO4 pad" };
static const struct LMS8Parameter GPIO_PE_5 = { 0x0009, 5, 5, 1, "GPIO_PE_5", "Pull up control of GPIO5 pad" };
static const struct LMS8Parameter GPIO_PE_6 = { 0x0009, 6, 6, 1, "GPIO_PE_6", "Pull up control of GPIO6 pad" };
static const struct LMS8Parameter GPIO_PE_7 = { 0x0009, 7, 7, 1, "GPIO_PE_7", "Pull up control of GPIO7 pad" };
static const struct LMS8Parameter GPIO_PE_8 = { 0x0009, 8, 8, 1, "GPIO_PE_8", "Pull up control of GPIO8 pad" };
static const struct LMS8Parameter GPIO_DS = { 0x000A, 8, 0, 0, "GPIO_DS", "Driver strength of GPIO pads" };
static const struct LMS8Parameter GPIO_DS_0 = { 0x000A, 0, 0, 0, "GPIO_DS_0", "Driver strength of GPIO0 pad" };
static const struct LMS8Parameter GPIO_DS_1 = { 0x000A, 1, 1, 0, "GPIO_DS_1", "Driver strength of GPIO1 pad" };
static const struct LMS8Parameter GPIO_DS_2 = { 0x000A, 2, 2, 0, "GPIO_DS_2", "Driver strength of GPIO2 pad" };
static const struct LMS8Parameter GPIO_DS_3 = { 0x000A, 3, 3, 0, "GPIO_DS_3", "Driver strength of GPIO3 pad" };
static const struct LMS8Parameter GPIO_DS_4 = { 0x000A, 4, 4, 0, "GPIO_DS_4", "Driver strength of GPIO4 pad" };
static const struct LMS8Parameter GPIO_DS_5 = { 0x000A, 5, 5, 0, "GPIO_DS_5", "Driver strength of GPIO5 pad" };
static const struct LMS8Parameter GPIO_DS_6 = { 0x000A, 6, 6, 0, "GPIO_DS_6", "Driver strength of GPIO6 pad" };
static const struct LMS8Parameter GPIO_DS_7 = { 0x000A, 7, 7, 0, "GPIO_DS_7", "Driver strength of GPIO7 pad" };
static const struct LMS8Parameter GPIO_DS_8 = { 0x000A, 8, 8, 0, "GPIO_DS_8", "Driver strength of GPIO8 pad" };
static const struct LMS8Parameter GPIO_InO = { 0x000B, 8, 0, 1, "GPIO_InO", "Input/output control of GPIO pads" };
static const struct LMS8Parameter GPIO_InO_0 = { 0x000B, 0, 0, 1, "GPIO_InO_0", "Input/output control of GPIO0 pad" };
static const struct LMS8Parameter GPIO_InO_1 = { 0x000B, 1, 1, 1, "GPIO_InO_1", "Input/output control of GPIO1 pad" };
static const struct LMS8Parameter GPIO_InO_2 = { 0x000B, 2, 2, 1, "GPIO_InO_2", "Input/output control of GPIO2 pad" };
static const struct LMS8Parameter GPIO_InO_3 = { 0x000B, 3, 3, 1, "GPIO_InO_3", "Input/output control of GPIO3 pad" };
static const struct LMS8Parameter GPIO_InO_4 = { 0x000B, 4, 4, 1, "GPIO_InO_4", "Input/output control of GPIO4 pad" };
static const struct LMS8Parameter GPIO_InO_5 = { 0x000B, 5, 5, 1, "GPIO_InO_5", "Input/output control of GPIO5 pad" };
static const struct LMS8Parameter GPIO_InO_6 = { 0x000B, 6, 6, 1, "GPIO_InO_6", "Input/output control of GPIO6 pad" };
static const struct LMS8Parameter GPIO_InO_7 = { 0x000B, 7, 7, 1, "GPIO_InO_7", "Input/output control of GPIO7 pad" };
static const struct LMS8Parameter GPIO_InO_8 = { 0x000B, 8, 8, 1, "GPIO_InO_8", "Input/output control of GPIO8 pad" };
static const struct LMS8Parameter TEMP_SENS_EN = { 0x000C, 10, 10, 0, "TEMP_SENS_EN", "Enable the temperature sensor biasing" };
static const struct LMS8Parameter TEMP_SENS_CLKEN = { 0x000C, 9, 9, 0, "TEMP_SENS_CLKEN", "Temperature sensor clock enable" };
static const struct LMS8Parameter TEMP_START_CONV = { 0x000C, 8, 8, 0, "TEMP_START_CONV", "Start the temperature conversion" };
static const struct LMS8Parameter TEMP_READ = { 0x000C, 7, 0, 0, "TEMP_READ", "Readout of temperature sensor" };
static const struct LMS8Parameter VER = { 0x000F, 15, 11, 8, "VER", "Chip version" };
static const struct LMS8Parameter REV = { 0x000F, 10, 6, 1, "REV", "Chip revision" };
static const struct LMS8Parameter MASK = { 0x000F, 5, 0, 0, "MASK", "Chip mask" };
static const struct LMS8Parameter PD_CALIB_COMP = { 0x0010, 12, 12, 1, "PD_CALIB_COMP", "Calibration comparator power down" };
static const struct LMS8Parameter RP_CALIB_COMP = {
    0x0010, 11, 11, 0, "RP_CALIB_COMP", "Comparator output; Used in rppolywo calibration"
};
static const struct LMS8Parameter RP_CALIB_BIAS = {
    0x0010, 10, 6, 16, "RP_CALIB_BIAS", "Calibration code for rppolywo; Code is set by calibration algorithm"
};
static const struct LMS8Parameter PD_FRP_BIAS = { 0x0010, 4, 4, 0, "PD_FRP_BIAS", "Power down signal for Fix/RP" };
static const struct LMS8Parameter PD_F_BIAS = { 0x0010, 3, 3, 0, "PD_F_BIAS", "Power down signal for Fix" };
static const struct LMS8Parameter PD_PTRP_BIAS = { 0x0010, 2, 2, 0, "PD_PTRP_BIAS", "Power down signal for PTAT/RP" };
static const struct LMS8Parameter PD_PT_BIAS = { 0x0010, 1, 1, 0, "PD_PT_BIAS", "Power down signal for PTAT" };
static const struct LMS8Parameter PD_BIAS = { 0x0010, 0, 0, 0, "PD_BIAS", "Power down signal for central bias block" };

static const struct LMS8Parameter EN_LDO_LOBUFA = { 0x0011, 8, 8, 0, "EN_LDO_LOBUFA", "Enables the LOBUF LDO for Channel A" };
static const struct LMS8Parameter EN_LDO_LOBUFB = { 0x0012, 8, 8, 0, "EN_LDO_LOBUFB", "Enables the LOBUF LDO for Channel B" };
static const struct LMS8Parameter EN_LDO_LOBUFC = { 0x0013, 8, 8, 0, "EN_LDO_LOBUFC", "Enables the LOBUF LDO for Channel C" };
static const struct LMS8Parameter EN_LDO_LOBUFD = { 0x0014, 8, 8, 0, "EN_LDO_LOBUFD", "Enables the LOBUF LDO for Channel D" };

static const struct LMS8Parameter EN_LDO_HFLNAA = { 0x0015, 8, 8, 0, "EN_LDO_HFLNAA", "Enables the LNA LDO for Channel A" };
static const struct LMS8Parameter EN_LDO_HFLNAB = { 0x0016, 8, 8, 0, "EN_LDO_HFLNAB", "Enables the LNA LDO for Channel B" };
static const struct LMS8Parameter EN_LDO_HFLNAC = { 0x0017, 8, 8, 0, "EN_LDO_HFLNAC", "Enables the LNA LDO for Channel C" };
static const struct LMS8Parameter EN_LDO_HFLNAD = { 0x0018, 8, 8, 0, "EN_LDO_HFLNAD", "Enables the LNA LDO for Channel D" };

static const struct LMS8Parameter EN_LDO_CLK_BUF = { 0x001A, 8, 8, 0, "EN_LDO_CLK_BUF", "Enables the LDO CLK" };
static const struct LMS8Parameter EN_LDO_PLL_DIV = { 0x001B, 8, 8, 0, "EN_LDO_PLL_DIV", "Enables the LDO PLL DIV" };
static const struct LMS8Parameter EN_LDO_PLL_CP = { 0x001C, 8, 8, 0, "EN_LDO_PLL_CP", "Enables the LDO PLL CP" };
static const struct LMS8Parameter PD_LDO_DIG_CORE = { 0x001F, 8, 8, 0, "PD_LDO_DIG_CORE", "Enables the LDO DIG CORE" };

static const struct LMS8Parameter SPDUP_LDO_LOBUFA = {
    0x0011, 9, 9, 0, "SPDUP_LDO_LOBUFA", "Short the noise filter resistor to speed up the settling time - LDO LOBUF Channel A"
};
static const struct LMS8Parameter SPDUP_LDO_LOBUFB = {
    0x0012, 9, 9, 0, "SPDUP_LDO_LOBUFB", "Short the noise filter resistor to speed up the settling time - LDO LOBUF Channel B"
};
static const struct LMS8Parameter SPDUP_LDO_LOBUFC = {
    0x0013, 9, 9, 0, "SPDUP_LDO_LOBUFC", "Short the noise filter resistor to speed up the settling time - LDO LOBUF Channel C"
};
static const struct LMS8Parameter SPDUP_LDO_LOBUFD = {
    0x0014, 9, 9, 0, "SPDUP_LDO_LOBUFD", "Short the noise filter resistor to speed up the settling time - LDO LOBUF Channel D"
};

static const struct LMS8Parameter SPDUP_LDO_HFLNAA = {
    0x0015, 9, 9, 0, "SPDUP_LDO_HFLNAA", "Short the noise filter resistor to speed up the settling time - LDO LNA Channel A"
};
static const struct LMS8Parameter SPDUP_LDO_HFLNAB = {
    0x0016, 9, 9, 0, "SPDUP_LDO_HFLNAB", "Short the noise filter resistor to speed up the settling time - LDO LNA Channel B"
};
static const struct LMS8Parameter SPDUP_LDO_HFLNAC = {
    0x0017, 9, 9, 0, "SPDUP_LDO_HFLNAC", "Short the noise filter resistor to speed up the settling time - LDO LNA Channel C"
};
static const struct LMS8Parameter SPDUP_LDO_HFLNAD = {
    0x0018, 9, 9, 0, "SPDUP_LDO_HFLNAD", "Short the noise filter resistor to speed up the settling time - LDO LNA Channel D"
};

static const struct LMS8Parameter SPDUP_LDO_CLK_BUF = {
    0x001A, 9, 9, 0, "SPDUP_LDO_CLK_BUF", "Short the noise filter resistor to speed up the settling time - LDO CLK"
};
static const struct LMS8Parameter SPDUP_LDO_PLL_DIV = {
    0x001B, 9, 9, 0, "SPDUP_LDO_PLL_DIV", "Short the noise filter resistor to speed up the settling time - LDO PLL DIV"
};
static const struct LMS8Parameter SPDUP_LDO_PLL_CP = {
    0x001C, 9, 9, 0, "SPDUP_LDO_PLL_CP", "Short the noise filter resistor to speed up the settling time - LDO PLL CP"
};
static const struct LMS8Parameter SPDUP_LDO_DIG_CORE = {
    0x001F, 9, 9, 0, "SPDUP_LDO_DIG_CORE", "Short the noise filter resistor to speed up the settling time - LDO DIG"
};

static const struct LMS8Parameter EN_LOADIMP_LDO_LOBUFA = { 0x0011,
    10,
    10,
    0,
    "EN_LOADIMP_LDO_LOBUFA",
    "Enables the load dependent bias to optimize the load regulation - LDO LOBUF Channel A" };
static const struct LMS8Parameter EN_LOADIMP_LDO_LOBUFB = { 0x0012,
    10,
    10,
    0,
    "EN_LOADIMP_LDO_LOBUFB",
    "Enables the load dependent bias to optimize the load regulation - LDO LOBUF Channel B" };
static const struct LMS8Parameter EN_LOADIMP_LDO_LOBUFC = { 0x0013,
    10,
    10,
    0,
    "EN_LOADIMP_LDO_LOBUFC",
    "Enables the load dependent bias to optimize the load regulation - LDO LOBUF Channel C" };
static const struct LMS8Parameter EN_LOADIMP_LDO_LOBUFD = { 0x0014,
    10,
    10,
    0,
    "EN_LOADIMP_LDO_LOBUFD",
    "Enables the load dependent bias to optimize the load regulation - LDO LOBUF Channel D" };

static const struct LMS8Parameter EN_LOADIMP_LDO_HFLNAA = { 0x0015,
    10,
    10,
    0,
    "EN_LOADIMP_LDO_HFLNAA",
    "Enables the load dependent bias to optimize the load regulation - LDO LNA Channel A" };
static const struct LMS8Parameter EN_LOADIMP_LDO_HFLNAB = { 0x0016,
    10,
    10,
    0,
    "EN_LOADIMP_LDO_HFLNAB",
    "Enables the load dependent bias to optimize the load regulation - LDO LNA Channel B" };
static const struct LMS8Parameter EN_LOADIMP_LDO_HFLNAC = { 0x0017,
    10,
    10,
    0,
    "EN_LOADIMP_LDO_HFLNAC",
    "Enables the load dependent bias to optimize the load regulation - LDO LNA Channel C" };
static const struct LMS8Parameter EN_LOADIMP_LDO_HFLNAD = { 0x0018,
    10,
    10,
    0,
    "EN_LOADIMP_LDO_HFLNAD",
    "Enables the load dependent bias to optimize the load regulation - LDO LNA Channel D" };

static const struct LMS8Parameter EN_LOADIMP_LDO_CLK_BUF = {
    0x001A, 10, 10, 0, "EN_LOADIMP_LDO_CLK_BUF", "Enables the load dependent bias to optimize the load regulation - LDO CLK"
};
static const struct LMS8Parameter EN_LOADIMP_LDO_PLL_DIV = {
    0x001B, 10, 10, 0, "EN_LOADIMP_LDO_PLL_DIV", "Enables the load dependent bias to optimize the load regulation - LDO LPLL DIV"
};
static const struct LMS8Parameter EN_LOADIMP_LDO_PLL_CP = {
    0x001C, 10, 10, 0, "EN_LOADIMP_LDO_PLL_CP", "Enables the load dependent bias to optimize the load regulation - LDO PLL CP"
};
static const struct LMS8Parameter EN_LOADIMP_LDO_DIG_CORE = {
    0x001F, 10, 10, 0, "EN_LOADIMP_LDO_DIG_CORE", "Enables the load dependent bias to optimize the load regulation - LDO DIG CORE"
};

static const struct LMS8Parameter RDIV_LOBUFA = {
    0x0011, 7, 0, 101, "RDIV_LOBUFA", "Controls the output voltage - LDO LOBUF Channel A"
};
static const struct LMS8Parameter RDIV_LOBUFB = {
    0x0012, 7, 0, 101, "RDIV_LOBUFB", "Controls the output voltage - LDO LOBUF Channel B"
};
static const struct LMS8Parameter RDIV_LOBUFC = {
    0x0013, 7, 0, 101, "RDIV_LOBUFC", "Controls the output voltage - LDO LOBUF Channel C"
};
static const struct LMS8Parameter RDIV_LOBUFD = {
    0x0014, 7, 0, 101, "RDIV_LOBUFD", "Controls the output voltage - LDO LOBUF Channel D"
};

static const struct LMS8Parameter RDIV_HFLNAA = {
    0x0015, 7, 0, 101, "RDIV_HFLNAA", "Controls the output voltage - LDO LNA Channel A"
};
static const struct LMS8Parameter RDIV_HFLNAB = {
    0x0016, 7, 0, 101, "RDIV_HFLNAB", "Controls the output voltage - LDO LNA Channel B"
};
static const struct LMS8Parameter RDIV_HFLNAC = {
    0x0017, 7, 0, 101, "RDIV_HFLNAC", "Controls the output voltage - LDO LNA Channel C"
};
static const struct LMS8Parameter RDIV_HFLNAD = {
    0x0018, 7, 0, 101, "RDIV_HFLNAD", "Controls the output voltage - LDO LNA Channel D"
};

static const struct LMS8Parameter RDIV_CLK_BUF = { 0x001A, 7, 0, 101, "RDIV_CLK_BUF", "Controls the output voltage - LDO CLK" };
static const struct LMS8Parameter RDIV_PLL_DIV = { 0x001B, 7, 0, 101, "RDIV_PLL_DIV", "Controls the output voltage - LDO PLL DIV" };
static const struct LMS8Parameter RDIV_PLL_CP = { 0x001C, 7, 0, 101, "RDIV_PLL_CP", "Controls the output voltage - LDO PLL CP" };
static const struct LMS8Parameter RDIV_DIG_CORE = {
    0x001F, 7, 0, 101, "RDIV_DIG_CORE", "Controls the output voltage - LDO DIG CORE"
};

static const struct LMS8Parameter CHx_MIXB_ICT = {
    0x1000, 9, 5, 16, "CHx_MIXA_ICT", "Controls the bias current of polarization circuit for MIXA"
};
static const struct LMS8Parameter CHx_MIXA_ICT = {
    0x1000, 4, 0, 16, "CHx_MIXB_ICT", "Controls the bias current of polarization circuit for MIXB"
};

static const struct LMS8Parameter CHx_PA_ILIN2X = { 0x1001, 10, 10, 0, "CHx_PA_ILIN2X", "Double the linearization bias current" };
static const struct LMS8Parameter CHx_PA_ICT_LIN = {
    0x1001, 9, 5, 16, "CHx_PA_ICT_LIN", "Controls the bias current of linearization section of HFPAD"
};
static const struct LMS8Parameter CHx_PA_ICT_MAIN = {
    0x1001, 4, 0, 16, "CHx_PA_ICT_MAIN", "Controls the bias current of main gm section of HFPAD"
};

static const struct LMS8Parameter CHx_PA_R50_EN0 = {
    0x1004, 7, 7, 1, "CHx_PA_R50_EN0", "Controls the switch in series with 50 Ohm resistor to ground at HFPAD input"
};
static const struct LMS8Parameter CHx_PA_BYPASS0 = { 0x1004, 6, 6, 0, "CHx_PA_BYPASS0", "Controls the HFPAD bypass switches" };
static const struct LMS8Parameter CHx_PA_PD0 = { 0x1004, 5, 5, 1, "CHx_PA_PD0", "Power down for HFPAD" };
static const struct LMS8Parameter CHx_MIXB_LOBUFF_PD0 = { 0x1004, 4, 4, 1, "CHx_MIXB_LOBUFF_PD0", "Power down for MIXB LO buffer" };
static const struct LMS8Parameter CHx_MIXB_BIAS_PD0 = { 0x1004, 3, 3, 1, "CHx_MIXB_BIAS_PD0", "Power down for MIXB bias" };
static const struct LMS8Parameter CHx_MIXA_LOBUFF_PD0 = { 0x1004, 2, 2, 1, "CHx_MIXA_LOBUFF_PD0", "Power down for MIXA LO buffer" };
static const struct LMS8Parameter CHx_MIXA_BIAS_PD0 = { 0x1004, 1, 1, 1, "CHx_MIXA_BIAS_PD0", "Power down for MIXA bias" };
static const struct LMS8Parameter CHx_LNA_PD0 = { 0x1004, 0, 0, 1, "CHx_LNA_PD0", "Power down for LNA" };

static const struct LMS8Parameter CHx_LNA_ICT_LIN0 = {
    0x1008, 15, 11, 16, "CHx_LNA_ICT_LIN0", "Controls the bias current of linearization section of LNA"
};
static const struct LMS8Parameter CHx_LNA_ICT_MAIN0 = {
    0x1008, 10, 6, 16, "CHx_LNA_ICT_MAIN0", "Controls the bias current of main gm section of LNA"
};
static const struct LMS8Parameter CHx_LNA_CGSCTRL0 = {
    0x1008, 5, 4, 2, "CHx_LNA_CGSCTRL0", "Controls the additional LNA input device gate-source capacitance"
};
static const struct LMS8Parameter CHx_LNA_GCTRL0 = { 0x1008, 3, 0, 8, "CHx_LNA_GCTRL0", "Controls the LNA gain" };

static const struct LMS8Parameter CHx_PA_LIN_LOSS0 = {
    0x100C, 7, 4, 0, "CHx_PA_LIN_LOSS0", "Controls the gain of HFPAD linearizing section"
};
static const struct LMS8Parameter CHx_PA_MAIN_LOSS0 = {
    0x100C, 3, 0, 0, "CHx_PA_MAIN_LOSS0", "Controls the gain of HFPAD main section"
};

static const struct LMS8Parameter CHx_PA_R50_EN1 = {
    0x1005, 7, 7, 1, "CHx_PA_R50_EN1", "Controls the switch in series with 50 Ohm resistor to ground at HFPAD input"
};
static const struct LMS8Parameter CHx_PA_BYPASS1 = { 0x1005, 6, 6, 0, "CHx_PA_BYPASS1", "Controls the HFPAD bypass switches" };
static const struct LMS8Parameter CHx_PA_PD1 = { 0x1005, 5, 5, 1, "CHx_PA_PD1", "Power down for HFPAD" };
static const struct LMS8Parameter CHx_MIXB_LOBUFF_PD1 = { 0x1005, 4, 4, 1, "CHx_MIXB_LOBUFF_PD1", "Power down for MIXB LO buffer" };
static const struct LMS8Parameter CHx_MIXB_BIAS_PD1 = { 0x1005, 3, 3, 1, "CHx_MIXB_BIAS_PD1", "Power down for MIXB bias" };
static const struct LMS8Parameter CHx_MIXA_LOBUFF_PD1 = { 0x1005, 2, 2, 1, "CHx_MIXA_LOBUFF_PD1", "Power down for MIXA LO buffer" };
static const struct LMS8Parameter CHx_MIXA_BIAS_PD1 = { 0x1005, 1, 1, 1, "CHx_MIXA_BIAS_PD1", "Power down for MIXA bias" };
static const struct LMS8Parameter CHx_LNA_PD1 = { 0x1005, 0, 0, 1, "CHx_LNA_PD1", "Power down for LNA" };

static const struct LMS8Parameter CHx_LNA_ICT_LIN1 = {
    0x1009, 15, 11, 16, "CHx_LNA_ICT_LIN1", "Controls the bias current of linearization section of LNA"
};
static const struct LMS8Parameter CHx_LNA_ICT_MAIN1 = {
    0x1009, 10, 6, 16, "CHx_LNA_ICT_MAIN1", "Controls the bias current of main gm section of LNA"
};
static const struct LMS8Parameter CHx_LNA_CGSCTRL1 = {
    0x1009, 5, 4, 2, "CHx_LNA_CGSCTRL1", "Controls the additional LNA input device gate-source capacitance"
};
static const struct LMS8Parameter CHx_LNA_GCTRL1 = { 0x1009, 3, 0, 8, "CHx_LNA_GCTRL1", "Controls the LNA gain" };

static const struct LMS8Parameter CHx_PA_LIN_LOSS1 = {
    0x100D, 7, 4, 0, "CHx_PA_LIN_LOSS1", "Controls the gain of HFPAD linearizing section"
};
static const struct LMS8Parameter CHx_PA_MAIN_LOSS1 = {
    0x100D, 3, 0, 0, "CHx_PA_MAIN_LOSS1", "Controls the gain of HFPAD main section"
};

static const struct LMS8Parameter CHx_PA_R50_EN2 = {
    0x1006, 7, 7, 1, "CHx_PA_R50_EN2", "Controls the switch in series with 50 Ohm resistor to ground at HFPAD input"
};
static const struct LMS8Parameter CHx_PA_BYPASS2 = { 0x1006, 6, 6, 0, "CHx_PA_BYPASS2", "Controls the HFPAD bypass switches" };
static const struct LMS8Parameter CHx_PA_PD2 = { 0x1006, 5, 5, 1, "CHx_PA_PD2", "Power down for HFPAD" };
static const struct LMS8Parameter CHx_MIXB_LOBUFF_PD2 = { 0x1006, 4, 4, 1, "CHx_MIXB_LOBUFF_PD2", "Power down for MIXB LO buffer" };
static const struct LMS8Parameter CHx_MIXB_BIAS_PD2 = { 0x1006, 3, 3, 1, "CHx_MIXB_BIAS_PD2", "Power down for MIXB bias" };
static const struct LMS8Parameter CHx_MIXA_LOBUFF_PD2 = { 0x1006, 2, 2, 1, "CHx_MIXA_LOBUFF_PD2", "Power down for MIXA LO buffer" };
static const struct LMS8Parameter CHx_MIXA_BIAS_PD2 = { 0x1006, 1, 1, 1, "CHx_MIXA_BIAS_PD2", "Power down for MIXA bias" };
static const struct LMS8Parameter CHx_LNA_PD2 = { 0x1006, 0, 0, 1, "CHx_LNA_PD2", "Power down for LNA" };

static const struct LMS8Parameter CHx_LNA_ICT_LIN2 = {
    0x100A, 15, 11, 16, "CHx_LNA_ICT_LIN2", "Controls the bias current of linearization section of LNA"
};
static const struct LMS8Parameter CHx_LNA_ICT_MAIN2 = {
    0x100A, 10, 6, 16, "CHx_LNA_ICT_MAIN2", "Controls the bias current of main gm section of LNA"
};
static const struct LMS8Parameter CHx_LNA_CGSCTRL2 = {
    0x100A, 5, 4, 2, "CHx_LNA_CGSCTRL2", "Controls the additional LNA input device gate-source capacitance"
};
static const struct LMS8Parameter CHx_LNA_GCTRL2 = { 0x100A, 3, 0, 8, "CHx_LNA_GCTRL2", "Controls the LNA gain" };

static const struct LMS8Parameter CHx_PA_LIN_LOSS2 = {
    0x100E, 7, 4, 0, "CHx_PA_LIN_LOSS2", "Controls the gain of HFPAD linearizing section"
};
static const struct LMS8Parameter CHx_PA_MAIN_LOSS2 = {
    0x100E, 3, 0, 0, "CHx_PA_MAIN_LOSS2", "Controls the gain of HFPAD main section"
};

static const struct LMS8Parameter CHx_PA_R50_EN3 = {
    0x1007, 7, 7, 1, "CHx_PA_R50_EN3", "Controls the switch in series with 50 Ohm resistor to ground at HFPAD input"
};
static const struct LMS8Parameter CHx_PA_BYPASS3 = { 0x1007, 6, 6, 0, "CHx_PA_BYPASS3", "Controls the HFPAD bypass switches" };
static const struct LMS8Parameter CHx_PA_PD3 = { 0x1007, 5, 5, 1, "CHx_PA_PD3", "Power down for HFPAD" };
static const struct LMS8Parameter CHx_MIXB_LOBUFF_PD3 = { 0x1007, 4, 4, 1, "CHx_MIXB_LOBUFF_PD3", "Power down for MIXB LO buffer" };
static const struct LMS8Parameter CHx_MIXB_BIAS_PD3 = { 0x1007, 3, 3, 1, "CHx_MIXB_BIAS_PD3", "Power down for MIXB bias" };
static const struct LMS8Parameter CHx_MIXA_LOBUFF_PD3 = { 0x1007, 2, 2, 1, "CHx_MIXA_LOBUFF_PD3", "Power down for MIXA LO buffer" };
static const struct LMS8Parameter CHx_MIXA_BIAS_PD3 = { 0x1007, 1, 1, 1, "CHx_MIXA_BIAS_PD3", "Power down for MIXA bias" };
static const struct LMS8Parameter CHx_LNA_PD3 = { 0x1007, 0, 0, 1, "CHx_LNA_PD3", "Power down for LNA" };

static const struct LMS8Parameter CHx_LNA_ICT_LIN3 = {
    0x100B, 15, 11, 16, "CHx_LNA_ICT_LIN3", "Controls the bias current of linearization section of LNA"
};
static const struct LMS8Parameter CHx_LNA_ICT_MAIN3 = {
    0x100B, 10, 6, 16, "CHx_LNA_ICT_MAIN3", "Controls the bias current of main gm section of LNA"
};
static const struct LMS8Parameter CHx_LNA_CGSCTRL3 = {
    0x100B, 5, 4, 2, "CHx_LNA_CGSCTRL3", "Controls the additional LNA input device gate-source capacitance"
};
static const struct LMS8Parameter CHx_LNA_GCTRL3 = { 0x100B, 3, 0, 8, "CHx_LNA_GCTRL3", "Controls the LNA gain" };

static const struct LMS8Parameter CHx_PA_LIN_LOSS3 = {
    0x100F, 7, 4, 0, "CHx_PA_LIN_LOSS3", "Controls the gain of HFPAD linearizing section"
};
static const struct LMS8Parameter CHx_PA_MAIN_LOSS3 = {
    0x100F, 3, 0, 0, "CHx_PA_MAIN_LOSS3", "Controls the gain of HFPAD main section"
};

static const struct LMS8Parameter CHx_PA_R50_EN_RB = {
    0x101D, 7, 7, 1, "CHx_PA_R50_EN_RB", "Controls the switch in series with 50 Ohm resistor to ground at HFPAD input - Readback"
};
static const struct LMS8Parameter CHx_PA_BYPASS_RB = {
    0x101D, 6, 6, 0, "CHx_PA_BYPASS3_RB", "Controls the HFPAD bypass switches - Readback"
};
static const struct LMS8Parameter CHx_PA_PD_RB = { 0x101D, 5, 5, 1, "CHx_PA_PD_RB", "Power down for HFPAD - Readback" };
static const struct LMS8Parameter CHx_MIXB_LOBUFF_PD_RB = {
    0x101D, 4, 4, 1, "CHx_MIXB_LOBUFF_PD_RB", "Power down for MIXB LO buffer - Readback"
};
static const struct LMS8Parameter CHx_MIXB_BIAS_PD_RB = {
    0x101D, 3, 3, 1, "CHx_MIXB_BIAS_PD_RB", "Power down for MIXB bias - Readback"
};
static const struct LMS8Parameter CHx_MIXA_LOBUFF_PD_RB = {
    0x101D, 2, 2, 1, "CHx_MIXA_LOBUFF_PD_RB", "Power down for MIXA LO buffer - Readback"
};
static const struct LMS8Parameter CHx_MIXA_BIAS_PD_RB = {
    0x101D, 1, 1, 1, "CHx_MIXA_BIAS_PD_RB", "Power down for MIXA bias - Readback"
};
static const struct LMS8Parameter CHx_LNA_PD_RB = { 0x101D, 0, 0, 1, "CHx_LNA_PD_RB", "Power down for LNA - Readback" };

static const struct LMS8Parameter CHx_LNA_ICT_LIN_RB = {
    0x101E, 15, 11, 16, "CHx_LNA_ICT_LIN_RB", "Controls the bias current of linearization section of LNA - Readback"
};
static const struct LMS8Parameter CHx_LNA_ICT_MAIN_RB = {
    0x101E, 10, 6, 16, "CHx_LNA_ICT_MAIN_RB", "Controls the bias current of main gm section of LNA - Readback"
};
static const struct LMS8Parameter CHx_LNA_CGSCTRL_RB = {
    0x101E, 5, 4, 2, "CHx_LNA_CGSCTRL_RB", "Controls the additional LNA input device gate-source capacitance - Readback"
};
static const struct LMS8Parameter CHx_LNA_GCTRL_RB = { 0x101E, 3, 0, 8, "CHx_LNA_GCTRL_RB", "Controls the LNA gain - Readback" };

static const struct LMS8Parameter CHx_PA_LIN_LOSS_RB = {
    0x101F, 7, 4, 0, "CHx_PA_LIN_LOSS_RB", "Controls the gain of HFPAD linearizing section - Readback"
};
static const struct LMS8Parameter CHx_PA_MAIN_LOSS_RB = {
    0x101F, 3, 0, 0, "CHx_PA_MAIN_LOSS_RB", "Controls the gain of HFPAD main section - Readback"
};

static const struct LMS8Parameter CHx_PD_SEL0_INTERNAL = {
    0x1010, 11, 11, 1, "CHx_PD_SEL0_INTERNAL", "PD control signals multiplexer SEL0 signal is internal"
};
static const struct LMS8Parameter CHx_PD_SEL0_INVERT = {
    0x1010, 10, 10, 0, "CHx_PD_SEL0_INVERT", "Invert the SEL0 signal of PD control signals"
};
static const struct LMS8Parameter CHx_PD_SEL0_MASK0 = {
    0x1010, 0, 0, 0, "CHx_PD_SEL0_MASK0", "GPIO0 mask for SEL0 signal of PD control signals multiplexer"
};
static const struct LMS8Parameter CHx_PD_SEL0_MASK1 = {
    0x1010, 1, 1, 0, "CHx_PD_SEL0_MASK1", "GPIO1 mask for SEL0 signal of PD control signals multiplexer"
};
static const struct LMS8Parameter CHx_PD_SEL0_MASK2 = {
    0x1010, 2, 2, 0, "CHx_PD_SEL0_MASK2", "GPIO2 mask for SEL0 signal of PD control signals multiplexer"
};
static const struct LMS8Parameter CHx_PD_SEL0_MASK3 = {
    0x1010, 3, 3, 0, "CHx_PD_SEL0_MASK3", "GPIO3 mask for SEL0 signal of PD control signals multiplexer"
};
static const struct LMS8Parameter CHx_PD_SEL0_MASK4 = {
    0x1010, 4, 4, 0, "CHx_PD_SEL0_MASK4", "GPIO4 mask for SEL0 signal of PD control signals multiplexer"
};
static const struct LMS8Parameter CHx_PD_SEL0_MASK5 = {
    0x1010, 5, 5, 0, "CHx_PD_SEL0_MASK5", "GPIO5 mask for SEL0 signal of PD control signals multiplexer"
};
static const struct LMS8Parameter CHx_PD_SEL0_MASK6 = {
    0x1010, 6, 6, 0, "CHx_PD_SEL0_MASK6", "GPIO6 mask for SEL0 signal of PD control signals multiplexer"
};
static const struct LMS8Parameter CHx_PD_SEL0_MASK7 = {
    0x1010, 7, 7, 0, "CHx_PD_SEL0_MASK7", "GPIO7 mask for SEL0 signal of PD control signals multiplexer"
};
static const struct LMS8Parameter CHx_PD_SEL0_MASK8 = {
    0x1010, 8, 8, 0, "CHx_PD_SEL0_MASK8", "GPIO8 mask for SEL0 signal of PD control signals multiplexer"
};

static const struct LMS8Parameter CHx_PD_SEL1_INTERNAL = {
    0x1011, 11, 11, 1, "CHx_PD_SEL1_INTERNAL", "PD control signals multiplexer SEL1 signal is internal"
};
static const struct LMS8Parameter CHx_PD_SEL1_INVERT = {
    0x1011, 10, 10, 0, "CHx_PD_SEL1_INVERT", "Invert the SEL1 signal of PD control signals"
};
static const struct LMS8Parameter CHx_PD_SEL1_MASK0 = {
    0x1011, 0, 0, 0, "CHx_PD_SEL1_MASK0", "GPIO0 mask for SEL1 signal of PD control signals multiplexer"
};
static const struct LMS8Parameter CHx_PD_SEL1_MASK1 = {
    0x1011, 1, 1, 0, "CHx_PD_SEL1_MASK1", "GPIO1 mask for SEL1 signal of PD control signals multiplexer"
};
static const struct LMS8Parameter CHx_PD_SEL1_MASK2 = {
    0x1011, 2, 2, 0, "CHx_PD_SEL1_MASK2", "GPIO2 mask for SEL1 signal of PD control signals multiplexer"
};
static const struct LMS8Parameter CHx_PD_SEL1_MASK3 = {
    0x1011, 3, 3, 0, "CHx_PD_SEL1_MASK3", "GPIO3 mask for SEL1 signal of PD control signals multiplexer"
};
static const struct LMS8Parameter CHx_PD_SEL1_MASK4 = {
    0x1011, 4, 4, 0, "CHx_PD_SEL1_MASK4", "GPIO4 mask for SEL1 signal of PD control signals multiplexer"
};
static const struct LMS8Parameter CHx_PD_SEL1_MASK5 = {
    0x1011, 5, 5, 0, "CHx_PD_SEL1_MASK5", "GPIO5 mask for SEL1 signal of PD control signals multiplexer"
};
static const struct LMS8Parameter CHx_PD_SEL1_MASK6 = {
    0x1011, 6, 6, 0, "CHx_PD_SEL1_MASK6", "GPIO6 mask for SEL1 signal of PD control signals multiplexer"
};
static const struct LMS8Parameter CHx_PD_SEL1_MASK7 = {
    0x1011, 7, 7, 0, "CHx_PD_SEL1_MASK7", "GPIO7 mask for SEL1 signal of PD control signals multiplexer"
};
static const struct LMS8Parameter CHx_PD_SEL1_MASK8 = {
    0x1011, 8, 8, 0, "CHx_PD_SEL1_MASK8", "GPIO8 mask for SEL1 signal of PD control signals multiplexer"
};

static const struct LMS8Parameter CHx_LNA_SEL0_INTERNAL = {
    0x1012, 11, 11, 1, "CHx_LNA_SEL0_INTERNAL", "PD control signals multiplexer SEL0 signal is internal"
};
static const struct LMS8Parameter CHx_LNA_SEL0_INVERT = {
    0x1012, 10, 10, 0, "CHx_LNA_SEL0_INVERT", "Invert the SEL0 signal of PD control signals"
};
static const struct LMS8Parameter CHx_LNA_SEL0_MASK0 = {
    0x1012, 0, 0, 0, "CHx_LNA_SEL0_MASK0", "GPIO0 mask for SEL0 signal of PD control signals multiplexer"
};
static const struct LMS8Parameter CHx_LNA_SEL0_MASK1 = {
    0x1012, 1, 1, 0, "CHx_LNA_SEL0_MASK1", "GPIO1 mask for SEL0 signal of PD control signals multiplexer"
};
static const struct LMS8Parameter CHx_LNA_SEL0_MASK2 = {
    0x1012, 2, 2, 0, "CHx_LNA_SEL0_MASK2", "GPIO2 mask for SEL0 signal of PD control signals multiplexer"
};
static const struct LMS8Parameter CHx_LNA_SEL0_MASK3 = {
    0x1012, 3, 3, 0, "CHx_LNA_SEL0_MASK3", "GPIO3 mask for SEL0 signal of PD control signals multiplexer"
};
static const struct LMS8Parameter CHx_LNA_SEL0_MASK4 = {
    0x1012, 4, 4, 0, "CHx_LNA_SEL0_MASK4", "GPIO4 mask for SEL0 signal of PD control signals multiplexer"
};
static const struct LMS8Parameter CHx_LNA_SEL0_MASK5 = {
    0x1012, 5, 5, 0, "CHx_LNA_SEL0_MASK5", "GPIO5 mask for SEL0 signal of PD control signals multiplexer"
};
static const struct LMS8Parameter CHx_LNA_SEL0_MASK6 = {
    0x1012, 6, 6, 0, "CHx_LNA_SEL0_MASK6", "GPIO6 mask for SEL0 signal of PD control signals multiplexer"
};
static const struct LMS8Parameter CHx_LNA_SEL0_MASK7 = {
    0x1012, 7, 7, 0, "CHx_LNA_SEL0_MASK7", "GPIO7 mask for SEL0 signal of PD control signals multiplexer"
};
static const struct LMS8Parameter CHx_LNA_SEL0_MASK8 = {
    0x1012, 8, 8, 0, "CHx_LNA_SEL0_MASK8", "GPIO8 mask for SEL0 signal of PD control signals multiplexer"
};

static const struct LMS8Parameter CHx_LNA_SEL1_INTERNAL = {
    0x1013, 11, 11, 1, "CHx_LNA_SEL1_INTERNAL", "PD control signals multiplexer SEL1 signal is internal"
};
static const struct LMS8Parameter CHx_LNA_SEL1_INVERT = {
    0x1013, 10, 10, 0, "CHx_LNA_SEL1_INVERT", "Invert the SEL1 signal of PD control signals"
};
static const struct LMS8Parameter CHx_LNA_SEL1_MASK0 = {
    0x1013, 0, 0, 0, "CHx_LNA_SEL1_MASK0", "GPIO0 mask for SEL1 signal of PD control signals multiplexer"
};
static const struct LMS8Parameter CHx_LNA_SEL1_MASK1 = {
    0x1013, 1, 1, 0, "CHx_LNA_SEL1_MASK1", "GPIO1 mask for SEL1 signal of PD control signals multiplexer"
};
static const struct LMS8Parameter CHx_LNA_SEL1_MASK2 = {
    0x1013, 2, 2, 0, "CHx_LNA_SEL1_MASK2", "GPIO2 mask for SEL1 signal of PD control signals multiplexer"
};
static const struct LMS8Parameter CHx_LNA_SEL1_MASK3 = {
    0x1013, 3, 3, 0, "CHx_LNA_SEL1_MASK3", "GPIO3 mask for SEL1 signal of PD control signals multiplexer"
};
static const struct LMS8Parameter CHx_LNA_SEL1_MASK4 = {
    0x1013, 4, 4, 0, "CHx_LNA_SEL1_MASK4", "GPIO4 mask for SEL1 signal of PD control signals multiplexer"
};
static const struct LMS8Parameter CHx_LNA_SEL1_MASK5 = {
    0x1013, 5, 5, 0, "CHx_LNA_SEL1_MASK5", "GPIO5 mask for SEL1 signal of PD control signals multiplexer"
};
static const struct LMS8Parameter CHx_LNA_SEL1_MASK6 = {
    0x1013, 6, 6, 0, "CHx_LNA_SEL1_MASK6", "GPIO6 mask for SEL1 signal of PD control signals multiplexer"
};
static const struct LMS8Parameter CHx_LNA_SEL1_MASK7 = {
    0x1013, 7, 7, 0, "CHx_LNA_SEL1_MASK7", "GPIO7 mask for SEL1 signal of PD control signals multiplexer"
};
static const struct LMS8Parameter CHx_LNA_SEL1_MASK8 = {
    0x1013, 8, 8, 0, "CHx_LNA_SEL1_MASK8", "GPIO8 mask for SEL1 signal of PD control signals multiplexer"
};

static const struct LMS8Parameter CHx_PA_SEL0_INTERNAL = {
    0x1014, 11, 11, 1, "CHx_PA_SEL0_INTERNAL", "PD control signals multiplexer SEL0 signal is internal"
};
static const struct LMS8Parameter CHx_PA_SEL0_INVERT = {
    0x1014, 10, 10, 0, "CHx_PA_SEL0_INVERT", "Invert the SEL0 signal of PD control signals"
};
static const struct LMS8Parameter CHx_PA_SEL0_MASK0 = {
    0x1014, 0, 0, 0, "CHx_PA_SEL0_MASK0", "GPIO0 mask for SEL0 signal of PD control signals multiplexer"
};
static const struct LMS8Parameter CHx_PA_SEL0_MASK1 = {
    0x1014, 1, 1, 0, "CHx_PA_SEL0_MASK1", "GPIO1 mask for SEL0 signal of PD control signals multiplexer"
};
static const struct LMS8Parameter CHx_PA_SEL0_MASK2 = {
    0x1014, 2, 2, 0, "CHx_PA_SEL0_MASK2", "GPIO2 mask for SEL0 signal of PD control signals multiplexer"
};
static const struct LMS8Parameter CHx_PA_SEL0_MASK3 = {
    0x1014, 3, 3, 0, "CHx_PA_SEL0_MASK3", "GPIO3 mask for SEL0 signal of PD control signals multiplexer"
};
static const struct LMS8Parameter CHx_PA_SEL0_MASK4 = {
    0x1014, 4, 4, 0, "CHx_PA_SEL0_MASK4", "GPIO4 mask for SEL0 signal of PD control signals multiplexer"
};
static const struct LMS8Parameter CHx_PA_SEL0_MASK5 = {
    0x1014, 5, 5, 0, "CHx_PA_SEL0_MASK5", "GPIO5 mask for SEL0 signal of PD control signals multiplexer"
};
static const struct LMS8Parameter CHx_PA_SEL0_MASK6 = {
    0x1014, 6, 6, 0, "CHx_PA_SEL0_MASK6", "GPIO6 mask for SEL0 signal of PD control signals multiplexer"
};
static const struct LMS8Parameter CHx_PA_SEL0_MASK7 = {
    0x1014, 7, 7, 0, "CHx_PA_SEL0_MASK7", "GPIO7 mask for SEL0 signal of PD control signals multiplexer"
};
static const struct LMS8Parameter CHx_PA_SEL0_MASK8 = {
    0x1014, 8, 8, 0, "CHx_PA_SEL0_MASK8", "GPIO8 mask for SEL0 signal of PD control signals multiplexer"
};

static const struct LMS8Parameter CHx_PA_SEL1_INTERNAL = {
    0x1015, 11, 11, 1, "CHx_PA_SEL1_INTERNAL", "PD control signals multiplexer SEL1 signal is internal"
};
static const struct LMS8Parameter CHx_PA_SEL1_INVERT = {
    0x1015, 10, 10, 0, "CHx_PA_SEL1_INVERT", "Invert the SEL1 signal of PD control signals"
};
static const struct LMS8Parameter CHx_PA_SEL1_MASK0 = {
    0x1015, 0, 0, 0, "CHx_PA_SEL1_MASK0", "GPIO0 mask for SEL1 signal of PD control signals multiplexer"
};
static const struct LMS8Parameter CHx_PA_SEL1_MASK1 = {
    0x1015, 1, 1, 0, "CHx_PA_SEL1_MASK1", "GPIO1 mask for SEL1 signal of PD control signals multiplexer"
};
static const struct LMS8Parameter CHx_PA_SEL1_MASK2 = {
    0x1015, 2, 2, 0, "CHx_PA_SEL1_MASK2", "GPIO2 mask for SEL1 signal of PD control signals multiplexer"
};
static const struct LMS8Parameter CHx_PA_SEL1_MASK3 = {
    0x1015, 3, 3, 0, "CHx_PA_SEL1_MASK3", "GPIO3 mask for SEL1 signal of PD control signals multiplexer"
};
static const struct LMS8Parameter CHx_PA_SEL1_MASK4 = {
    0x1015, 4, 4, 0, "CHx_PA_SEL1_MASK4", "GPIO4 mask for SEL1 signal of PD control signals multiplexer"
};
static const struct LMS8Parameter CHx_PA_SEL1_MASK5 = {
    0x1015, 5, 5, 0, "CHx_PA_SEL1_MASK5", "GPIO5 mask for SEL1 signal of PD control signals multiplexer"
};
static const struct LMS8Parameter CHx_PA_SEL1_MASK6 = {
    0x1015, 6, 6, 0, "CHx_PA_SEL1_MASK6", "GPIO6 mask for SEL1 signal of PD control signals multiplexer"
};
static const struct LMS8Parameter CHx_PA_SEL1_MASK7 = {
    0x1015, 7, 7, 0, "CHx_PA_SEL1_MASK7", "GPIO7 mask for SEL1 signal of PD control signals multiplexer"
};
static const struct LMS8Parameter CHx_PA_SEL1_MASK8 = {
    0x1015, 8, 8, 0, "CHx_PA_SEL1_MASK8", "GPIO8 mask for SEL1 signal of PD control signals multiplexer"
};

static const struct LMS8Parameter CHx_PD_INT_SEL0 = {
    0x1016, 0, 0, 0, "CHx_PD_INT_SEL0", "Internal value of PD control signals multiplexer selection signals - Bit 0"
};
static const struct LMS8Parameter CHx_PD_INT_SEL1 = {
    0x1016, 1, 1, 0, "CHx_PD_INT_SEL1", "Internal value of PD control signals multiplexer selection signals - Bit 1"
};

static const struct LMS8Parameter CHx_LNA_INT_SEL0 = {
    0x1016, 2, 2, 0, "CHx_LNA_INT_SEL0", "Internal value of LNA control signals multiplexer selection signals - Bit 0"
};
static const struct LMS8Parameter CHx_LNA_INT_SEL1 = {
    0x1016, 3, 3, 0, "CHx_LNA_INT_SEL1", "Internal value of LNA control signals multiplexer selection signals - Bit 1"
};

static const struct LMS8Parameter CHx_PA_INT_SEL0 = {
    0x1016, 4, 4, 0, "CHx_PA_INT_SEL0", "Internal value of PA control signals multiplexer selection signals - Bit 0"
};
static const struct LMS8Parameter CHx_PA_INT_SEL1 = {
    0x1016, 5, 5, 0, "CHx_PA_INT_SEL1", "Internal value of PA control signals multiplexer selection signals - Bit 1"
};

static const struct LMS8Parameter HLMIXx_VGCAS0 = { 0x2000, 13, 7, 64, "HLMIXx_VGCAS0", "HLMIX LO bias voltage" };
static const struct LMS8Parameter HLMIXx_ICT_BIAS0 = { 0x2000, 6, 2, 0, "HLMIXx_ICT_BIAS0", "HLMIX core bias current control" };
static const struct LMS8Parameter HLMIXx_BIAS_PD0 = { 0x2000, 1, 1, 0, "HLMIXx_BIAS_PD0", "HLMIX core bias control" };
static const struct LMS8Parameter HLMIXx_LOBUF_PD0 = { 0x2000, 0, 0, 0, "HLMIXx_LOBUF_PD0", "HLMIX LO buffer power down" };
static const struct LMS8Parameter HLMIXx_MIXLOSS0 = { 0x2004, 5, 2, 0, "HLMIXx_MIXLOSS0", "HLMIX loss control" };

static const struct LMS8Parameter HLMIXx_VGCAS1 = { 0x2001, 13, 7, 64, "HLMIXx_VGCAS1", "HLMIX LO bias voltage" };
static const struct LMS8Parameter HLMIXx_ICT_BIAS1 = { 0x2001, 6, 2, 0, "HLMIXx_ICT_BIAS1", "HLMIX core bias current control" };
static const struct LMS8Parameter HLMIXx_BIAS_PD1 = { 0x2001, 1, 1, 0, "HLMIXx_BIAS_PD1", "HLMIX core bias control" };
static const struct LMS8Parameter HLMIXx_LOBUF_PD1 = { 0x2001, 0, 0, 0, "HLMIXx_LOBUF_PD1", "HLMIX LO buffer power down" };
static const struct LMS8Parameter HLMIXx_MIXLOSS1 = { 0x2005, 5, 2, 0, "HLMIXx_MIXLOSS1", "HLMIX loss control" };

static const struct LMS8Parameter HLMIXx_VGCAS2 = { 0x2002, 13, 7, 64, "HLMIXx_VGCAS2", "HLMIX LO bias voltage" };
static const struct LMS8Parameter HLMIXx_ICT_BIAS2 = { 0x2002, 6, 2, 0, "HLMIXx_ICT_BIAS2", "HLMIX core bias current control" };
static const struct LMS8Parameter HLMIXx_BIAS_PD2 = { 0x2002, 1, 1, 0, "HLMIXx_BIAS_PD2", "HLMIX core bias control" };
static const struct LMS8Parameter HLMIXx_LOBUF_PD2 = { 0x2002, 0, 0, 0, "HLMIXx_LOBUF_PD2", "HLMIX LO buffer power down" };
static const struct LMS8Parameter HLMIXx_MIXLOSS2 = { 0x2006, 5, 2, 0, "HLMIXx_MIXLOSS2", "HLMIX loss control" };

static const struct LMS8Parameter HLMIXx_VGCAS3 = { 0x2003, 13, 7, 64, "HLMIXx_VGCAS3", "HLMIX LO bias voltage" };
static const struct LMS8Parameter HLMIXx_ICT_BIAS3 = { 0x2003, 6, 2, 0, "HLMIXx_ICT_BIAS3", "HLMIX core bias current control" };
static const struct LMS8Parameter HLMIXx_BIAS_PD3 = { 0x2003, 1, 1, 0, "HLMIXx_BIAS_PD3", "HLMIX core bias control" };
static const struct LMS8Parameter HLMIXx_LOBUF_PD3 = { 0x2003, 0, 0, 0, "HLMIXx_LOBUF_PD3", "HLMIX LO buffer power down" };
static const struct LMS8Parameter HLMIXx_MIXLOSS3 = { 0x2007, 5, 2, 0, "HLMIXx_MIXLOSS3", "HLMIX loss control" };

static const struct LMS8Parameter HLMIXx_VGCAS_RB = { 0x200E, 13, 7, 64, "HLMIXx_VGCAS_RB", "HLMIX LO bias voltage - Readback" };
static const struct LMS8Parameter HLMIXx_ICT_BIAS_RB = {
    0x200E, 6, 2, 0, "HLMIXx_ICT_BIAS_RB", "HLMIX core bias current control - Readback"
};
static const struct LMS8Parameter HLMIXx_BIAS_PD_RB = {
    0x200E, 1, 1, 0, "HLMIXx_BIAS_PD_RB", "HLMIX core bias control - Readback"
};
static const struct LMS8Parameter HLMIXx_LOBUF_PD_RB = {
    0x200E, 0, 0, 0, "HLMIXx_LOBUF_PD_RB", "HLMIX LO buffer power down - Readback"
};
static const struct LMS8Parameter HLMIXx_MIXLOSS_RB = { 0x200F, 5, 2, 0, "HLMIXx_MIXLOSS_RB", "HLMIX loss control - Readback" };

static const struct LMS8Parameter HLMIXx_CONFIG_SEL0_INTERNAL = {
    0x2008, 11, 11, 1, "HLMIXx_CONFIG_SEL0_INTERNAL", "HLMIX bias control signals multiplexer SEL0 signal is internal"
};
static const struct LMS8Parameter HLMIXx_CONFIG_SEL0_INVERT = {
    0x2008, 10, 10, 0, "HLMIXx_CONFIG_SEL0_INVERT", "Invert the SEL0 signal of HLMIX bias control signals"
};
static const struct LMS8Parameter HLMIXx_CONFIG_SEL0_MASK0 = {
    0x2008, 0, 0, 0, "HLMIXx_CONFIG_SEL0_MASK0", "GPIO0 mask for SEL0 signal of HLMIXx bias control signals multiplexer"
};
static const struct LMS8Parameter HLMIXx_CONFIG_SEL0_MASK1 = {
    0x2008, 1, 1, 0, "HLMIXx_CONFIG_SEL0_MASK1", "GPIO1 mask for SEL0 signal of HLMIXx bias control signals multiplexer"
};
static const struct LMS8Parameter HLMIXx_CONFIG_SEL0_MASK2 = {
    0x2008, 2, 2, 0, "HLMIXx_CONFIG_SEL0_MASK2", "GPIO2 mask for SEL0 signal of HLMIXx bias control signals multiplexer"
};
static const struct LMS8Parameter HLMIXx_CONFIG_SEL0_MASK3 = {
    0x2008, 3, 3, 0, "HLMIXx_CONFIG_SEL0_MASK3", "GPIO3 mask for SEL0 signal of HLMIXx bias control signals multiplexer"
};
static const struct LMS8Parameter HLMIXx_CONFIG_SEL0_MASK4 = {
    0x2008, 4, 4, 0, "HLMIXx_CONFIG_SEL0_MASK4", "GPIO4 mask for SEL0 signal of HLMIXx bias control signals multiplexer"
};
static const struct LMS8Parameter HLMIXx_CONFIG_SEL0_MASK5 = {
    0x2008, 5, 5, 0, "HLMIXx_CONFIG_SEL0_MASK5", "GPIO5 mask for SEL0 signal of HLMIXx bias control signals multiplexer"
};
static const struct LMS8Parameter HLMIXx_CONFIG_SEL0_MASK6 = {
    0x2008, 6, 6, 0, "HLMIXx_CONFIG_SEL0_MASK6", "GPIO6 mask for SEL0 signal of HLMIXx bias control signals multiplexer"
};
static const struct LMS8Parameter HLMIXx_CONFIG_SEL0_MASK7 = {
    0x2008, 7, 7, 0, "HLMIXx_CONFIG_SEL0_MASK7", "GPIO7 mask for SEL0 signal of HLMIXx bias control signals multiplexer"
};
static const struct LMS8Parameter HLMIXx_CONFIG_SEL0_MASK8 = {
    0x2008, 8, 8, 0, "HLMIXx_CONFIG_SEL0_MASK8", "GPIO8 mask for SEL0 signal of HLMIXx bias control signals multiplexer"
};

static const struct LMS8Parameter HLMIXx_CONFIG_SEL1_INTERNAL = {
    0x2009, 11, 11, 1, "HLMIXx_CONFIG_SEL1_INTERNAL", "HLMIX bias control signals multiplexer SEL1 signal is internal"
};
static const struct LMS8Parameter HLMIXx_CONFIG_SEL1_INVERT = {
    0x2009, 10, 10, 0, "HLMIXx_CONFIG_SEL1_INVERT", "Invert the SEL1 signal of HLMIX bias control signals"
};
static const struct LMS8Parameter HLMIXx_CONFIG_SEL1_MASK0 = {
    0x2009, 0, 0, 0, "HLMIXx_CONFIG_SEL1_MASK0", "GPIO0 mask for SEL1 signal of HLMIXx bias control signals multiplexer"
};
static const struct LMS8Parameter HLMIXx_CONFIG_SEL1_MASK1 = {
    0x2009, 1, 1, 0, "HLMIXx_CONFIG_SEL1_MASK1", "GPIO1 mask for SEL1 signal of HLMIXx bias control signals multiplexer"
};
static const struct LMS8Parameter HLMIXx_CONFIG_SEL1_MASK2 = {
    0x2009, 2, 2, 0, "HLMIXx_CONFIG_SEL1_MASK2", "GPIO2 mask for SEL1 signal of HLMIXx bias control signals multiplexer"
};
static const struct LMS8Parameter HLMIXx_CONFIG_SEL1_MASK3 = {
    0x2009, 3, 3, 0, "HLMIXx_CONFIG_SEL1_MASK3", "GPIO3 mask for SEL1 signal of HLMIXx bias control signals multiplexer"
};
static const struct LMS8Parameter HLMIXx_CONFIG_SEL1_MASK4 = {
    0x2009, 4, 4, 0, "HLMIXx_CONFIG_SEL1_MASK4", "GPIO4 mask for SEL1 signal of HLMIXx bias control signals multiplexer"
};
static const struct LMS8Parameter HLMIXx_CONFIG_SEL1_MASK5 = {
    0x2009, 5, 5, 0, "HLMIXx_CONFIG_SEL1_MASK5", "GPIO5 mask for SEL1 signal of HLMIXx bias control signals multiplexer"
};
static const struct LMS8Parameter HLMIXx_CONFIG_SEL1_MASK6 = {
    0x2009, 6, 6, 0, "HLMIXx_CONFIG_SEL1_MASK6", "GPIO6 mask for SEL1 signal of HLMIXx bias control signals multiplexer"
};
static const struct LMS8Parameter HLMIXx_CONFIG_SEL1_MASK7 = {
    0x2009, 7, 7, 0, "HLMIXx_CONFIG_SEL1_MASK7", "GPIO7 mask for SEL1 signal of HLMIXx bias control signals multiplexer"
};
static const struct LMS8Parameter HLMIXx_CONFIG_SEL1_MASK8 = {
    0x2009, 8, 8, 0, "HLMIXx_CONFIG_SEL1_MASK8", "GPIO8 mask for SEL1 signal of HLMIXx bias control signals multiplexer"
};

static const struct LMS8Parameter HLMIXx_CONFIG_INT_SEL0 = {
    0x200C, 0, 0, 0, "HLMIXx_CONFIG_INT_SEL0", "Internal value of HLMIX control signals multiplexer selection signals - Bit 0"
};
static const struct LMS8Parameter HLMIXx_CONFIG_INT_SEL1 = {
    0x200C, 1, 1, 0, "HLMIXx_CONFIG_INT_SEL1", "Internal value of HLMIX control signals multiplexer selection signals - Bit 1"
};

static const struct LMS8Parameter HLMIXx_LOSS_SEL0_INTERNAL = {
    0x200A, 11, 11, 1, "HLMIXx_LOSS_SEL0_INTERNAL", "HLMIX loss control signals multiplexer SEL0 signal is internal"
};
static const struct LMS8Parameter HLMIXx_LOSS_SEL0_INVERT = {
    0x200A, 10, 10, 0, "HLMIXx_LOSS_SEL0_INVERT", "Invert the SEL0 signal of HLMIX loss control signals"
};
static const struct LMS8Parameter HLMIXx_LOSS_SEL0_MASK0 = {
    0x200A, 0, 0, 0, "HLMIXx_LOSS_SEL0_MASK0", "GPIO0 mask for SEL0 signal of HLMIXx loss control signals multiplexer"
};
static const struct LMS8Parameter HLMIXx_LOSS_SEL0_MASK1 = {
    0x200A, 1, 1, 0, "HLMIXx_LOSS_SEL0_MASK1", "GPIO1 mask for SEL0 signal of HLMIXx loss control signals multiplexer"
};
static const struct LMS8Parameter HLMIXx_LOSS_SEL0_MASK2 = {
    0x200A, 2, 2, 0, "HLMIXx_LOSS_SEL0_MASK2", "GPIO2 mask for SEL0 signal of HLMIXx loss control signals multiplexer"
};
static const struct LMS8Parameter HLMIXx_LOSS_SEL0_MASK3 = {
    0x200A, 3, 3, 0, "HLMIXx_LOSS_SEL0_MASK3", "GPIO3 mask for SEL0 signal of HLMIXx loss control signals multiplexer"
};
static const struct LMS8Parameter HLMIXx_LOSS_SEL0_MASK4 = {
    0x200A, 4, 4, 0, "HLMIXx_LOSS_SEL0_MASK4", "GPIO4 mask for SEL0 signal of HLMIXx loss control signals multiplexer"
};
static const struct LMS8Parameter HLMIXx_LOSS_SEL0_MASK5 = {
    0x200A, 5, 5, 0, "HLMIXx_LOSS_SEL0_MASK5", "GPIO5 mask for SEL0 signal of HLMIXx loss control signals multiplexer"
};
static const struct LMS8Parameter HLMIXx_LOSS_SEL0_MASK6 = {
    0x200A, 6, 6, 0, "HLMIXx_LOSS_SEL0_MASK6", "GPIO6 mask for SEL0 signal of HLMIXx loss control signals multiplexer"
};
static const struct LMS8Parameter HLMIXx_LOSS_SEL0_MASK7 = {
    0x200A, 7, 7, 0, "HLMIXx_LOSS_SEL0_MASK7", "GPIO7 mask for SEL0 signal of HLMIXx loss control signals multiplexer"
};
static const struct LMS8Parameter HLMIXx_LOSS_SEL0_MASK8 = {
    0x200A, 8, 8, 0, "HLMIXx_LOSS_SEL0_MASK8", "GPIO8 mask for SEL0 signal of HLMIXx loss control signals multiplexer"
};

static const struct LMS8Parameter HLMIXx_LOSS_SEL1_INTERNAL = {
    0x200B, 11, 11, 1, "HLMIXx_LOSS_SEL1_INTERNAL", "HLMIX loss control signals multiplexer SEL1 signal is internal"
};
static const struct LMS8Parameter HLMIXx_LOSS_SEL1_INVERT = {
    0x200B, 10, 10, 0, "HLMIXx_LOSS_SEL1_INVERT", "Invert the SEL1 signal of HLMIX loss control signals"
};
static const struct LMS8Parameter HLMIXx_LOSS_SEL1_MASK0 = {
    0x200B, 0, 0, 0, "HLMIXx_LOSS_SEL1_MASK0", "GPIO0 mask for SEL1 signal of HLMIXx loss control signals multiplexer"
};
static const struct LMS8Parameter HLMIXx_LOSS_SEL1_MASK1 = {
    0x200B, 1, 1, 0, "HLMIXx_LOSS_SEL1_MASK1", "GPIO1 mask for SEL1 signal of HLMIXx loss control signals multiplexer"
};
static const struct LMS8Parameter HLMIXx_LOSS_SEL1_MASK2 = {
    0x200B, 2, 2, 0, "HLMIXx_LOSS_SEL1_MASK2", "GPIO2 mask for SEL1 signal of HLMIXx loss control signals multiplexer"
};
static const struct LMS8Parameter HLMIXx_LOSS_SEL1_MASK3 = {
    0x200B, 3, 3, 0, "HLMIXx_LOSS_SEL1_MASK3", "GPIO3 mask for SEL1 signal of HLMIXx loss control signals multiplexer"
};
static const struct LMS8Parameter HLMIXx_LOSS_SEL1_MASK4 = {
    0x200B, 4, 4, 0, "HLMIXx_LOSS_SEL1_MASK4", "GPIO4 mask for SEL1 signal of HLMIXx loss control signals multiplexer"
};
static const struct LMS8Parameter HLMIXx_LOSS_SEL1_MASK5 = {
    0x200B, 5, 5, 0, "HLMIXx_LOSS_SEL1_MASK5", "GPIO5 mask for SEL1 signal of HLMIXx loss control signals multiplexer"
};
static const struct LMS8Parameter HLMIXx_LOSS_SEL1_MASK6 = {
    0x200B, 6, 6, 0, "HLMIXx_LOSS_SEL1_MASK6", "GPIO6 mask for SEL1 signal of HLMIXx loss control signals multiplexer"
};
static const struct LMS8Parameter HLMIXx_LOSS_SEL1_MASK7 = {
    0x200B, 7, 7, 0, "HLMIXx_LOSS_SEL1_MASK7", "GPIO7 mask for SEL1 signal of HLMIXx loss control signals multiplexer"
};
static const struct LMS8Parameter HLMIXx_LOSS_SEL1_MASK8 = {
    0x200B, 8, 8, 0, "HLMIXx_LOSS_SEL1_MASK8", "GPIO8 mask for SEL1 signal of HLMIXx loss control signals multiplexer"
};

static const struct LMS8Parameter HLMIXx_LOSS_INT_SEL0 = {
    0x200C, 2, 2, 0, "HLMIXx_LOSS_INT_SEL0", "Internal value of HLMIX loss control signals multiplexer selection signals - Bit 0"
};
static const struct LMS8Parameter HLMIXx_LOSS_INT_SEL1 = {
    0x200C, 3, 3, 0, "HLMIXx_LOSS_INT_SEL1", "Internal value of HLMIX loss control signals multiplexer selection signals - Bit 1"
};

static const struct LMS8Parameter EN_VCOBIAS = { 0x4000, 11, 11, 0, "EN_VCOBIAS", "Enables VCO LDO and Bias Circuits" };
static const struct LMS8Parameter BYP_VCOREG = { 0x4000, 10, 10, 0, "BYP_VCOREG", "Bypasses VCO LDO" };
static const struct LMS8Parameter CURLIM_VCOREG = {
    0x4000, 9, 9, 1, "CURLIM_VCOREG", "Enables output current limitation in the VCO LDO"
};
static const struct LMS8Parameter SPDUP_VCOREG = {
    0x4000, 8, 8, 0, "SPDUP_VCOREG", "Shorts the noise filter resistor in the VCO LDO for fast settling time"
};
static const struct LMS8Parameter VDIV_VCOREG = { 0x4000, 7, 0, 32, "VDIV_VCOREG", "Controls the VCO LDO Output Voltage" };

static const struct LMS8Parameter PLL_XBUF_SLFBEN = {
    0x4001, 2, 2, 0, "PLL_XBUF_SLFBEN", "Enables self-biasing at the input of the XBUF"
};
static const struct LMS8Parameter PLL_XBUF_BYPEN = { 0x4001, 1, 1, 0, "PLL_XBUF_BYPEN", "Shorts the input stage buffer in XBUF" };
static const struct LMS8Parameter PLL_XBUF_EN = { 0x4001, 0, 0, 0, "PLL_XBUF_EN", "Enables XBUF" };

static const struct LMS8Parameter FCAL_START = {
    0x4002, 12, 12, 0, "FCAL_START", "Starts the automatic VCO frequency calibration algorithm (sticky-bit)"
};
static const struct LMS8Parameter VCO_SEL_FINAL_VAL = {
    0x4002, 11, 11, 0, "VCO_SEL_FINAL_VAL", "Valid bit for VCO_SEL_FINAL<1:0> result of automatic VCO frequency calibration process"
};
static const struct LMS8Parameter VCO_SEL_FINAL = {
    0x4002, 10, 9, 0, "VCO_SEL_FINAL", "Defines the optimal VCO core for synthesizing the targeted LO frequency"
};
static const struct LMS8Parameter FREQ_FINAL_VAL = {
    0x4002, 8, 8, 0, "FREQ_FINAL_VAL", "Valid bit for VCO_FREQ_FINAL<7:0>result of automatic VCO frequency calibration process"
};
static const struct LMS8Parameter FREQ_FINAL = { 0x4002,
    7,
    0,
    0,
    "FREQ_FINAL",
    "Defines the optimal cap bank configuration of the active LC-VCO core for synthesizing the targeted LO frequency" };
static const struct LMS8Parameter VCO_SEL_FORCE = { 0x4003,
    13,
    13,
    0,
    "VCO_SEL_FORCE",
    "Forces the user-defined VCO_SEL_INIT<1:0> word to select the active LC-VCO core and skips the VCO auto-select process during "
    "automatic VCO frequency calibration" };
static const struct LMS8Parameter VCO_SEL_INIT = {
    0x4003, 12, 11, 2, "VCO_SEL_INIT", "Defines active LC-VCO core when skipping the VCO auto-select process"
};
static const struct LMS8Parameter FREQ_INIT_POS = { 0x4003,
    10,
    8,
    7,
    "FREQ_INIT_POS",
    "Defines the starting bit-position for optimal cap-bank binary search process for the active LC-VCO core" };
static const struct LMS8Parameter FREQ_INIT = { 0x4003,
    7,
    0,
    0,
    "FREQ_INIT",
    "Initial cap bank configuration for binary search process during the automatic VCO frequency calibration" };
static const struct LMS8Parameter FREQ_SETTLING_N = { 0x4004,
    11,
    8,
    4,
    "FREQ_SETTLING_N",
    "VCO oscillation frequency settling-time during auto-calibration after updating cap bank configuration; Expressed as the "
    "number of reference clock cycles" };
static const struct LMS8Parameter VTUNE_WAIT_N = { 0x4004,
    7,
    0,
    64,
    "VTUNE_WAIT_N",
    "VCO tuning-voltage settling time at the beginning of the auto-calibration after opening the PLL loop; Expressed as the number "
    "of reference clock cycles" };
static const struct LMS8Parameter VCO_SEL_FREQ_MAX = {
    0x4005, 15, 8, 250, "VCO_SEL_FREQ_MAX", "High-frequency cap-bank configuration used during VCO auto-select process"
};
static const struct LMS8Parameter VCO_SEL_FREQ_MIN = {
    0x4005, 7, 0, 5, "VCO_SEL_FREQ_MIN", "Low-frequency cap-bank configuration used during VCO auto-select process"
};

static const struct LMS8Parameter VCO_FREQ_MAN = { 0x4006,
    15,
    8,
    128,
    "VCO_FREQ_MAN",
    "Cap Bank configuration multiplexed to the input of active VCO core when manual VCO calibration mode is selected" };
static const struct LMS8Parameter VCO_SEL_MAN = {
    0x4006, 7, 6, 2, "VCO_SEL_MAN", "VCO select word for choosing the active VCO core when manual VCO calibration mode is selected"
};
static const struct LMS8Parameter FREQ_HIGH = { 0x4006, 5, 5, 0, "FREQ_HIGH", "VCO Frequency Estimator Output" };
static const struct LMS8Parameter FREQ_EQUAL = { 0x4006, 4, 4, 0, "FREQ_EQUAL", "VCO Frequency Estimator Output" };
static const struct LMS8Parameter FREQ_LOW = { 0x4006, 3, 3, 0, "FREQ_LOW", "VCO Frequency Estimator Output" };
static const struct LMS8Parameter CTUNE_STEP_DONE = {
    0x4006, 2, 2, 0, "CTUNE_STEP_DONE", "VCO Frequency Estimator Output signalizes the end of VCO coarse-tuning step"
};
static const struct LMS8Parameter CTUNE_START = {
    0x4006, 1, 1, 0, "CTUNE_START", "Starts the process of estimating the VCO oscillation frequency"
};
static const struct LMS8Parameter CTUNE_EN = {
    0x4006, 0, 0, 0, "CTUNE_EN", "Starts the process of estimating the VCO oscillation frequency"
};

static const struct LMS8Parameter PLL_CFG_SEL0_INTERNAL = {
    0x4008, 11, 11, 1, "PLL_CFG_SEL0_INTERNAL", "PLL profile multiplexer SEL0 signal is generated from"
};
static const struct LMS8Parameter PLL_CFG_SEL0_INVERT = {
    0x4008, 10, 10, 0, "PLL_CFG_SEL0_INVERT", "Invert the SEL0 signal of PLL profile multiplexer"
};
static const struct LMS8Parameter PLL_CFG_SEL0_MASK0 = {
    0x4008, 0, 0, 0, "PLL_CFG_SEL0_MASK0", "GPIO0 mask for SEL0 signal of PLL profile multiplexer"
};
static const struct LMS8Parameter PLL_CFG_SEL0_MASK1 = {
    0x4008, 1, 1, 0, "PLL_CFG_SEL0_MASK1", "GPIO1 mask for SEL0 signal of PLL profile multiplexer"
};
static const struct LMS8Parameter PLL_CFG_SEL0_MASK2 = {
    0x4008, 2, 2, 0, "PLL_CFG_SEL0_MASK2", "GPIO2 mask for SEL0 signal of PLL profile multiplexer"
};
static const struct LMS8Parameter PLL_CFG_SEL0_MASK3 = {
    0x4008, 3, 3, 0, "PLL_CFG_SEL0_MASK3", "GPIO3 mask for SEL0 signal of PLL profile multiplexer"
};
static const struct LMS8Parameter PLL_CFG_SEL0_MASK4 = {
    0x4008, 4, 4, 0, "PLL_CFG_SEL0_MASK4", "GPIO4 mask for SEL0 signal of PLL profile multiplexer"
};
static const struct LMS8Parameter PLL_CFG_SEL0_MASK5 = {
    0x4008, 5, 5, 0, "PLL_CFG_SEL0_MASK5", "GPIO5 mask for SEL0 signal of PLL profile multiplexer"
};
static const struct LMS8Parameter PLL_CFG_SEL0_MASK6 = {
    0x4008, 6, 6, 0, "PLL_CFG_SEL0_MASK6", "GPIO6 mask for SEL0 signal of PLL profile multiplexer"
};
static const struct LMS8Parameter PLL_CFG_SEL0_MASK7 = {
    0x4008, 7, 7, 0, "PLL_CFG_SEL0_MASK7", "GPIO7 mask for SEL0 signal of PLL profile multiplexer"
};
static const struct LMS8Parameter PLL_CFG_SEL0_MASK8 = {
    0x4008, 8, 8, 0, "PLL_CFG_SEL0_MASK8", "GPIO8 mask for SEL0 signal of PLL profile multiplexer"
};

static const struct LMS8Parameter PLL_CFG_SEL0 = { 0x4008, 11, 0, 0x0800, "PLL_CFG_SEL0", "PLL profile multiplexer SEL0 control" };

static const struct LMS8Parameter PLL_CFG_SEL1_INTERNAL = {
    0x4009, 11, 11, 1, "PLL_CFG_SEL1_INTERNAL", "PLL profile multiplexer SEL1 signal is generated from"
};
static const struct LMS8Parameter PLL_CFG_SEL1_INVERT = {
    0x4009, 10, 10, 0, "PLL_CFG_SEL1_INVERT", "Invert the SEL1 signal of PLL profile multiplexer"
};
static const struct LMS8Parameter PLL_CFG_SEL1_MASK0 = {
    0x4009, 0, 0, 0, "PLL_CFG_SEL1_MASK0", "GPIO0 mask for SEL1 signal of PLL profile multiplexer"
};
static const struct LMS8Parameter PLL_CFG_SEL1_MASK1 = {
    0x4009, 1, 1, 0, "PLL_CFG_SEL1_MASK1", "GPIO1 mask for SEL1 signal of PLL profile multiplexer"
};
static const struct LMS8Parameter PLL_CFG_SEL1_MASK2 = {
    0x4009, 2, 2, 0, "PLL_CFG_SEL1_MASK2", "GPIO2 mask for SEL1 signal of PLL profile multiplexer"
};
static const struct LMS8Parameter PLL_CFG_SEL1_MASK3 = {
    0x4009, 3, 3, 0, "PLL_CFG_SEL1_MASK3", "GPIO3 mask for SEL1 signal of PLL profile multiplexer"
};
static const struct LMS8Parameter PLL_CFG_SEL1_MASK4 = {
    0x4009, 4, 4, 0, "PLL_CFG_SEL1_MASK4", "GPIO4 mask for SEL1 signal of PLL profile multiplexer"
};
static const struct LMS8Parameter PLL_CFG_SEL1_MASK5 = {
    0x4009, 5, 5, 0, "PLL_CFG_SEL1_MASK5", "GPIO5 mask for SEL1 signal of PLL profile multiplexer"
};
static const struct LMS8Parameter PLL_CFG_SEL1_MASK6 = {
    0x4009, 6, 6, 0, "PLL_CFG_SEL1_MASK6", "GPIO6 mask for SEL1 signal of PLL profile multiplexer"
};
static const struct LMS8Parameter PLL_CFG_SEL1_MASK7 = {
    0x4009, 7, 7, 0, "PLL_CFG_SEL1_MASK7", "GPIO7 mask for SEL1 signal of PLL profile multiplexer"
};
static const struct LMS8Parameter PLL_CFG_SEL1_MASK8 = {
    0x4009, 8, 8, 0, "PLL_CFG_SEL1_MASK8", "GPIO8 mask for SEL1 signal of PLL profile multiplexer"
};

static const struct LMS8Parameter PLL_CFG_SEL1 = { 0x4009, 11, 0, 0x0800, "PLL_CFG_SEL1", "PLL profile multiplexer SEL1 control" };

static const struct LMS8Parameter PLL_CFG_SEL2_INTERNAL = {
    0x400A, 11, 11, 1, "PLL_CFG_SEL2_INTERNAL", "PLL profile multiplexer SEL1 signal is generated from"
};
static const struct LMS8Parameter PLL_CFG_SEL2_INVERT = {
    0x400A, 10, 10, 0, "PLL_CFG_SEL2_INVERT", "Invert the SEL2 signal of PLL profile multiplexer"
};
static const struct LMS8Parameter PLL_CFG_SEL2_MASK0 = {
    0x400A, 0, 0, 0, "PLL_CFG_SEL2_MASK0", "GPIO0 mask for SEL2 signal of PLL profile multiplexer"
};
static const struct LMS8Parameter PLL_CFG_SEL2_MASK1 = {
    0x400A, 1, 1, 0, "PLL_CFG_SEL2_MASK1", "GPIO1 mask for SEL2 signal of PLL profile multiplexer"
};
static const struct LMS8Parameter PLL_CFG_SEL2_MASK2 = {
    0x400A, 2, 2, 0, "PLL_CFG_SEL2_MASK2", "GPIO2 mask for SEL2 signal of PLL profile multiplexer"
};
static const struct LMS8Parameter PLL_CFG_SEL2_MASK3 = {
    0x400A, 3, 3, 0, "PLL_CFG_SEL2_MASK3", "GPIO3 mask for SEL2 signal of PLL profile multiplexer"
};
static const struct LMS8Parameter PLL_CFG_SEL2_MASK4 = {
    0x400A, 4, 4, 0, "PLL_CFG_SEL2_MASK4", "GPIO4 mask for SEL2 signal of PLL profile multiplexer"
};
static const struct LMS8Parameter PLL_CFG_SEL2_MASK5 = {
    0x400A, 5, 5, 0, "PLL_CFG_SEL2_MASK5", "GPIO5 mask for SEL2 signal of PLL profile multiplexer"
};
static const struct LMS8Parameter PLL_CFG_SEL2_MASK6 = {
    0x400A, 6, 6, 0, "PLL_CFG_SEL2_MASK6", "GPIO6 mask for SEL2 signal of PLL profile multiplexer"
};
static const struct LMS8Parameter PLL_CFG_SEL2_MASK7 = {
    0x400A, 7, 7, 0, "PLL_CFG_SEL2_MASK7", "GPIO7 mask for SEL2 signal of PLL profile multiplexer"
};
static const struct LMS8Parameter PLL_CFG_SEL2_MASK8 = {
    0x400A, 8, 8, 0, "PLL_CFG_SEL2_MASK8", "GPIO8 mask for SEL2 signal of PLL profile multiplexer"
};

static const struct LMS8Parameter PLL_CFG_SEL2 = { 0x400A, 11, 0, 0x0800, "PLL_CFG_SEL2", "PLL profile multiplexer SEL2 control" };

static const struct LMS8Parameter PLL_CFG_INT_SEL0 = {
    0x400B, 0, 0, 0, "PLL_CFG_INT_SEL0", "Internal PLL profile control - Bit 0"
};
static const struct LMS8Parameter PLL_CFG_INT_SEL1 = {
    0x400B, 1, 1, 0, "PLL_CFG_INT_SEL1", "Internal PLL profile control - Bit 1"
};
static const struct LMS8Parameter PLL_CFG_INT_SEL2 = {
    0x400B, 2, 2, 0, "PLL_CFG_INT_SEL2", "Internal PLL profile control - Bit 2"
};

static const struct LMS8Parameter PLL_CFG_INT_SEL = { 0x400B, 2, 0, 0, "PLL_CFG_INT_SEL", "Internal PLL profile control" };

static const struct LMS8Parameter PLL_RSTN = { 0x400B, 9, 9, 0, "PLL_RSTN", "PLL reset, active low" };
static const struct LMS8Parameter CTUNE_RES = { 0x400B, 8, 7, 1, "CTUNE_RES", "Automatic VCO Frequency Calibration Resolution" };
static const struct LMS8Parameter PLL_CALIBRATION_MODE = { 0x400B, 6, 6, 0, "PLL_CALIBRATION_MODE", "PLL calibration mode" };
static const struct LMS8Parameter PLL_CALIBRATION_EN = { 0x400B, 5, 5, 0, "PLL_CALIBRATION_EN", "Activate PLL calibration" };
static const struct LMS8Parameter PLL_FLOCK_INTERNAL = { 0x400B, 4, 4, 0, "PLL_FLOCK_INTERNAL", "Fast lock control" };
static const struct LMS8Parameter PLL_FLOCK_INTVAL = {
    0x400B, 3, 3, 0, "PLL_FLOCK_INTVAL", "Fast lock control internal select value"
};

static const struct LMS8Parameter VTUNE_HIGH = { 0x400C, 2, 2, 0, "VTUNE_HIGH", "VCO Tuning voltage high" };
static const struct LMS8Parameter VTUNE_LOW = { 0x400C, 1, 1, 0, "VTUNE_LOW", "VCO Tuning voltage low" };
static const struct LMS8Parameter PLL_LOCK = { 0x400C, 0, 0, 0, "PLL_LOCK", "PLL lock detect" };

static const struct LMS8Parameter SEL_BIAS_CORE = {
    0x400E, 10, 10, 0, "SEL_BIAS_CORE", "Selects the bias for the core of  LO Distribution Network"
};
static const struct LMS8Parameter PLL_LODIST_ICT_CORE = {
    0x400E, 9, 5, 8, "PLL_LODIST_ICT_CORE", "Controls the IP20FRP bias current value when SEL_BIAS_CORE=1"
};
static const struct LMS8Parameter PLL_LODIST_ICT_BUF = {
    0x400E, 4, 0, 8, "PLL_LODIST_ICT_BUF", "Controls the input bias current value for LO Distribution Network output buffers"
};

static const struct LMS8Parameter PLL_ICT_OUT0 = { 0x400F,
    1,
    0,
    2,
    "PLL_ICT_OUT0",
    "Controls the current drive-strength of the LO Distribution Network Output Buffer Stage for CHA" };
static const struct LMS8Parameter PLL_ICT_OUT1 = { 0x400F,
    3,
    2,
    2,
    "PLL_ICT_OUT1",
    "Controls the current drive-strength of the LO Distribution Network Output Buffer Stage for CHB" };
static const struct LMS8Parameter PLL_ICT_OUT2 = { 0x400F,
    5,
    4,
    2,
    "PLL_ICT_OUT2",
    "Controls the current drive-strength of the LO Distribution Network Output Buffer Stage for CHB" };
static const struct LMS8Parameter PLL_ICT_OUT3 = { 0x400F,
    7,
    6,
    2,
    "PLL_ICT_OUT3",
    "Controls the current drive-strength of the LO Distribution Network Output Buffer Stage for CHB" };

static const struct LMS8Parameter BSIGL = { 0x4010, 15, 9, 2, "BSIGL", "BIST signature" };
static const struct LMS8Parameter BSTATE = { 0x4010, 8, 8, 0, "BSTATE", "BIST state indicator" };
static const struct LMS8Parameter EN_SDM_TSTO = { 0x4010, 4, 4, 0, "EN_SDM_TSTO", "Enable test buffer output" };
static const struct LMS8Parameter BEN = { 0x4010, 1, 1, 0, "BEN", "Enable BIST" };
static const struct LMS8Parameter BSTART = { 0x4010, 0, 0, 0, "BSTART", "Starts BIST" };

static const struct LMS8Parameter BSIGH = { 0x4011, 15, 0, 0, "BSIGH", "PLL_SDM_BIST Output" };

static const struct LMS8Parameter PLL_LODIST_EN_BIAS_n = {
    0x4100, 12, 12, 0, "PLL_LODIST_EN_BIAS_n", "Enable for LO distribution bias"
};
static const struct LMS8Parameter PLL_LODIST_EN_DIV2IQ_n = {
    0x4100, 11, 11, 0, "PLL_LODIST_EN_DIV2IQ_n", "Enable for IQ generator in LO distribution"
};
static const struct LMS8Parameter PLL_EN_VTUNE_COMP_n = {
    0x4100, 10, 10, 0, "PLL_EN_VTUNE_COMP_n", "Enable for tuning voltage comparator in PLL"
};
static const struct LMS8Parameter PLL_EN_LD_n = { 0x4100, 9, 9, 0, "PLL_EN_LD_n", "Lock detector enable" };
static const struct LMS8Parameter PLL_EN_PFD_n = { 0x4100, 8, 8, 0, "PLL_EN_PFD_n", "Enable for PFD in PLL" };
static const struct LMS8Parameter PLL_EN_CP_n = { 0x4100, 7, 7, 0, "PLL_EN_CP_n", "Enable for charge pump in PLL" };
static const struct LMS8Parameter PLL_EN_CPOFS_n = {
    0x4100, 6, 6, 0, "PLL_EN_CPOFS_n", "Enable for offset (bleeding) current in charge pump"
};
static const struct LMS8Parameter PLL_EN_VCO_n = { 0x4100, 5, 5, 0, "PLL_EN_VCO_n", "Enable for VCO" };
static const struct LMS8Parameter PLL_EN_FFDIV_n = { 0x4100, 4, 4, 0, "PLL_EN_FFDIV_n", "Enable for feed-forward divider in PLL" };
static const struct LMS8Parameter PLL_EN_FB_PDIV2_n = { 0x4100, 3, 3, 0, "PLL_EN_FB_PDIV2_n", "Enable for feedback pre-divider" };
static const struct LMS8Parameter PLL_EN_FFCORE_n = { 0x4100, 2, 2, 0, "PLL_EN_FFCORE_n", "Enable for feed-forward divider core" };
static const struct LMS8Parameter PLL_EN_FBDIV_n = { 0x4100, 1, 1, 0, "PLL_EN_FBDIV_n", "Enable for feedback divider core" };
static const struct LMS8Parameter PLL_SDM_CLK_EN_n = { 0x4100, 0, 0, 0, "PLL_SDM_CLK_EN_n", "Enable for sigma-delta modulator" };

static const struct LMS8Parameter R3_n = { 0x4101, 15, 12, 1, "R3_n", "Control word for loop filter - R3" };
static const struct LMS8Parameter R2_n = { 0x4101, 11, 8, 1, "R2_n", "Control word for loop filter - R2" };
static const struct LMS8Parameter C2_n = { 0x4101, 7, 4, 8, "C2_n", "Control word for loop filter - C2" };
static const struct LMS8Parameter C1_n = { 0x4101, 3, 0, 8, "C1_n", "Control word for loop filter - C1" };
static const struct LMS8Parameter VTUNE_VCT_n = {
    0x4102, 6, 5, 1, "VTUNE_VCT_n", "Tuning voltage control word during coarse tuning (LPFSW=1)"
};
static const struct LMS8Parameter LPFSW_n = { 0x4102, 4, 4, 0, "LPFSW_n", "Loop filter control" };
static const struct LMS8Parameter C3_n = { 0x4102, 3, 0, 8, "C3_n", "Control word for loop filter - C3" };

static const struct LMS8Parameter FLIP_n = { 0x4103, 14, 14, 0, "FLIP_n", "Flip for PFD inputs" };
static const struct LMS8Parameter DEL_n = { 0x4103, 13, 12, 0, "DEL_n", "Reset path delay" };
static const struct LMS8Parameter PULSE_n = { 0x4103, 11, 6, 4, "PULSE_n", "Charge pump pulse current" };
static const struct LMS8Parameter OFS_n = { 0x4103, 5, 0, 0, "OFS_n", "Charge pump offset (bleeding) current" };
static const struct LMS8Parameter LD_VCT_n = { 0x4104, 6, 5, 2, "LD_VCT_n", "Threshold voltage for lock detector" };
static const struct LMS8Parameter ICT_CP_n = { 0x4104, 4, 0, 16, "ICT_CP_n", "Charge pump bias current" };

static const struct LMS8Parameter VCO_FREQ_n = { 0x4105, 7, 0, 128, "VCO_FREQ_n", "VCO cap bank code" };
static const struct LMS8Parameter SPDUP_VCO_n = {
    0x4106, 12, 12, 0, "SPDUP_VCO_n", "Speed-up VCO core by bypassing the noise filter"
};
static const struct LMS8Parameter VCO_AAC_EN_n = {
    0x4106, 11, 11, 1, "VCO_AAC_EN_n", "Enable for automatic VCO amplitude control"
};
static const struct LMS8Parameter VDIV_SWVDD_n = { 0x4106, 10, 9, 2, "VDIV_SWVDD_n", "Capacitor bank switches bias voltage" };
static const struct LMS8Parameter VCO_SEL_n = { 0x4106, 8, 7, 3, "VCO_SEL_n", "VCO core selection" };
static const struct LMS8Parameter VCO_AMP_n = { 0x4106, 6, 0, 1, "VCO_AMP_n", "VCO amplitude control word" };

static const struct LMS8Parameter FFDIV_SEL_n = { 0x4107, 4, 4, 0, "FFDIV_SEL_n", "Feed-forward divider multiplexer select bit" };
static const struct LMS8Parameter FFCORE_MOD_n = { 0x4107, 3, 2, 0, "FFCORE_MOD_n", "Feed-forward divider multiplexer select bit" };
static const struct LMS8Parameter FF_MOD_n = {
    0x4107, 1, 0, 0, "FF_MOD_n", "Multiplexer for divider outputs. In normal operation FF_MOD should be equal to FFCORE_MOD."
};
static const struct LMS8Parameter INTMOD_EN_n = { 0x4108, 14, 14, 0, "INTMOD_EN_n", "Integer mode enable" };
static const struct LMS8Parameter DITHER_EN_n = { 0x4108, 13, 13, 0, "DITHER_EN_n", "Enable dithering in SDM" };
static const struct LMS8Parameter SEL_SDMCLK_n = {
    0x4108, 12, 12, 0, "SEL_SDMCLK_n", "Selects between the feedback divider output and FREF for SDM"
};
static const struct LMS8Parameter REV_SDMCLK_n = { 0x4108, 11, 11, 0, "REV_SDMCLK_n", "Reverses the SDM clock" };
static const struct LMS8Parameter INTMOD_n = { 0x4108, 9, 0, 216, "INTMOD_n", "Integer section of division ratio" };
static const struct LMS8Parameter FRACMODL_n = {
    0x4109, 15, 0, 22320, "FRACMODL_n", "Fractional control of the division ratio LSB"
};
static const struct LMS8Parameter FRACMODH_n = { 0x410A, 3, 0, 5, "FRACMODH_n", "Fractional control of the division ratio MSB" };

static const struct LMS8Parameter PLL_LODIST_EN_OUT_n = {
    0x410B, 15, 12, 0, "PLL_LODIST_EN_OUT_n", "LO distribution enable signals"
};
static const struct LMS8Parameter PLL_LODIST_FSP_OUT3_n = {
    0x410B, 11, 9, 0, "PLL_LODIST_FSP_OUT3_n", "LO distribution channel D frequency, sign and phase control"
};
static const struct LMS8Parameter PLL_LODIST_FSP_OUT2_n = {
    0x410B, 8, 6, 0, "PLL_LODIST_FSP_OUT2_n", "LO distribution channel C frequency, sign and phase control"
};
static const struct LMS8Parameter PLL_LODIST_FSP_OUT1_n = {
    0x410B, 5, 3, 0, "PLL_LODIST_FSP_OUT1_n", "LO distribution channel B frequency, sign and phase control"
};
static const struct LMS8Parameter PLL_LODIST_FSP_OUT0_n = {
    0x410B, 2, 0, 0, "PLL_LODIST_FSP_OUT0_n", "LO distribution channel A frequency, sign and phase control"
};
static const struct LMS8Parameter FLOCK_R3_n = { 0x410C, 15, 12, 4, "FLOCK_R3_n", "Loop filter R3 used during fact lock" };
static const struct LMS8Parameter FLOCK_R2_n = { 0x410C, 11, 8, 4, "FLOCK_R2_n", "Loop filter R2 used during fact lock" };
static const struct LMS8Parameter FLOCK_C2_n = { 0x410C, 7, 4, 8, "FLOCK_C2_n", "Loop filter C2 used during fact lock" };
static const struct LMS8Parameter FLOCK_C1_n = { 0x410C, 3, 0, 8, "FLOCK_C1_n", "Loop filter C1 used during fact lock" };
static const struct LMS8Parameter FLOCK_C3_n = { 0x410D, 15, 12, 8, "FLOCK_C3_n", "Loop filter C3 used during fact lock" };
static const struct LMS8Parameter FLOCK_PULSE_n = {
    0x410D, 11, 6, 63, "FLOCK_PULSE_n", "Charge pump pulse current used during fast lock"
};
static const struct LMS8Parameter FLOCK_OFS_n = {
    0x410D, 5, 0, 0, "FLOCK_OFS_n", "Charge pump offset (bleeding) current used during fast lock"
};
static const struct LMS8Parameter FLOCK_LODIST_EN_OUT_n = {
    0x410E, 14, 11, 0, "FLOCK_LODIST_EN_OUT_n", "LO distribution enable signals used during fast lock"
};
static const struct LMS8Parameter FLOCK_VCO_SPDUP_n = {
    0x410E, 10, 10, 0, "FLOCK_VCO_SPDUP_n", "VCO speedup used during fast lock"
};
static const struct LMS8Parameter FLOCK_N_n = {
    0x410E, 9, 0, 400, "FLOCK_N_n", "Duration of fast lock in PLL reference frequency clock cycles"
};

static const struct LMS8Parameter FLOCK_LODIST_EN_OUT0_n = {
    0x410E, 11, 11, 0, "FLOCK_LODIST_EN_OUT0_n", "LO distribution enable signal used during fast lock - Channel A"
};
static const struct LMS8Parameter FLOCK_LODIST_EN_OUT1_n = {
    0x410E, 12, 12, 0, "FLOCK_LODIST_EN_OUT1_n", "LO distribution enable signal used during fast lock - Channel B"
};
static const struct LMS8Parameter FLOCK_LODIST_EN_OUT2_n = {
    0x410E, 13, 13, 0, "FLOCK_LODIST_EN_OUT2_n", "LO distribution enable signal used during fast lock - Channel C"
};
static const struct LMS8Parameter FLOCK_LODIST_EN_OUT3_n = {
    0x410E, 14, 14, 0, "FLOCK_LODIST_EN_OUT3_n", "LO distribution enable signal used during fast lock - Channel D"
};

static const struct LMS8Parameter PLL_LODIST_EN_OUT0_n = {
    0x410B, 12, 12, 0, "PLL_LODIST_EN_OUT0_n", "LO distribution enable signal - Channel A"
};
static const struct LMS8Parameter PLL_LODIST_EN_OUT1_n = {
    0x410B, 13, 13, 0, "PLL_LODIST_EN_OUT1_n", "LO distribution enable signal - Channel B"
};
static const struct LMS8Parameter PLL_LODIST_EN_OUT2_n = {
    0x410B, 14, 14, 0, "PLL_LODIST_EN_OUT2_n", "LO distribution enable signal - Channel C"
};
static const struct LMS8Parameter PLL_LODIST_EN_OUT3_n = {
    0x410B, 15, 15, 0, "PLL_LODIST_EN_OUT3_n", "LO distribution enable signal - Channel D"
};

static const struct LMS8Parameter PLL_LODIST_FSP_OUT02_n = {
    0x410B, 2, 2, 0, "PLL_LODIST_FSP_OUT02_n", "LO distribution channel A - Divide by 2"
};
static const struct LMS8Parameter PLL_LODIST_FSP_OUT12_n = {
    0x410B, 5, 5, 0, "PLL_LODIST_FSP_OUT12_n", "LO distribution channel B - Divide by 2"
};
static const struct LMS8Parameter PLL_LODIST_FSP_OUT22_n = {
    0x410B, 8, 8, 0, "PLL_LODIST_FSP_OUT22_n", "LO distribution channel C - Divide by 2"
};
static const struct LMS8Parameter PLL_LODIST_FSP_OUT32_n = {
    0x410B, 11, 11, 0, "PLL_LODIST_FSP_OUT32_n", "LO distribution channel D - Divide by 2"
};

static const struct LMS8Parameter PLL_LODIST_FSP_OUT010_n = {
    0x410B, 1, 0, 0, "PLL_LODIST_FSP_OUT010_n", "LO distribution channel A - Phase control"
};
static const struct LMS8Parameter PLL_LODIST_FSP_OUT110_n = {
    0x410B, 4, 3, 0, "PLL_LODIST_FSP_OUT110_n", "LO distribution channel B - Phase control"
};
static const struct LMS8Parameter PLL_LODIST_FSP_OUT210_n = {
    0x410B, 7, 6, 0, "PLL_LODIST_FSP_OUT210_n", "LO distribution channel C - Phase control"
};
static const struct LMS8Parameter PLL_LODIST_FSP_OUT310_n = {
    0x410B, 10, 9, 0, "PLL_LODIST_FSP_OUT310_n", "LO distribution channel D - Phase control"
};

} // namespace lime

#endif
