#include "LMSBoards.h"

#include <string_view>

using namespace std::literals::string_view_literals;

namespace lime {

/// The list of device names corresponding to the eLMS_DEV enumerator.
static constexpr std::array<const std::string_view, LMS_DEV_COUNT> LMS_DEV_NAMES = {
    "UNKNOWN"sv,
    "EVB6"sv,
    "DigiGreen"sv,
    "DigiRed"sv,
    "EVB7"sv,
    "ZIPPER"sv,
    "Socket Board"sv,
    "EVB7_v2"sv,
    "Stream"sv,
    "Novena"sv,
    "DataSpark"sv,
    "RF-Spark"sv,
    "LMS6002-USB Stick"sv,
    "RF-ESpark"sv,
    "LimeSDR-USB"sv,
    "LimeSDR-PCIe"sv,
    "LimeSDR-QPCIe"sv,
    "LimeSDR-Mini"sv,
    "uStream"sv,
    "LimeSDR SONY PA"sv,
    "LimeSDR-USB SP"sv,
    "LMS7002M Ultimate EVB"sv,
    "LimeNET-Micro"sv,
    "LimeSDR-Core"sv,
    "LimeSDR-Core-HE"sv,
    "LimeSDR-Mini_v2"sv,
    "LimeSDR X3"sv,
    "LimeSDR XTRX"sv,
    "LimeSDR MMX8"sv,
    "LimeSDR Micro"sv,
    "XSDR"sv,
};

const std::string_view GetDeviceName(const eLMS_DEV device)
{
    if (LMS_DEV_UNKNOWN < device && device < LMS_DEV_COUNT)
        return LMS_DEV_NAMES.at(device);

    return LMS_DEV_NAMES.at(LMS_DEV_UNKNOWN);
}

/// The expansion board names corresponding to the eEXP_BOARD enumerator.
static constexpr std::array<const std::string_view, EXP_BOARD_COUNT> EXP_BOARD_NAMES = {
    "UNKNOWN"sv,
    "UNSUPPORTED"sv,
    "NOT AVAILABLE"sv,
    "Myriad1"sv,
    "Myriad2"sv,
    "Novena"sv,
    "HPM1000"sv,
    "Myriad7"sv,
    "HMP7"sv,
    "Myriad7 Novena"sv,
};

const std::string_view GetExpansionBoardName(const eEXP_BOARD board)
{
    if (EXP_BOARD_UNKNOWN < board && board < EXP_BOARD_COUNT)
        return EXP_BOARD_NAMES.at(board);

    return EXP_BOARD_NAMES.at(EXP_BOARD_UNKNOWN);
}

} // namespace lime
