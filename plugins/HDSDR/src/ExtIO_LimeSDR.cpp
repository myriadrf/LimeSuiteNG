/*
The MIT License

Copyright(c) 2017 Jiang Wei  <jiangwei@jiangwei.org>
New and modified work Copyright(c) 2024 Lime Microsystems Ltd

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files(the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE
*/

/*---------------------------------------------------------------------------*/
#define EXTIO_EXPORTS 1
#define HWNAME "ExtIO_LimeNG"
#define HWMODEL "ExtIO_LimeNG"
#define VERNUM "2.0"
#define EXT_BLOCKLEN 4096 /* only multiples of 512 */
//---------------------------------------------------------------------------
#include "ExtIO_LimeSDR.h"
#include <windows.h>
#include <Windowsx.h>
#include <commctrl.h>
#include "resource.h"
#include <process.h>
//---------------------------------------------------------------------------
#include <limesuiteng/complex.h>
#include <limesuiteng/types.h>
#include <limesuiteng/DeviceHandle.h>
#include <limesuiteng/DeviceRegistry.h>
#include <limesuiteng/Logger.h>
#include <limesuiteng/SDRDevice.h>
#include <limesuiteng/SDRDescriptor.h>
#include <limesuiteng/StreamConfig.h>
#include <limesuiteng/VersionInfo.h>
//---------------------------------------------------------------------------
#include <array>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <string_view>
#include <vector>
//---------------------------------------------------------------------------

#ifdef _DEBUG
#define _MYDEBUG // Activate a debug console
#endif

#ifdef _MYDEBUG
/* Debug Trace Enabled */
#include <stdio.h>
#define DbgPrintf(Message) std::printf("%s", Message)
#else
/* Debug Trace Disabled */
#define DbgPrintf(Message) MessageBox(NULL, Message, NULL, MB_OK | MB_ICONERROR)
#endif

using namespace std::literals::string_literals;
using namespace std::literals::string_view_literals;

//---------------------------------------------------------------------------
constexpr std::array<double, 8> sampleRates = {
    2000000,
    4000000,
    8000000,
    10000000,
    15000000,
    20000000,
    25000000,
    30000000,
};

constexpr double minimumLPFBandwidth = 1.4e6;
constexpr double maximumLPFBandwidth = 130e6;

constexpr double minimumCalibrationBandwidth = 2.5e6;
constexpr double maximumCalibrationBandwidth = 120e6;

enum class CalibrationStatus : uint8_t { NotCalibrated, Calibrated, CalibrationErr };

CalibrationStatus isCalibrated = CalibrationStatus::NotCalibrated;
bool isRunning = false;
bool isLPFEnabled = false;
bool isErrorLoggingEnabled = true;
std::string lastUsedDeviceName;
pfnExtIOCallback ExtIOCallback = nullptr;
std::array<lime::complex16_t, EXT_BLOCKLEN> buffer;

std::vector<lime::DeviceHandle> deviceList;
lime::SDRDevice* device = nullptr;
lime::LogHandler logHandler = nullptr;

HANDLE threadHandle = INVALID_HANDLE_VALUE;
EXTERN_C IMAGE_DOS_HEADER __ImageBase;
HWND dialogWindowHandle = nullptr;
HWND errorDialogWindowHandle = nullptr;

std::size_t currentDeviceIndex = 0U;
std::size_t numberOfDevices = 0U;
uint8_t numberOfChannels = 0U;
std::size_t channel = 0U;
std::size_t oversample = 2U;
uint8_t sampleRateIndex = 1U;
uint8_t antennaSelect = 1U;
int64_t currentLOFreq = 28.5e6; // default HDSDR LO freq
double LPFBandwidth = sampleRates.at(sampleRateIndex);
double calibrationBandwidth = sampleRates.at(sampleRateIndex);

/* 53 dB gain */
uint16_t LNA = 10;
uint16_t TIA = 3;
uint16_t PGA = 12;
//---------------------------------------------------------------------------
static void RecvThread(void* p)
{
    while (isRunning) {
        lime::complex16_t* bufferPointer = buffer.data();
        uint32_t samplesRead = device->StreamRx(0, &bufferPointer, EXT_BLOCKLEN, nullptr);
        if (ExtIOCallback != nullptr) {
            ExtIOCallback(samplesRead * 2, 0, 0, buffer.data());
        }
    }
    _endthread();
}
//---------------------------------------------------------------------------
static void error(lime::LogLevel lvl, const std::string& msg)
{
#ifdef _MYDEBUG
    DbgPrintf(msg.c_str());
    DbgPrintf("\n");
#else
    if (isErrorLoggingEnabled && lvl < lime::LogLevel::Warning) {
        if (lvl == lime::LogLevel::Critical) {
            ExtIOCallback(-1, extHw_Stop, 0, NULL);
        }
        DbgPrintf(msg.c_str());
    }
#endif
}
//---------------------------------------------------------------------------
static bool SetGain()
{
    int rcc_ctl_pga_rbb = (430 * std::pow(0.65, PGA / 10.0) - 110.35) / 20.4516 + 16; //From datasheet

    if (device->SetParameter(0, 0, "G_LNA_RFE"s, LNA) != lime::OpStatus::Success) {
        return false;
    }

    if (device->SetParameter(0, 0, "G_PGA_RBB"s, PGA) != lime::OpStatus::Success) {
        return false;
    }

    if (device->SetParameter(0, 0, "RCC_CTL_PGA_RBB"s, rcc_ctl_pga_rbb) != lime::OpStatus::Success) {
        return false;
    }

    if (device->SetParameter(0, 0, "G_TIA_RFE"s, TIA) != lime::OpStatus::Success) {
        return false;
    }

    return true;
}
//---------------------------------------------------------------------------
static void PerformCalibration(bool enableErrorLogging)
{
    int64_t freq = -1;
    if (isRunning) {
        freq = GetHWLO64();
        StopHW();
    }
    isErrorLoggingEnabled = enableErrorLogging;

    isCalibrated = CalibrationStatus::Calibrated;

    if (device->Calibrate(0, lime::TRXDir::Rx, channel, calibrationBandwidth) != lime::OpStatus::Success) {
        isCalibrated = CalibrationStatus::CalibrationErr;
    }

    isErrorLoggingEnabled = true;
    if (freq != -1) {
        StartHW64(freq);
    }
}
//---------------------------------------------------------------------------
static bool DisableLPF()
{
    int64_t freq = -1;
    if (isRunning) {
        freq = GetHWLO64();
        StopHW();
    }

    /* Making sure that tia isn't -12dB */
    if (device->SetParameter(0, 0, "G_TIA_RFE"s, 3) != lime::OpStatus::Success) {
        return false;
    }

    /* If the bandwidth is higher than 110e6, LPF is bypassed */
    if (device->SetLowPassFilter(0, lime::TRXDir::Rx, channel, 130e6) != lime::OpStatus::Success) {
        return false;
    }

    isLPFEnabled = false;

    if (device->SetParameter(0, 0, "G_TIA_RFE"s, TIA) != lime::OpStatus::Success) {
        return false;
    }

    if (freq != -1) {
        StartHW64(freq);
    }

    return true;
}
//---------------------------------------------------------------------------
static bool EnableLPF()
{
    if (LPFBandwidth > minimumLPFBandwidth && LPFBandwidth <= maximumLPFBandwidth) {
        int64_t freq = -1;

        if (isRunning) {
            freq = GetHWLO64();
            StopHW();
        }

        if (device->SetParameter(0, 0, "G_TIA_RFE"s, 3) != lime::OpStatus::Success) {
            return false;
        }

        if (device->SetLowPassFilter(0, lime::TRXDir::Rx, channel, LPFBandwidth) != lime::OpStatus::Success) {
            return false;
        }

        if (device->SetParameter(0, 0, "G_TIA_RFE"s, TIA) != lime::OpStatus::Success) {
            return false;
        }
        isLPFEnabled = true;

        if (LPFBandwidth > minimumCalibrationBandwidth) {
            calibrationBandwidth = LPFBandwidth;
        } else {
            calibrationBandwidth = minimumCalibrationBandwidth;
        }

        if (freq != -1) {
            StartHW64(freq);
        }
    } else {
        DbgPrintf("RxLPF frequency out of range, available range from 1.4 to 130 MHz");
        return false;
    }

    return true;
}
//---------------------------------------------------------------------------
static bool InitializeLMS()
{
    device = lime::DeviceRegistry::makeDevice(deviceList.at(currentDeviceIndex));
    if (device == nullptr) {
        return false;
    }

    const lime::SDRDescriptor& descriptor = device->GetDescriptor();

    numberOfChannels = descriptor.rfSOC.at(0).channelCount;
    if (numberOfChannels <= 0) {
        return false;
    }

    lime::SDRConfig configuration;
    auto& channelConfiguration = configuration.channel[channel].rx;

    channelConfiguration.enabled = true;
    channelConfiguration.sampleRate = sampleRates.at(sampleRateIndex);
    channelConfiguration.oversample = oversample;
    channelConfiguration.centerFrequency = currentLOFreq;
    channelConfiguration.path = antennaSelect;
    channelConfiguration.calibrate = true;
    channelConfiguration.gain[lime::eGainTypes::LNA] = LNA;
    channelConfiguration.gain[lime::eGainTypes::PGA] = PGA;
    channelConfiguration.gain[lime::eGainTypes::TIA] = TIA;
    channelConfiguration.lpf = isLPFEnabled ? LPFBandwidth : 130e6;

    isCalibrated = CalibrationStatus::Calibrated;

    if (device->Configure(configuration, 0) != lime::OpStatus::Success) {
        isCalibrated = CalibrationStatus::CalibrationErr;

        return false;
    }

    return true;
}

//---------------------------------------------------------------------------
static int UpdateDialog()
{
    Button_SetCheck(GetDlgItem(dialogWindowHandle, IDC_CHECK_ALPF), isLPFEnabled); // LPF checkbox
    ComboBox_SetCurSel(GetDlgItem(dialogWindowHandle, IDC_COMBO_CHAN), channel); // Channel dropdown
    ComboBox_SetCurSel(GetDlgItem(dialogWindowHandle, IDC_COMBO_DEVICE), currentDeviceIndex); // Device dropdown

    /* Set antenna selection */
    for (int i = 0; i < ComboBox_GetCount(GetDlgItem(dialogWindowHandle, IDC_COMBO_ANT)); i++) {
        if (ComboBox_GetItemData(GetDlgItem(dialogWindowHandle, IDC_COMBO_ANT), i) == antennaSelect) {
            ComboBox_SetCurSel(GetDlgItem(dialogWindowHandle, IDC_COMBO_ANT), i);
        }
    }

    /* Update LNA slider */
    LNA = device->GetParameter(0, 0, "G_LNA_RFE"s);
    SendDlgItemMessage(dialogWindowHandle, IDC_SLIDER_LNA, TBM_SETPOS, TRUE, 16 - LNA);
    std::string lna_value = std::to_string(LNA > 8 ? (LNA - 15) : (LNA - 11) * 3);
    lna_value.append(" dB"sv);
    Static_SetText(GetDlgItem(dialogWindowHandle, IDC_TEXT_LNA), lna_value.c_str());

    /* Update TIA slider */
    TIA = device->GetParameter(0, 0, "G_TIA_RFE"s);
    SendDlgItemMessage(dialogWindowHandle, IDC_SLIDER_TIA, TBM_SETPOS, TRUE, 4 - TIA);
    std::string tia_value = std::to_string((TIA == 3) ? 0 : (TIA == 2) ? -3 : -12);
    tia_value.append(" dB"sv);
    Static_SetText(GetDlgItem(dialogWindowHandle, IDC_TEXT_TIA), tia_value.c_str());

    /* Update PGA slider */
    PGA = device->GetParameter(0, 0, "G_PGA_RBB"s);
    SendDlgItemMessage(dialogWindowHandle, IDC_SLIDER_PGA, TBM_SETPOS, TRUE, 31 - PGA);
    std::string pga_value = std::to_string(PGA - 12);
    pga_value.append(" dB"sv);
    Static_SetText(GetDlgItem(dialogWindowHandle, IDC_TEXT_PGA), pga_value.c_str());

    /* Update LPF bandwidth */
    std::array<char, 7> bandwidth { 0, 0, 0, 0, 0, 0, 0 };
    std::snprintf(bandwidth.data(), bandwidth.size(), "%3.2f", LPFBandwidth / 1e6);
    SetDlgItemText(dialogWindowHandle, IDC_ALPF_BW, bandwidth.data());

    /* Update calibration bandwidth */
    std::snprintf(bandwidth.data(), bandwidth.size(), "%3.2f", calibrationBandwidth / 1e6);
    SetDlgItemText(dialogWindowHandle, IDC_CAL_BW, bandwidth.data());

    /* Update calibration text */
    if (isCalibrated == CalibrationStatus::Calibrated) {
        Static_SetText(GetDlgItem(dialogWindowHandle, IDC_TEXT_CALIBRATED), "Calibrated");
    } else if (isCalibrated == CalibrationStatus::NotCalibrated) {
        Static_SetText(GetDlgItem(dialogWindowHandle, IDC_TEXT_CALIBRATED), "Not calibrated");
    } else if (isCalibrated == CalibrationStatus::CalibrationErr) {
        Static_SetText(GetDlgItem(dialogWindowHandle, IDC_TEXT_CALIBRATED), "Calibration failed");
    }

    return TRUE;
}
//---------------------------------------------------------------------------
static int InitializeDialog(HWND hwndDlg, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    /* Add device choices */
    for (std::size_t i = 0; i < numberOfDevices; i++) {
        std::string info = std::to_string(i + 1) + ". "s + deviceList[i].Serialize();
        ComboBox_AddString(GetDlgItem(hwndDlg, IDC_COMBO_DEVICE), info.c_str());
    }
    ComboBox_SetCurSel(GetDlgItem(hwndDlg, IDC_COMBO_DEVICE), currentDeviceIndex);

    const lime::SDRDescriptor& descriptor = device->GetDescriptor();

    /* Add antenna choices */
    for (std::size_t i = 0; i < descriptor.rfSOC.at(0).pathNames.at(lime::TRXDir::Rx).size(); ++i) {
        ComboBox_AddString(
            GetDlgItem(hwndDlg, IDC_COMBO_ANT), descriptor.rfSOC.at(0).pathNames.at(lime::TRXDir::Rx).at(i).c_str());
        ComboBox_SetItemData(GetDlgItem(hwndDlg, IDC_COMBO_ANT), i, i);
    }

    /* Add channel choices */
    for (uint8_t i = 0; i < numberOfChannels; i++) {
        std::string channels = "RX"s + std::to_string(i + 1);
        ComboBox_AddString(GetDlgItem(hwndDlg, IDC_COMBO_CHAN), channels.c_str());
    }

    /* Add tick marks, set range for LNA slider */
    SendDlgItemMessage(hwndDlg, IDC_SLIDER_LNA, TBM_SETRANGEMIN, FALSE, 1);
    SendDlgItemMessage(hwndDlg, IDC_SLIDER_LNA, TBM_SETRANGEMAX, FALSE, 15);
    for (int i = 0; i < 15; i++) {
        SendDlgItemMessage(hwndDlg, IDC_SLIDER_LNA, TBM_SETTIC, FALSE, i);
    }

    /* Add tick marks, set range for TIA slider */
    SendDlgItemMessage(hwndDlg, IDC_SLIDER_TIA, TBM_SETRANGEMIN, FALSE, 1);
    SendDlgItemMessage(hwndDlg, IDC_SLIDER_TIA, TBM_SETRANGEMAX, FALSE, 3);
    for (int i = 0; i < 3; i++) {
        SendDlgItemMessage(hwndDlg, IDC_SLIDER_TIA, TBM_SETTIC, FALSE, i);
    }

    /* Add tick marks, set range for PGA slider */
    SendDlgItemMessage(hwndDlg, IDC_SLIDER_PGA, TBM_SETRANGEMIN, FALSE, 0);
    SendDlgItemMessage(hwndDlg, IDC_SLIDER_PGA, TBM_SETRANGEMAX, FALSE, 31);
    for (int i = 0; i < 31; i++) {
        SendDlgItemMessage(hwndDlg, IDC_SLIDER_PGA, TBM_SETTIC, FALSE, i);
    }

    /* Add library version */
    Static_SetText(GetDlgItem(hwndDlg, IDC_TEXT_LIBVER), lime::GetLibraryVersion().c_str());
    /* Add ExtIO version */
    Static_SetText(GetDlgItem(hwndDlg, IDC_TEXT_EXTVER), VERNUM);

    UpdateDialog();
    return TRUE;
}
//---------------------------------------------------------------------------
static int UpdateScroll(HWND hwndDlg, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    /* LNA slider moved */
    if (GetDlgItem(hwndDlg, IDC_SLIDER_LNA) == reinterpret_cast<HWND>(lParam)) {
        if (LNA != 16 - SendDlgItemMessage(hwndDlg, IDC_SLIDER_LNA, TBM_GETPOS, 0, NULL)) {
            LNA = 16 - SendDlgItemMessage(hwndDlg, IDC_SLIDER_LNA, TBM_GETPOS, 0, NULL);
            std::string lna_value = std::to_string(LNA > 8 ? (LNA - 15) : (LNA - 11) * 3); // Calculate from index to dB
            lna_value.append(" dB"sv);
            Static_SetText(GetDlgItem(hwndDlg, IDC_TEXT_LNA), lna_value.c_str());

            if (LNA <= 15) {
                device->SetParameter(0, 0, "G_LNA_RFE"s, LNA);
            }

            ExtIOCallback(-1, extHw_Changed_ATT, 0, NULL);
            isCalibrated = CalibrationStatus::NotCalibrated;
            UpdateDialog();
            return TRUE;
        }
    }
    /* TIA slider moved */
    if (GetDlgItem(hwndDlg, IDC_SLIDER_TIA) == reinterpret_cast<HWND>(lParam)) {
        if (TIA != 4 - SendDlgItemMessage(hwndDlg, IDC_SLIDER_TIA, TBM_GETPOS, 0, NULL)) {
            TIA = 4 - SendDlgItemMessage(hwndDlg, IDC_SLIDER_TIA, TBM_GETPOS, 0, NULL);
            std::string tia_value = std::to_string((TIA == 3) ? 0 : (TIA == 2) ? -3 : -12); // Calculate from index to dB
            tia_value.append(" dB"sv);
            Static_SetText(GetDlgItem(hwndDlg, IDC_TEXT_TIA), tia_value.c_str());

            if (TIA <= 3) {
                device->SetParameter(0, 0, "G_TIA_RFE"s, TIA);
            }

            ExtIOCallback(-1, extHw_Changed_ATT, 0, NULL);
            isCalibrated = CalibrationStatus::NotCalibrated;
            UpdateDialog();
            return TRUE;
        }
    }
    /* PGA slider moved */
    if (GetDlgItem(hwndDlg, IDC_SLIDER_PGA) == reinterpret_cast<HWND>(lParam)) {
        if (PGA != 31 - SendDlgItemMessage(hwndDlg, IDC_SLIDER_PGA, TBM_GETPOS, 0, NULL)) {
            PGA = 31 - SendDlgItemMessage(hwndDlg, IDC_SLIDER_PGA, TBM_GETPOS, 0, NULL);
            std::string pga_value = std::to_string(PGA - 12); // Calculate from index to dB
            pga_value.append(" dB"sv);
            Static_SetText(GetDlgItem(hwndDlg, IDC_TEXT_PGA), pga_value.c_str());

            int rcc_ctl_pga_rbb = (430 * std::pow(0.65, PGA / 10.0) - 110.35) / 20.4516 + 16; //From datasheet

            if (PGA <= 31) {
                device->SetParameter(0, 0, "G_PGA_RBB"s, PGA);
                device->SetParameter(0, 0, "RCC_CTL_PGA_RBB"s, rcc_ctl_pga_rbb);
            }

            ExtIOCallback(-1, extHw_Changed_ATT, 0, NULL);
            isCalibrated = CalibrationStatus::NotCalibrated;
            UpdateDialog();
            return TRUE;
        }
    }

    return FALSE;
}
//---------------------------------------------------------------------------
static int OnAntennaChange(HWND hwndDlg, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    if (GET_WM_COMMAND_CMD(wParam, lParam) == CBN_SELCHANGE) {
        int currentSel = ComboBox_GetCurSel(GET_WM_COMMAND_HWND(wParam, lParam));
        antennaSelect = ComboBox_GetItemData(GET_WM_COMMAND_HWND(wParam, lParam), currentSel);

        int64_t freq = -1;
        if (isRunning) {
            freq = GetHWLO64();
            StopHW();
        }

        if (device->SetAntenna(0, lime::TRXDir::Rx, channel, antennaSelect) != lime::OpStatus::Success) {
            return FALSE;
        }

        PerformCalibration(false);

        if (freq != -1) {
            StartHW64(freq);
        }

        UpdateDialog();

        return TRUE;
    }
    return FALSE;
}
//---------------------------------------------------------------------------
static int OnDeviceChange(HWND hwndDlg, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    if (GET_WM_COMMAND_CMD(wParam, lParam) == CBN_SELCHANGE) {
        if (currentDeviceIndex != ComboBox_GetCurSel(GET_WM_COMMAND_HWND(wParam, lParam))) {
            int64_t freq = -1;

            if (isRunning) {
                freq = GetHWLO64();
                StopHW();
            }

            lime::DeviceRegistry::freeDevice(device);
            device = nullptr;

            currentDeviceIndex = ComboBox_GetCurSel(GET_WM_COMMAND_HWND(wParam, lParam));

            /* Default settings */
            channel = 0;
            sampleRateIndex = 1;
            antennaSelect = 1;

            LNA = 10;
            TIA = 3;
            PGA = 16;

            isLPFEnabled = true;
            LPFBandwidth = sampleRates.at(sampleRateIndex);
            calibrationBandwidth = sampleRates.at(sampleRateIndex);

            if (!InitializeLMS()) {
                return FALSE;
            }

            /* Remove all channel selections */
            while (ComboBox_GetCount(GetDlgItem(hwndDlg, IDC_COMBO_CHAN)) != 0) {
                ComboBox_DeleteString(GetDlgItem(hwndDlg, IDC_COMBO_CHAN), 0);
            }

            /* Add channel selections */
            for (uint8_t i = 0; i < numberOfChannels; i++) {
                std::string channels = "RX"s + std::to_string(i + 1);
                ComboBox_AddString(GetDlgItem(hwndDlg, IDC_COMBO_CHAN), channels.c_str());
            }

            while (ComboBox_GetCount(GetDlgItem(hwndDlg, IDC_COMBO_ANT)) != 0) {
                ComboBox_DeleteString(GetDlgItem(hwndDlg, IDC_COMBO_ANT), 0);
            }

            const lime::SDRDescriptor& descriptor = device->GetDescriptor();

            /* Add antenna choices */
            for (std::size_t i = 0; i < descriptor.rfSOC.at(0).pathNames.at(lime::TRXDir::Rx).size(); ++i) {
                ComboBox_AddString(
                    GetDlgItem(hwndDlg, IDC_COMBO_ANT), descriptor.rfSOC.at(0).pathNames.at(lime::TRXDir::Rx).at(i).c_str());
                ComboBox_SetItemData(GetDlgItem(hwndDlg, IDC_COMBO_ANT), i, i);
            }

            UpdateDialog();

            /* Change last used device name */
            lastUsedDeviceName = descriptor.name;

            ExtIOCallback(-1, extHw_Changed_SampleRate, 0, NULL);
            ExtIOCallback(-1, extHw_Changed_ATT, 0, NULL);

            if (freq != -1) {
                StartHW64(freq);
            }

            return TRUE;
        }
    }
    return FALSE;
}
//---------------------------------------------------------------------------
static int OnChannelChange(HWND hwndDlg, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    if (GET_WM_COMMAND_CMD(wParam, lParam) == CBN_SELCHANGE) {
        if (channel != ComboBox_GetCurSel(GET_WM_COMMAND_HWND(wParam, lParam))) {
            int64_t freq = -1;
            if (isRunning) {
                freq = GetHWLO64();
                StopHW();
            }

            if (device->EnableChannel(0, lime::TRXDir::Rx, channel, false) != lime::OpStatus::Success) {
                return FALSE;
            }
            if (device->EnableChannel(0, lime::TRXDir::Tx, channel, false) != lime::OpStatus::Success) {
                return FALSE;
            }

            channel = ComboBox_GetCurSel(GET_WM_COMMAND_HWND(wParam, lParam));

            if (device->EnableChannel(0, lime::TRXDir::Rx, channel, true) != lime::OpStatus::Success) {
                return FALSE;
            }
            if (device->EnableChannel(0, lime::TRXDir::Tx, channel, true) != lime::OpStatus::Success) {
                return FALSE;
            }

            if (device->SetAntenna(0, lime::TRXDir::Rx, channel, antennaSelect) != lime::OpStatus::Success) {
                return FALSE;
            }

            if (isLPFEnabled) {
                EnableLPF();
            } else {
                DisableLPF();
            }

            if (!SetGain()) {
                return FALSE;
            }

            PerformCalibration(false);

            if (freq != -1) {
                StartHW64(freq);
            }

            UpdateDialog();

            return TRUE;
        }
    }

    return FALSE;
}
//---------------------------------------------------------------------------
static int OnCalibrate(HWND hwndDlg, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    if (GET_WM_COMMAND_CMD(wParam, lParam) == BN_CLICKED) {

        int buffSize = GetWindowTextLength(GetDlgItem(hwndDlg, IDC_CAL_BW));
        char* textBuffer = new char[buffSize + 1];

        GetDlgItemText(hwndDlg, IDC_CAL_BW, textBuffer, buffSize + 1);

        calibrationBandwidth = std::atof(textBuffer) * 1e6;
        if (calibrationBandwidth >= minimumCalibrationBandwidth && calibrationBandwidth <= maximumCalibrationBandwidth) {
            PerformCalibration(true);

            UpdateDialog();
        } else {
            DbgPrintf("Frequency out of range, available range from 2.5 to 120 MHz");
            calibrationBandwidth = sampleRates.at(sampleRateIndex);
            if (calibrationBandwidth < minimumCalibrationBandwidth) {
                calibrationBandwidth = minimumCalibrationBandwidth;
            }
            UpdateDialog();
        }

        delete[] textBuffer;
        return TRUE;
    }

    return FALSE;
}
//---------------------------------------------------------------------------
static int OnSet(HWND hwndDlg, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    if (GET_WM_COMMAND_CMD(wParam, lParam) == BN_CLICKED) {
        int buffSize = GetWindowTextLength(GetDlgItem(hwndDlg, IDC_ALPF_BW));
        char* textBuffer = new char[buffSize + 1];

        GetDlgItemText(hwndDlg, IDC_ALPF_BW, textBuffer, buffSize + 1);

        LPFBandwidth = std::atof(textBuffer) * 1e6;
        EnableLPF();
        PerformCalibration(false);
        UpdateDialog();

        delete[] textBuffer;
        return TRUE;
    }
    return FALSE;
}
//---------------------------------------------------------------------------
static int OnLPF(HWND hwndDlg, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    if (GET_WM_COMMAND_CMD(wParam, lParam) == BN_CLICKED) {
        if (IsDlgButtonChecked(hwndDlg, IDC_CHECK_ALPF) == BST_CHECKED) {
            EnableLPF();
            PerformCalibration(false);
            UpdateDialog();
        } else if (IsDlgButtonChecked(hwndDlg, IDC_CHECK_ALPF) == BST_UNCHECKED) {
            DisableLPF();
            PerformCalibration(false);
            UpdateDialog();
        }
    }
    return TRUE;
}
//---------------------------------------------------------------------------
static int OnReset(HWND hwndDlg, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    if (GET_WM_COMMAND_CMD(wParam, lParam) == BN_CLICKED) {
        int64_t freq = -1;
        if (isRunning) {
            freq = GetHWLO64();
            StopHW();
        }

        /* Default settings */
        isLPFEnabled = true;
        channel = 0;
        sampleRateIndex = 1;
        LPFBandwidth = sampleRates.at(sampleRateIndex);
        calibrationBandwidth = LPFBandwidth;
        antennaSelect = 1;

        LNA = 10;
        TIA = 3;
        PGA = 16;

        lime::DeviceRegistry::freeDevice(device);
        device = nullptr;
        InitializeLMS();

        ExtIOCallback(-1, extHw_Changed_SampleRate, 0, NULL);
        ExtIOCallback(-1, extHw_Changed_ATT, 0, NULL);

        if (freq != -1) {
            StartHW64(freq);
        }

        UpdateDialog();

        return TRUE;
    }
    return FALSE;
}
//---------------------------------------------------------------------------
static int CommandMessage(HWND hwndDlg, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    switch (GET_WM_COMMAND_ID(wParam, lParam)) {
    /* Changed antenna */
    case IDC_COMBO_ANT: return OnAntennaChange(hwndDlg, uMsg, wParam, lParam);

    /* Changed device */
    case IDC_COMBO_DEVICE: return OnDeviceChange(hwndDlg, uMsg, wParam, lParam);

    /* Changed channel */
    case IDC_COMBO_CHAN: return OnChannelChange(hwndDlg, uMsg, wParam, lParam);

    /* Pressed Calibrate button */
    case IDC_BUTTON_CAl: return OnCalibrate(hwndDlg, uMsg, wParam, lParam);

    /* Pressed Set button*/
    case IDC_BUTTON_SET: return OnSet(hwndDlg, uMsg, wParam, lParam);

    /* LPF checkbox clicked */
    case IDC_CHECK_ALPF: return OnLPF(hwndDlg, uMsg, wParam, lParam);

    /* Pressed Reset button */
    case IDC_BUTTON_DEFAULT: return OnReset(hwndDlg, uMsg, wParam, lParam);
    }

    return FALSE;
}
//---------------------------------------------------------------------------
static int StaticTextColorMessage(HWND hwndDlg, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    /* Calibrated text color */
    if (GetDlgCtrlID(reinterpret_cast<HWND>(lParam)) == IDC_TEXT_CALIBRATED) {
        HDC hdcStatic = reinterpret_cast<HDC>(wParam);
        if (isCalibrated == CalibrationStatus::Calibrated) {
            SetTextColor(hdcStatic, RGB(24, 135, 0));
        } else if (isCalibrated == CalibrationStatus::NotCalibrated) {
            SetTextColor(hdcStatic, RGB(0, 0, 0));
        } else if (isCalibrated == CalibrationStatus::CalibrationErr) {
            SetTextColor(hdcStatic, RGB(255, 0, 0));
        }

        SetBkColor(reinterpret_cast<HDC>(wParam), COLORREF(GetSysColor(COLOR_3DFACE)));
        return reinterpret_cast<INT_PTR>(GetSysColorBrush(COLOR_3DFACE));
    }

    return FALSE;
}
//---------------------------------------------------------------------------
static INT_PTR CALLBACK MainDlgProc(HWND hwndDlg, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    switch (uMsg) {
        /* Init starting variables */
    case WM_INITDIALOG: return InitializeDialog(hwndDlg, uMsg, wParam, lParam);

    /* Update dialog */
    case WM_SHOWWINDOW: return UpdateDialog();

    /* Scroll message */
    case WM_VSCROLL: return UpdateScroll(hwndDlg, uMsg, wParam, lParam);

    /* Command message */
    case WM_COMMAND: return CommandMessage(hwndDlg, uMsg, wParam, lParam);

    /* Static text color message */
    case WM_CTLCOLORSTATIC: return StaticTextColorMessage(hwndDlg, uMsg, wParam, lParam);

    /* Closed dialog window */
    case WM_CLOSE: ShowWindow(dialogWindowHandle, SW_HIDE); return TRUE;

    /* Destroy dialog window */
    case WM_DESTROY:
        ShowWindow(dialogWindowHandle, SW_HIDE);
        dialogWindowHandle = NULL;
        return TRUE;

    default: return FALSE;
    }
}
//---------------------------------------------------------------------------
extern "C" {
bool __declspec(dllexport) __stdcall InitHW(char* name, char* model, int& type)
{
    /* Create debug console window */
#ifdef _MYDEBUG
    if (AllocConsole()) {
        FILE* f;
        freopen_s(&f, "CONOUT$", "wt", stdout);
        SetConsoleTitle(TEXT("Debug Console ExtIO_Lime " VERNUM));
        SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_GREEN | FOREGROUND_BLUE | FOREGROUND_RED);
    }
#endif
    logHandler = &error;
    lime::registerLogHandler(logHandler);

    deviceList = lime::DeviceRegistry::enumerate();

    numberOfDevices = deviceList.size();
    if (numberOfDevices < 1) {
        DbgPrintf("No LMS device\n");
        return false;
    } else if (numberOfDevices < currentDeviceIndex + 1) {
        currentDeviceIndex = 0;
    }

    /* If device was changed, reset settings to default */
    if (lastUsedDeviceName != deviceList.at(currentDeviceIndex).Serialize()) {
        isLPFEnabled = true;
        sampleRateIndex = 1;
        LPFBandwidth = sampleRates.at(sampleRateIndex);
        calibrationBandwidth = LPFBandwidth;
        channel = 0;

        LNA = 10;
        TIA = 3;
        PGA = 16;

        lastUsedDeviceName = deviceList.at(currentDeviceIndex).Serialize();
    }

    type = exthwUSBdata16;
    std::strcpy(name, HWNAME);
    std::strcpy(model, deviceList.at(currentDeviceIndex).name.c_str());

    return true;
}
//---------------------------------------------------------------------------
bool EXTIO_API OpenHW(void)
{
    if (!InitializeLMS()) {
        return false;
    }

    dialogWindowHandle = CreateDialog(
        reinterpret_cast<HINSTANCE>(&__ImageBase), MAKEINTRESOURCE(ExtioDialog), NULL, static_cast<DLGPROC>(MainDlgProc));
    ShowWindow(dialogWindowHandle, SW_HIDE);

    return true;
}

//---------------------------------------------------------------------------
int EXTIO_API StartHW(long LOfreq)
{
    int ret = StartHW64(static_cast<int64_t>(LOfreq));
    return ret;
}
//---------------------------------------------------------------------------
int EXTIO_API StartHW64(int64_t LOfreq)
{
    SetHWLO64(LOfreq);

    lime::StreamConfig config;

    config.channels.at(lime::TRXDir::Rx) = { 0 };
    config.bufferSize = 1024 * 128;
    config.format = lime::DataFormat::I16;

    if (device->StreamSetup(config, 0) != lime::OpStatus::Success) {
        return -1;
    }

    device->StreamStart(0);

    isRunning = true;

    threadHandle = reinterpret_cast<HANDLE>(_beginthread(RecvThread, 0, NULL));
    SetThreadPriority(threadHandle, THREAD_PRIORITY_TIME_CRITICAL);

    return EXT_BLOCKLEN;
}
//---------------------------------------------------------------------------
void EXTIO_API StopHW(void)
{
    if (isRunning) {
        isRunning = false;

        WaitForSingleObject(threadHandle, INFINITE);
        threadHandle = INVALID_HANDLE_VALUE;

        device->StreamStop(0);
    }
}
//---------------------------------------------------------------------------
void EXTIO_API CloseHW(void)
{
    lime::DeviceRegistry::freeDevice(device);
    DestroyWindow(dialogWindowHandle);
}
//---------------------------------------------------------------------------
int EXTIO_API SetHWLO(long LOfreq)
{
    int64_t ret = SetHWLO64(static_cast<int64_t>(LOfreq));
    return (ret & 0xFFFFFFFF);
}
//---------------------------------------------------------------------------
int64_t EXTIO_API SetHWLO64(int64_t LOfreq)
{
    int64_t ret = 0;
    double freq = static_cast<double>(LOfreq);
    if (LOfreq < 1e3) {
        freq = 1e3;
        ret = -1 * (1e3);
    }

    if (LOfreq > 3800e6) {
        freq = 3800e6;
        ret = 3800e6;
    }

    if (isRunning && freq < 30e6) {
        StopHW();
        StartHW64(freq);
    } else {
        if (currentLOFreq != LOfreq) {
            isCalibrated = CalibrationStatus::NotCalibrated;
            device->SetFrequency(0, lime::TRXDir::Rx, channel, freq);

            UpdateDialog();
            ExtIOCallback(-1, extHw_Changed_LO, 0, NULL);
        }
    }

    return ret;
}
//---------------------------------------------------------------------------
int EXTIO_API GetStatus(void) { return 0; }
//---------------------------------------------------------------------------
void EXTIO_API SetCallback(pfnExtIOCallback funcptr)
{
    ExtIOCallback = funcptr;
    return;
}
//---------------------------------------------------------------------------
void EXTIO_API VersionInfo(const char* progname, int ver_major, int ver_minor) { return; }
//---------------------------------------------------------------------------
long EXTIO_API GetHWLO(void)
{
    int64_t glLOfreq = GetHWLO64();
    return static_cast<long>(glLOfreq & 0xFFFFFFFF);
}
//---------------------------------------------------------------------------
int64_t EXTIO_API GetHWLO64(void)
{
    double freq = device->GetFrequency(0, lime::TRXDir::Rx, channel);

    freq = std::ceil(freq);
    currentLOFreq = static_cast<int64_t>(freq);
    return currentLOFreq;
}
//---------------------------------------------------------------------------
long EXTIO_API GetHWSR(void)
{
    long sampleRate = 0;

    if (device != nullptr) {
        double freq = device->GetSampleRate(0, lime::TRXDir::Rx, channel);

        sampleRate = std::round(freq);
    }
    return sampleRate;
}
//---------------------------------------------------------------------------
int EXTIO_API GetAttenuators(int atten_idx, float* attenuation)
{
    if (atten_idx < 74) {
        *attenuation = atten_idx;
        return 0;
    }
    return -1; // Finished
}
//---------------------------------------------------------------------------
int EXTIO_API GetActualAttIdx(void)
{
    double _gain = 0;

    if (device != nullptr) {
        if (device->GetGain(0, lime::TRXDir::Rx, channel, lime::eGainTypes::UNKNOWN, _gain) != lime::OpStatus::Success) {
            return -1; // ERROR
        }
    }

    return static_cast<int>(_gain);
}
//---------------------------------------------------------------------------
int EXTIO_API SetAttenuator(int atten_idx)
{
    if (device != nullptr) {
        if (device->SetGain(0, lime::TRXDir::Rx, channel, lime::eGainTypes::UNKNOWN, atten_idx) != lime::OpStatus::Success) {
            return -1; // ERROR
        }
        isCalibrated = CalibrationStatus::NotCalibrated;
        UpdateDialog();
    }
    return 0;
}
//---------------------------------------------------------------------------
int EXTIO_API ExtIoGetSrates(int srate_idx, double* samplerate)
{
    if (srate_idx < sampleRates.size()) {
        *samplerate = sampleRates.at(srate_idx);
        return 0;
    } else {
        return -1; // Finished
    }
}
//---------------------------------------------------------------------------
int EXTIO_API ExtIoGetActualSrateIdx(void) { return sampleRateIndex; }
//---------------------------------------------------------------------------
int EXTIO_API ExtIoSetSrate(int srate_idx)
{
    if (srate_idx >= 0 && srate_idx < sampleRates.size()) {
        int64_t freq = 0;

        if (isRunning) {
            freq = GetHWLO64();
            StopHW();
        }

        if (device->SetSampleRate(0, lime::TRXDir::Rx, 0, sampleRates.at(srate_idx), oversample) != lime::OpStatus::Success) {
            return -1;
        }

        calibrationBandwidth = LPFBandwidth = sampleRates.at(srate_idx);
        if (calibrationBandwidth < minimumCalibrationBandwidth) {
            calibrationBandwidth = minimumCalibrationBandwidth;
        }

        if (isLPFEnabled) {
            EnableLPF();
        }

        PerformCalibration(false);

        if (freq != 0) {
            StartHW64(freq);
        }

        sampleRateIndex = srate_idx;
        ExtIOCallback(-1, extHw_Changed_SampleRate, 0, NULL);

        UpdateDialog();
        return 0;
    }

    return -1; // ERROR
}
//---------------------------------------------------------------------------
int EXTIO_API ExtIoGetSetting(int idx, char* description, char* value)
{
    switch (idx) {
    case 0:
        std::snprintf(description, 1024, "%s", "HW Name");
        std::strncpy(value, lastUsedDeviceName.c_str(), 1024);
        return 0;
    case 1:
        std::snprintf(description, 1024, "%s", "Channel");
        std::snprintf(value, 1024, "%zd", channel);
        return 0;
    case 2:
        std::snprintf(description, 1024, "%s", "Antenna");
        std::snprintf(value, 1024, "%d", antennaSelect);
        return 0;
    case 3:
        std::snprintf(description, 1024, "%s", "LPF bandwidth");
        std::snprintf(value, 1024, "%f", LPFBandwidth);
        return 0;
    case 4:
        std::snprintf(description, 1024, "%s", "LPF enable");
        std::snprintf(value, 1024, "%d", isLPFEnabled);
        return 0;
    case 5:
        std::snprintf(description, 1024, "%s", "Sample rate index");
        std::snprintf(value, 1024, "%d", sampleRateIndex);
        return 0;
    case 6:
        std::snprintf(description, 1024, "%s", "LNA gain");
        std::snprintf(value, 1024, "%d", LNA);
        return 0;
    case 7:
        std::snprintf(description, 1024, "%s", "TIA gain");
        std::snprintf(value, 1024, "%d", TIA);
        return 0;
    case 8:
        std::snprintf(description, 1024, "%s", "PGA gain");
        std::snprintf(value, 1024, "%d", PGA);
        return 0;
    case 9:
        std::snprintf(description, 1024, "%s", "Current Device index");
        std::snprintf(value, 1024, "%zd", currentDeviceIndex);
        return 0;
    case 10:
        std::snprintf(description, 1024, "%s", "Calibration bandwidth");
        std::snprintf(value, 1024, "%f", calibrationBandwidth);
        return 0;
    case 11:
        std::snprintf(description, 1024, "%s", "Current LO frequency");
        std::snprintf(value, 1024, "%lld", currentLOFreq);
        return 0;
    default: return -1; // ERROR
    }
}
//---------------------------------------------------------------------------
void EXTIO_API ExtIoSetSetting(int idx, const char* value)
{
    int tempInt = 0;
    int64_t temp64Int = 0;
    double tempFloat = 0.0;

    switch (idx) {
    case 0: lastUsedDeviceName = std::string { value }; return;
    case 1:
        tempInt = std::atoi(value);

        if (tempInt >= 0 && tempInt < 2) {
            channel = tempInt;
        }
        return;
    case 2:
        tempInt = std::atoi(value);

        if (tempInt >= 0 && tempInt < 4) {
            antennaSelect = tempInt;
        }
        return;
    case 3:
        tempFloat = std::atof(value);

        if (tempFloat > minimumLPFBandwidth && tempFloat <= maximumLPFBandwidth) {
            LPFBandwidth = tempFloat;
        }
        return;
    case 4:
        tempInt = std::atoi(value);

        if (tempInt == 0 || tempInt == 1) {
            isLPFEnabled = tempInt;
        }
        return;
    case 5:
        tempInt = std::atoi(value);

        if (tempInt >= 0 && tempInt < sampleRates.size()) {
            sampleRateIndex = tempInt;
        }
        return;
    case 6:
        tempInt = std::atoi(value);

        if (tempInt > 0 && tempInt < 16) {
            LNA = tempInt;
        }
        return;
    case 7:
        tempInt = std::atoi(value);

        if (tempInt > 0 && tempInt < 4) {
            TIA = tempInt;
        }
        return;
    case 8:
        tempInt = std::atoi(value);

        if (tempInt >= 0 && tempInt < 32) {
            PGA = tempInt;
        }
        return;
    case 9: currentDeviceIndex = std::atoi(value); return;
    case 10:
        tempFloat = std::atof(value);

        if (tempFloat >= minimumCalibrationBandwidth && tempFloat <= maximumCalibrationBandwidth) {
            calibrationBandwidth = tempFloat;
        }
        return;
    case 11:
        temp64Int = std::atol(value);

        if (temp64Int > 1e3 && temp64Int <= 3800e6) {
            currentLOFreq = temp64Int;
        }
        return;
    default: return;
    }
}
//---------------------------------------------------------------------------
void EXTIO_API ShowGUI()
{
    ShowWindow(dialogWindowHandle, SW_SHOW);
    return;
}
//---------------------------------------------------------------------------
void EXTIO_API HideGUI()
{
    ShowWindow(dialogWindowHandle, SW_HIDE);
    return;
}
//---------------------------------------------------------------------------
void EXTIO_API SwitchGUI()
{
    if (IsWindowVisible(dialogWindowHandle)) {
        ShowWindow(dialogWindowHandle, SW_HIDE);
    } else {
        ShowWindow(dialogWindowHandle, SW_SHOW);
    }
    return;
}
//---------------------------------------------------------------------------
} // extern "C"