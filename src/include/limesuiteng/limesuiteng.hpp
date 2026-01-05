#ifndef LIMESUITENG_HPP
#define LIMESUITENG_HPP

#include "limesuiteng/DeviceRegistry.h"
#include "limesuiteng/DeviceHandle.h"
#include "limesuiteng/Logger.h"
#include "limesuiteng/RFSOCDescriptor.h"
#include "limesuiteng/RFStream.h"
#include "limesuiteng/SDRConfig.h"
#include "limesuiteng/SDRDevice.h"
#include "limesuiteng/SDRDescriptor.h"
#include "limesuiteng/StreamConfig.h"
#include "limesuiteng/VersionInfo.h"

#endif

/**
 * @mainpage LimeSuiteNG 1.0.0 library manual
 *
 * @section intro Introduction
 *
 * LimeSuiteNG library is a component of LimeSuiteNG suite, which enables communication 
 * with LimeSDR devices from user space. This documentation focuses on LimeSuiteNG library 
 * public API, provides other important notes, explanations and is intended for application 
 * developers, who wish to communicate with LimeSDR devices using custom built applications.
 *
 * This documentation does not provide library installation steps. For more information about 
 * the library installation for different platforms and other components of LimeSuiteNG suite, 
 * visit the main <A HREF="https://limesuiteng.myriadrf.org/">  LimeSuiteNG documentation</A> page.
 * 
 * @section api_section Application Programming Interface (API)
 * 
 * See @ref api "Application Programming Interface" for a full list of LimeSuiteNG library functions.
 * 
 * @section lib_feat_section Library features
 * 
 * TODO: Add list of supported library features.
 * 
 * @section getting_started Getting started
 * 
 * To start using LimeSuiteNG library public API, include the library header file:
 * 
 * TODO: Add include headers for access to public API.
 * 
 * To get started using LimeSuiteNG library public API, visit Topics page. For a full list of public
 * API members, visit @ref api "API" page.
 */

 
/**
 * @page api Application Programming Interface
 * 
 * This page provides a complete list of functions, enumerations, structures that can be accessed 
 * through public API:
 * 
 * @section funcs Functions 
 * 
 * @subsection registration SDR Device registration
 * <ul>
 *    <li> lime::DeviceRegistry::enumerate() </li>
 *    <li> lime::DeviceRegistry::enumerate(const lime::DeviceHandle&) </li>
 *    <li> lime::DeviceRegistry::makeDevice(const lime::DeviceHandle&) </li>
 *    <li> lime::DeviceRegistry::freeDevice(lime::SDRDevice*) </li>
 *    <li> lime::DeviceRegistry::moduleNames(void) </li>
 * </ul>
 * 
 * @subsection control_adn_config SDR Device control and configuration
 * <ul>
 *    <li> lime::SDRDevice::Init() </li>
 *    <li> lime::SDRDevice::Reset() </li>
 *    <li> lime::SDRDevice::EnableChannel(uint8_t, lime::TRXDir, uint8_t, bool) </li>
 *    <li> lime::SDRDevice::StreamCreate(const lime::StreamConfig&, uint8_t) </li>
 *    <li> lime::SDRDevice::UploadTxWaveform(const lime::StreamConfig&, uint8_t, const void**, uint32_t) </li>
 *    <li> lime::SDRDevice::Calibrate(uint8_t, lime::TRXDir, uint8_t, double) </li>
 *    <li> lime::SDRDevice::Synchronize(bool) </li>
 *    <li><b>Configuration:</b>
 *       <ul>
 *          <li> lime::SDRDevice::Configure(const lime::SDRConfig&, uint8_t) </li>
 *          <li> lime::SDRDevice::LoadConfig(uint8_t, const std::string&) </li>
 *          <li> lime::SDRDevice::SaveConfig(uint8_t, const std::string&) </li>
 *          <li> lime::SDRDevice::GetParameter(uint8_t, uint8_t, const std::string&) </li>
 *          <li> lime::SDRDevice::SetParameter(uint8_t, uint8_t, const std::string&, uint16_t) </li>
 *          <li> lime::SDRDevice::GetParameter(uint8_t, uint8_t, uint16_t, uint8_t, uint8_t) </li>
 *          <li> lime::SDRDevice::SetParameter(uint8_t, uint8_t, uint16_t, uint8_t, uint8_t, uint16_t) </li>
 *       </ul>
 *    </li>
 *    <li> <b> Filter configuration:</b>
 *       <ul>
 *        <li> lime::SDRDevice::ConfigureGFIR(uint8_t, lime::TRXDir, uint8_t, lime::ChannelConfig::Direction::GFIRFilter) </li>
 *        <li> lime::SDRDevice::GetGFIRCoefficients(uint8_t, lime::TRXDir, uint8_t, uint8_t) </li>
 *        <li> lime::SDRDevice::SetGFIRCoefficients(uint8_t, lime::TRXDir, uint8_t, uint8_t, std::vector<double>) </li>
 *        <li> lime::SDRDevice::SetGFIR(uint8_t, lime::TRXDir, uint8_t, uint8_t, bool) </li>
 *        <li> lime::SDRDevice::GetLowPassFilter(uint8_t, lime::TRXDir, uint8_t) </li>
 *        <li> lime::SDRDevice::SetLowPassFilter(uint8_t, lime::TRXDir, uint8_t, double) </li>
 *       </ul>
 *    </li>
 *    <li><b> Additional info:</b>
 *       <ul>
 *          <li> lime::SDRDevice::GetDescriptor() </li>
 *          <li> lime::SDRDevice::GetGPSLock(lime::SDRDevice::GPS_Lock*) </li>
 *          <li> lime::SDRDevice::GetCGENLocked(uint8_t) </li>
 *          <li> lime::SDRDevice::GetTemperature(uint8_t) </li>
 *          <li> lime::SDRDevice::GetSXLocked(uint8_t, lime::TRXDir) </li>
 *       </ul>
 *    </li>
 *    <li><b>Configuration by parameter:</b>
 *       <ul>
 *          <li> lime::SDRDevice::GetClockFreq(uint8_t, uint8_t) </li>
 *          <li> lime::SDRDevice::SetClockFreq(uint8_t,double,uint8_t) </li>
 *          <li> lime::SDRDevice::GetFrequency(uint8_t,lime::TRXDir,uint8_t) </li>
 *          <li> lime::SDRDevice::SetFrequency(uint8_t, lime::TRXDir, uint8_t, double) </li>
 *          <li> lime::SDRDevice::GetNCOFrequency(uint8_t, lime::TRXDir, uint8_t, uint8_t, double&) </li>
 *          <li> lime::SDRDevice::SetNCOFrequency(uint8_t, lime::TRXDir, uint8_t, uint8_t, double, double) </li>
 *          <li> lime::SDRDevice::GetNCOOffset(uint8_t, lime::TRXDir, uint8_t) </li>
 *          <li> lime::SDRDevice::GetNCOIndex(uint8_t, lime::TRXDir, uint8_t) </li>
 *          <li> lime::SDRDevice::SetNCOIndex(uint8_t, lime::TRXDir, uint8_t, uint8_t, bool) </li>
 *          <li> lime::SDRDevice::GetSampleRate(uint8_t, lime::TRXDir, uint8_t, uint32_t*) </li>
 *          <li> lime::SDRDevice::SetSampleRate(uint8_t, lime::TRXDir, uint8_t, double, uint8_t) </li>
 *          <li> lime::SDRDevice::GetGain(uint8_t, lime::TRXDir, uint8_t, lime::eGainTypes, double&) </li>
 *          <li> lime::SDRDevice::SetGain(uint8_t, lime::TRXDir, uint8_t, lime::eGainTypes, double) </li>
 *          <li> lime::SDRDevice::GetAntenna(uint8_t, lime::TRXDir, uint8_t) </li>
 *          <li> lime::SDRDevice::SetAntenna(uint8_t, lime::TRXDir, uint8_t, uint8_t) </li>
 *          <li> lime::SDRDevice::GetTestSignal(uint8_t, lime::TRXDir, uint8_t) </li>
 *          <li> lime::SDRDevice::SetTestSignal(uint8_t, lime::TRXDir, uint8_t, lime::ChannelConfig::Direction::TestSignal, int16_t, int16_t) </li>
 *          <li> lime::SDRDevice::GetDCOffsetMode(uint8_t, lime::TRXDir, uint8_t) </li>
 *          <li> lime::SDRDevice::SetDCOffsetMode(uint8_t, lime::TRXDir, uint8_t, bool) </li>
 *          <li> lime::SDRDevice::GetDCOffset(uint8_t, lime::TRXDir, uint8_t) </li>
 *          <li> lime::SDRDevice::SetDCOffset(uint8_t, lime::TRXDir, uint8_t, const lime::complex64f_t&) </li>
 *          <li> lime::SDRDevice::GetIQBalance(uint8_t, lime::TRXDir, uint8_t) </li>
 *          <li> lime::SDRDevice::SetIQBalance(uint8_t, lime::TRXDir, uint8_t, const lime::complex64f_t&) </li>
 *       </ul>
 *    </li>
 *    <li><b>Register access:</b>
 *       <ul>
 *          <li> lime::SDRDevice::ReadRegister(uint8_t, unsigned int, bool) </li>
 *          <li> lime::SDRDevice::WriteRegister(uint8_t, unsigned int, unsigned int, bool) </li>
 *       </ul>
 *    </li>
 *    <li><b>Low speed interfaces:</b>
 *       <ul>
 *          <li> lime::SDRDevice::SPI(uint32_t, const uint32_t*, uint32_t*, uint32_t) </li>
 *          <li> lime::SDRDevice::I2CWrite(int, const uint8_t*, uint32_t) </li>
 *          <li> lime::SDRDevice::I2CRead(int, uint8_t*, uint32_t) </li>
 *       </ul>
 *    </li>
 *    <li><b>Utility:</b>
 *       <ul>
 *          <li> lime::SDRDevice::EnableCache(bool) </li>
 *          <li> lime::SDRDevice::GetHardwareTimestamp(uint8_t) </li>
 *          <li> lime::SDRDevice::SetHardwareTimestamp(uint8_t, const uint64_t) </li>
 *       </ul>
 *    </li>
 * @if SPECIAL_API
 *    <li><b>Special:</b>
 *       <ul>
 *          <li> lime::SDRDevice::SetMessageLogCallback(lime::SDRDevice::LogCallbackType) </li>
 *          <li> lime::SDRDevice::GetInternalChip(uint32_t) </li>
 *          <li> lime::SDRDevice::UploadMemory(lime::eMemoryDevice, uint8_t, const char*, size_t, lime::SDRDevice::UploadMemoryCallback) </li>
 *          <li> lime::SDRDevice::MemoryWrite(std::shared_ptr<lime::DataStorage>, lime::Region, const void*) </li>
 *          <li> lime::SDRDevice::MemoryRead(std::shared_ptr<lime::DataStorage>, lime::Region, void*) </li>
 *          <li> lime::SDRDevice::WriteSerialNumber(uint64_t) </li>
 *          <li> lime::SDRDevice::GetGPIOControls() </li>
 *       </ul>
 *    </li>
 * @endif
 * </ul>
 * 
 * @subsection stream_control SDR Device stream control
 * <ul>
 *    <li>lime::RFStream::GetHardwareTimestamp() </li>
 *    <li>lime::RFStream::Setup(const lime::StreamConfig&) </li>
 *    <li>lime::RFStream::GetConfig() </li>
 *    <li>lime::RFStream::Start() </li>
 *    <li>lime::RFStream::StageStart() </li>
 *    <li>lime::RFStream::Stop() </li>
 *    <li>lime::RFStream::Teardown() </li>
 *    <li>lime::RFStream::StreamRx(lime::complex32f_t* const*, uint32_t, lime::StreamMeta*, std::chrono::microseconds) </li>
 *    <li>lime::RFStream::StreamRx(lime::complex16_t* const*, uint32_t, lime::StreamMeta*, std::chrono::microseconds) </li>
 *    <li>lime::RFStream::StreamRx(lime::complex12_t* const*, uint32_t, lime::StreamMeta*, std::chrono::microseconds) </li>
 *    <li>lime::RFStream::StreamTx(const lime::complex32f_t* const*, uint32_t, const lime::StreamMeta*, std::chrono::microseconds) </li>
 *    <li>lime::RFStream::StreamTx(const lime::complex16_t* const*, uint32_t, const lime::StreamMeta*, std::chrono::microseconds) </li>
 *    <li>lime::RFStream::StreamTx(const lime::complex12_t* const*, uint32_t, const lime::StreamMeta*, std::chrono::microseconds) </li>
 *    <li>lime::RFStream::StreamStatus(lime::StreamStats*, lime::StreamStats*) </li>
 * </ul>
 * 
 * @subsection timestamp Timestamp management
 * <ul>
 *    <li>lime::Timespec::AddTicks(int64_t)</li>
 *    <li>lime::Timespec::GetTicks()</li>
 *    <li>lime::Timespec::GetSeconds()</li>
 *    <li>lime::Timespec::GetFracSeconds()</li>
 *    <li>lime::Timespec::GetRealSeconds()</li>
 *    <li>lime::Timespec::SetTickRate(double)</li>
 *    <li>lime::Timespec::GetTickRate()</li>
 *    <li><b>Operators</b>
 *       <ul>
 *          <li>lime::operator==(const lime::Timespec&, const lime::Timespec&)</li>
 *          <li>lime::operator!=(const lime::Timespec&, const lime::Timespec&)</li>
 *          <li>lime::operator<(const lime::Timespec&, const lime::Timespec&)</li>
 *          <li>lime::operator>(const lime::Timespec&, const lime::Timespec&)</li>
 *          <li>lime::operator+(lime::Timespec, const lime::Timespec&)</li>
 *          <li>lime::operator-(lime::Timespec, const lime::Timespec&)</li>
 *          <li>lime::abs(const lime::Timespec&)</li>
 *       </ul>
 *    </li>
 * </ul>
 * 
 * @subsection logging Logger
 * <ul>
 *    <li>lime::registerLogHandler(const lime::LogHandlerCString)</li>
 *    <li>lime::registerLogHandler(const lime::LogHandler)</li>
 *    <li>lime::GetLastErrorMessageCString(void)</li>
 *    <li>lime::GetLastErrorMessage(void)</li>
 *    <li>lime::critical(const char*, ...)</li>
 *    <li>lime::critical(const std::string&)</li>
 *    <li>lime::error(const char*, ...)</li>
 *    <li>lime::error(const std::string&)</li>
 *    <li>lime::warning(const char*, ...)</li>
 *    <li>lime::warning(const std::string&)</li>
 *    <li>lime::info(const char*, ...)</li>
 *    <li>lime::info(const std::string&)</li>
 *    <li>lime::debug(const char*, ...)</li>
 *    <li>lime::debug(const std::string&)</li>
 *    <li>lime::log(const lime::LogLevel, const char*, ...)</li>
 *    <li>lime::log(const lime::LogLevel, const std::string&)</li>
 *    <li>lime::ReportError(const lime::OpStatus)</li>
 *    <li>lime::ReportError(const lime::OpStatus, const char*, ...)</li>
 *    <li>lime::ReportError(const lime::OpStatus, const std::string&)</li>
 *    <li>lime::ReportError(const int, const char*, ...)</li>
 *    <li>lime::ReportError(const int, const std::string&)</li>
 * </ul>
 * 
 * @subsection string_manip String manipulation
 * <ul>
 *    <li>lime::ToString(lime::TRXDir)</li>
 *    <li>lime::ToString(lime::OpStatus)</li>
 *    <li>lime::ToString(lime::eGainTypes)</li>
 *    <li>lime::ToString(lime::eMemoryDevice)</li>
 * </ul>
 * 
 * 
 * @section class Classes
 * 
 * @subsection If_classes Interface classes
 * <ul>
 *    <li>lime::DeviceRegistry</li>
 *    <li>lime::SDRDevice</li>
 *    <li>lime::RFStream</li>
 * </ul>
 * 
 * @subsection strg_classes Storage classes
 * <ul>
 *    <li><b>SDR device description</b>
 *       <ul>
 *          <li>lime::DeviceHandle</li>
 *       </ul>
 *    </li>
 *    <li><b>Stream meta data</b>
 *       <ul>
 *          <li>lime::StreamTxMeta</li>
 *          <li>lime::StreamRxMeta</li>
 *          <li>lime::Timespec</li>
 *       </ul>
 *    </li>
 * </ul>
 * 
 * @section struct Structures
 * 
 * <ul>
 *    <li><b>SDR description</b>
 *       <ul>
 *          <li>lime::SDRDescriptor</li>
 *          <li>lime::DataStorage</li>
 *          <li>lime::CustomParameter</li>
 *          <li>lime::SDRDevice::GPS_Lock</li>
 *       </ul>
 *    </li>
 *    <li><b>SDR device configuration</b>
 *       <ul>
 *          <li>lime::SDRConfig</li>
 *          <li>lime::ChannelConfig</li>
 *          <li>lime::ChannelConfig::Direction</li>
 *          <li>lime::ChannelConfig::Direction::GFIRFilter</li>
 *          <li>lime::ChannelConfig::Direction::TestSignal</li>
 *       </ul>
 *    </li>
 *    <li><b>SDR Stream</b>
 *       <ul>
 *          <li>lime::StreamConfig</li>
 *          <li>lime::StreamConfig::Extras</li>
 *          <li>lime::StreamConfig::Extras::PacketTransmission</li>
 *          <li>lime::StreamStats</li>
 *          <li>lime::StreamStats::FIFOStats</li>
 *          <li>lime::StreamMeta</li>
 *       </ul>
 *    </li>
 *    <li><b>RF SoC description</b>
 *       <ul>
 *          <li>lime::GainValue</li>
 *          <li>lime::RFSOCDescriptor</li>
 *       </ul>
 *    </li>
 *    <li><b>Registers</b>
 *       <ul>
 *          <li>lime::CSRegister</li>
 *          <li>lime::Register</li>
 *       </ul>
 *    </li>
 *    <li><b>Complex types</b>
 *       <ul>
 *          <li>lime::complex64f_t</li>
 *          <li>lime::complex32f_t</li>
 *          <li>lime::complex16_t</li>
 *          <li>lime::complex12_t</li>
 *       </ul>
 *    </li>
 *    <li><b>Utility</b>
 *       <ul>
 *          <li>lime::Range</li>
 *          <li>lime::Region</li>
 *          <li>lime::CustomParameterIO</li>
 *       </ul>
 *    </li> 
 * </ul>
 * 
 * @section enums Enumerations 
 * 
 * <ul>
 *    <li>lime::LogLevel</li>
 *    <li>lime::OpStatus</li>
 *    <li><b>SDR device options</b>
 *       <ul>
 *          <li>lime::SDRDevice::GPS_Lock::LockStatus</li>
 *          <li>lime::TRXDir</li>
 *          <li>lime::DataFormat</li>
 *          <li>lime::eGainTypes</li>
 *          <li>lime::eMemoryDevice</li>
 *       </ul>
 *    </li>
 *    <li><b>SDR stream options</b>
 *       <ul>
 *          <li>lime::TimestampType</li>
 *       </ul>
 *    </li>
 *    <li><b>SDR test signal options:</b>
 *       <ul>
 *          <li> lime::ChannelConfig::Direction::TestSignal::Divide</li>
 *          <li> lime::ChannelConfig::Direction::TestSignal::Scale</li>
 *       </ul>
 *    </li>
 * </ul>
*/

/**
 * @page common_parameters Common public API parameters
 * 
 * @section Device_index Device indexes
 * 
 * Some of the SDR device configuration functions from the @ref api "public API" accept a parameter called <b>moduleIndex</b>. The <b>moduleIndex</b> 
 * parameter specifies an index of a device which will be configured. Typically device index points to RF chip (LMS7002M), but in the case
 * of LimeSDR MMX8 this parameter points to the onboard LimeSDR XTRXs and allows to configure the specific parameter for the
 * individual LimeSDR XTRX board RF chip. 
 *
 * Device indexes always start with index <b>0</b>. The last device on a system is indexed as <b>total device count - 1</b>.
 * 
 * For typical LimeSDR devices this parameter should be set to device index <b>0</b>.
 */