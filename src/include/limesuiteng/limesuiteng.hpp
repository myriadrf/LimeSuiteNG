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
 * through public API.
 * 
 * <b>Warning:</b> All public API members are part of namespace <b>lime</b>. To access individual members use scope resolution operator <b>lime::function()</b>.
 * 
 * @section funcs Functions 
 * 
 * @subsection registration SDR Device registration
 * <ul>
 *    <li> @ref lime::DeviceRegistry::enumerate() "enumerate()" </li>
 *    <li> @ref lime::DeviceRegistry::enumerate(const lime::DeviceHandle&) "enumerate(const DeviceHandle&)"</li>
 *    <li> @ref lime::DeviceRegistry::makeDevice(const lime::DeviceHandle&) "makeDevice()"</li>
 *    <li> @ref lime::DeviceRegistry::freeDevice(lime::SDRDevice*) "freeDevice()"</li>
 *    <li> @ref lime::DeviceRegistry::moduleNames(void) "moduleNames()"</li>
 * </ul>
 * 
 * @subsection control_adn_config SDR Device control and configuration
 * <ul>
 *    <li> @ref lime::SDRDevice::Init() "Init()"</li>
 *    <li> @ref lime::SDRDevice::Reset() "Reset()"</li>
 *    <li> @ref lime::SDRDevice::EnableChannel(uint8_t, lime::TRXDir, uint8_t, bool) "EnableChannel()"</li>
 *    <li> @ref lime::SDRDevice::Calibrate(uint8_t, lime::TRXDir, uint8_t, double) "Calibrate()"</li>
 *    <li> @ref lime::SDRDevice::StreamCreate(const lime::StreamConfig&, uint8_t) "StreamCreate()"</li>
 *    <li><b>Configuration:</b>
 *       <ul>
 *          <li> @ref lime::SDRDevice::Configure(const lime::SDRConfig&, uint8_t) "Configure()"</li>
 *          <li> @ref lime::SDRDevice::LoadConfig(uint8_t, const std::string&) "LoadConfig()"</li>
 *          <li> @ref lime::SDRDevice::SaveConfig(uint8_t, const std::string&) "SaveConfig()"</li>
 *          <li> @ref lime::SDRDevice::GetParameter(uint8_t, uint8_t, const std::string&) "GetParameter(const string& parameterKey)"</li>
 *          <li> @ref lime::SDRDevice::SetParameter(uint8_t, uint8_t, const std::string&, uint16_t) "SetParameter(const string& parameterKey, uint8_t msb, uint8_t lsb, uint16_t value)"</li>
 *          <li> @ref lime::SDRDevice::GetParameter(uint8_t, uint8_t, uint16_t, uint8_t, uint8_t) "GetParameter(uint16_t address)"</li>
 *          <li> @ref lime::SDRDevice::SetParameter(uint8_t, uint8_t, uint16_t, uint8_t, uint8_t, uint16_t) "SetParameter(uint16_t address, uint8_t msb, uint8_t lsb, uint16_t value)"</li>
 *          <li> @ref lime::SDRDevice::ReadRegister(uint8_t, unsigned int, bool) "ReadRegister()"</li>
 *          <li> @ref lime::SDRDevice::WriteRegister(uint8_t, unsigned int, unsigned int, bool) "WriteRegister()"</li>
 *       </ul>
 *    </li>
 *    <li> <b> Filter configuration:</b>
 *       <ul>
 *        <li> @ref lime::SDRDevice::ConfigureGFIR(uint8_t, lime::TRXDir, uint8_t, lime::ChannelConfig::Direction::GFIRFilter) "ConfigureGFIR()"</li>
 *        <li> @ref lime::SDRDevice::GetGFIRCoefficients(uint8_t, lime::TRXDir, uint8_t, uint8_t) "GetGFIRCoefficients()"</li>
 *        <li> @ref lime::SDRDevice::SetGFIRCoefficients(uint8_t, lime::TRXDir, uint8_t, uint8_t, std::vector<double>) "SetGFIRCoefficients()"</li>
 *        <li> @ref lime::SDRDevice::SetGFIR(uint8_t, lime::TRXDir, uint8_t, uint8_t, bool) "SetGFIR()"</li>
 *        <li> @ref lime::SDRDevice::GetLowPassFilter(uint8_t, lime::TRXDir, uint8_t) "GetLowPassFilter()"</li>
 *        <li> @ref lime::SDRDevice::SetLowPassFilter(uint8_t, lime::TRXDir, uint8_t, double) "SetLowPassFilter()"</li>
 *       </ul>
 *    </li>
 *    <li><b> Additional info:</b>
 *       <ul>
 *          <li> @ref lime::SDRDevice::GetDescriptor() "GetDescriptor()"</li>
 *          <li> @ref lime::SDRDevice::GetGPSLock(lime::SDRDevice::GPS_Lock*) "GetGPSLock()"</li>
 *          <li> @ref lime::SDRDevice::GetCGENLocked(uint8_t) "GetCGENLocked()"</li>
 *          <li> @ref lime::SDRDevice::GetTemperature(uint8_t) "GetTemperature()"</li>
 *          <li> @ref lime::SDRDevice::GetSXLocked(uint8_t, lime::TRXDir) "GetSXLocked()"</li>
 *       </ul>
 *    </li>
 *    <li><b>Configuration by parameter:</b>
 *       <ul>
 *          <li> @ref lime::SDRDevice::GetClockFreq(uint8_t, uint8_t) "GetClockFreq()"</li>
 *          <li> @ref lime::SDRDevice::SetClockFreq(uint8_t,double,uint8_t) "SetClockFreq()"</li>
 *          <li> @ref lime::SDRDevice::GetFrequency(uint8_t,lime::TRXDir,uint8_t) "GetFrequency()"</li>
 *          <li> @ref lime::SDRDevice::SetFrequency(uint8_t, lime::TRXDir, uint8_t, double) "SetFrequency()"</li>
 *          <li> @ref lime::SDRDevice::GetNCOFrequency(uint8_t, lime::TRXDir, uint8_t, uint8_t, double&) "GetNCOFrequency()"</li>
 *          <li> @ref lime::SDRDevice::SetNCOFrequency(uint8_t, lime::TRXDir, uint8_t, uint8_t, double, double) "SetNCOFrequency()"</li>
 *          <li> @ref lime::SDRDevice::GetNCOOffset(uint8_t, lime::TRXDir, uint8_t) "GetNCOOffset()"</li>
 *          <li> @ref lime::SDRDevice::GetNCOIndex(uint8_t, lime::TRXDir, uint8_t) "GetNCOIndex()"</li>
 *          <li> @ref lime::SDRDevice::SetNCOIndex(uint8_t, lime::TRXDir, uint8_t, uint8_t, bool) "SetNCOIndex()"</li>
 *          <li> @ref lime::SDRDevice::GetSampleRate(uint8_t, lime::TRXDir, uint8_t, uint32_t*) "GetSampleRate()"</li>
 *          <li> @ref lime::SDRDevice::SetSampleRate(uint8_t, lime::TRXDir, uint8_t, double, uint8_t) "SetSampleRate()"</li>
 *          <li> @ref lime::SDRDevice::GetGain(uint8_t, lime::TRXDir, uint8_t, lime::eGainTypes, double&) "GetGain()"</li>
 *          <li> @ref lime::SDRDevice::SetGain(uint8_t, lime::TRXDir, uint8_t, lime::eGainTypes, double) "SetGain()"</li>
 *          <li> @ref lime::SDRDevice::GetAntenna(uint8_t, lime::TRXDir, uint8_t) "GetAntenna()"</li>
 *          <li> @ref lime::SDRDevice::SetAntenna(uint8_t, lime::TRXDir, uint8_t, uint8_t) "SetAntenna()"</li>
 *          <li> @ref lime::SDRDevice::GetTestSignal(uint8_t, lime::TRXDir, uint8_t) "GetTestSignal()"</li>
 *          <li> @ref lime::SDRDevice::SetTestSignal(uint8_t, lime::TRXDir, uint8_t, lime::ChannelConfig::Direction::TestSignal, int16_t, int16_t) "SetTestSignal()"</li>
 *          <li> @ref lime::SDRDevice::GetDCOffsetMode(uint8_t, lime::TRXDir, uint8_t) "GetDCOffsetMode()"</li>
 *          <li> @ref lime::SDRDevice::SetDCOffsetMode(uint8_t, lime::TRXDir, uint8_t, bool) "SetDCOffsetMode()"</li>
 *          <li> @ref lime::SDRDevice::GetDCOffset(uint8_t, lime::TRXDir, uint8_t) "GetDCOffset()"</li>
 *          <li> @ref lime::SDRDevice::SetDCOffset(uint8_t, lime::TRXDir, uint8_t, const lime::complex64f_t&) "SetDCOffset()"</li>
 *          <li> @ref lime::SDRDevice::GetIQBalance(uint8_t, lime::TRXDir, uint8_t) "GetIQBalance()"</li>
 *          <li> @ref lime::SDRDevice::SetIQBalance(uint8_t, lime::TRXDir, uint8_t, const lime::complex64f_t&) "SetIQBalance()"</li>
 *       </ul>
 *    </li>
 *    <li><b>Low speed interfaces:</b>
 *       <ul>
 *          <li> @ref lime::SDRDevice::SPI(uint32_t, const uint32_t*, uint32_t*, uint32_t) "SPI()"</li>
 *          <li> @ref lime::SDRDevice::I2CWrite(int, const uint8_t*, uint32_t) "I2CWrite()"</li>
 *          <li> @ref lime::SDRDevice::I2CRead(int, uint8_t*, uint32_t) "I2CRead()"</li>
 *       </ul>
 *    </li>
 *    <li><b>Utility:</b>
 *       <ul>
 *          <li> @ref lime::SDRDevice::UploadTxWaveform(const lime::StreamConfig&, uint8_t, const void**, uint32_t) "UploadTxWaveform()"</li>
 *          <li> @ref lime::SDRDevice::EnableCache(bool) "EnableCache()"</li>
 *          <li> @ref lime::SDRDevice::Synchronize(bool) "Synchronize()"</li>
 *          <li> @ref lime::SDRDevice::GetHardwareTimestamp(uint8_t) "GetHardwareTimestamp()"</li>
 *          <li> @ref lime::SDRDevice::SetHardwareTimestamp(uint8_t, const uint64_t) "SetHardwareTimestamp()"</li>
 *       </ul>
 *    </li>
 * @if SPECIAL_API
 *    <li><b>Special:</b>
 *       <ul>
 *          <li> @ref lime::SDRDevice::SetMessageLogCallback(lime::SDRDevice::LogCallbackType) "SetMessageLogCallback()"</li>
 *          <li> @ref lime::SDRDevice::GetInternalChip(uint32_t) "GetInternalChip()"</li>
 *          <li> @ref lime::SDRDevice::UploadMemory(lime::eMemoryDevice, uint8_t, const char*, size_t, lime::SDRDevice::UploadMemoryCallback) "UploadMemory()"</li>
 *          <li> @ref lime::SDRDevice::MemoryWrite(std::shared_ptr<lime::DataStorage>, lime::Region, const void*) "MemoryWrite()"</li>
 *          <li> @ref lime::SDRDevice::MemoryRead(std::shared_ptr<lime::DataStorage>, lime::Region, void*) "MemoryRead()"</li>
 *          <li> @ref lime::SDRDevice::WriteSerialNumber(uint64_t) "WriteSerialNumber()"</li>
 *          <li> @ref lime::SDRDevice::GetGPIOControls() "GetGPIOControls()"</li>
 *       </ul>
 *    </li>
 * @endif
 * </ul>
 * 
 * @subsection stream_control SDR Device stream control
 * <ul>
 *    <li> @ref lime::RFStream::GetHardwareTimestamp() "GetHardwareTimestamp()"</li>
 *    <li> @ref lime::RFStream::Setup(const lime::StreamConfig&) "Setup()"</li>
 *    <li> @ref lime::RFStream::GetConfig() "GetConfig()"</li>
 *    <li> @ref lime::RFStream::Start() "Start()"</li>
 *    <li> @ref lime::RFStream::StageStart() "StageStart()"</li>
 *    <li> @ref lime::RFStream::Stop() "Stop()"</li>
 *    <li> @ref lime::RFStream::Teardown() "Teardown()"</li>
 *    <li> @ref lime::RFStream::StreamStatus(lime::StreamStats*, lime::StreamStats*) "StreamStatus()"</li>
 *    <li> @ref lime::RFStream::Receive(lime::complex32f_t* const*, uint32_t , lime::StreamRxMeta*) "Receive(complex32f_t* const* samples)"</li>
 *    <li> @ref lime::RFStream::Receive(lime::complex16_t* const*, uint32_t , lime::StreamRxMeta*) "Receive(complex16_t* const* samples)"</li>
 *    <li> @ref lime::RFStream::Receive(lime::complex12_t* const*, uint32_t , lime::StreamRxMeta*) "Receive(complex12_t* const* samples)"</li>
 *    <li> @ref lime::RFStream::Transmit(const lime::complex32f_t* const*, uint32_t, const lime::StreamTxMeta*) "Transmit(complex32f_t* const* samples)"</li>
 *    <li> @ref lime::RFStream::Transmit(const lime::complex16_t* const*, uint32_t, const lime::StreamTxMeta*) "Transmit(complex16_t* const* samples)"</li>
 *    <li> @ref lime::RFStream::Transmit(const lime::complex12_t* const*, uint32_t, const lime::StreamTxMeta*) "Transmit(complex12_t* const* samples)"</li>
 *    <li><b>Deprecated</b> 
 *       <ul>
 *          <li> @ref lime::RFStream::StreamRx(lime::complex32f_t* const*, uint32_t, lime::StreamMeta*, std::chrono::microseconds) "StreamRx(complex32f_t* const* samples)"</li>
 *          <li> @ref lime::RFStream::StreamRx(lime::complex16_t* const*, uint32_t, lime::StreamMeta*, std::chrono::microseconds) "StreamRx(complex16_t* const* samples)"</li>
 *          <li> @ref lime::RFStream::StreamRx(lime::complex12_t* const*, uint32_t, lime::StreamMeta*, std::chrono::microseconds) "StreamRx(complex12_t* const* samples)"</li>
 *          <li> @ref lime::RFStream::StreamTx(const lime::complex32f_t* const*, uint32_t, const lime::StreamMeta*, std::chrono::microseconds) "StreamTx(complex32f_t* const* samples)"</li>
 *          <li> @ref lime::RFStream::StreamTx(const lime::complex16_t* const*, uint32_t, const lime::StreamMeta*, std::chrono::microseconds) "StreamTx(complex16_t* const* samples)"</li>
 *          <li> @ref lime::RFStream::StreamTx(const lime::complex12_t* const*, uint32_t, const lime::StreamMeta*, std::chrono::microseconds) "StreamTx(complex12_t* const* samples)"</li>
 *       </ul>
 *    </li>
 * </ul>
 * 
 * @subsection timestamp Timestamp management
 * <ul>
 *    <li> @ref lime::Timespec::AddTicks(int64_t) "AddTicks()"</li>
 *    <li> @ref lime::Timespec::GetTicks() "GetTicks()"</li>
 *    <li> @ref lime::Timespec::GetSeconds() "GetSeconds()"</li>
 *    <li> @ref lime::Timespec::GetFracSeconds() "GetFracSeconds()"</li>
 *    <li> @ref lime::Timespec::GetRealSeconds() "GetRealSeconds()"</li>
 *    <li> @ref lime::Timespec::SetTickRate(double) "SetTickRate()"</li>
 *    <li> @ref lime::Timespec::GetTickRate() "GetTickRate()"</li>
 *    <li><b>Operators</b>
 *       <ul>
 *          <li> @ref lime::operator==(const lime::Timespec&, const lime::Timespec&) "operator==()"</li>
 *          <li> @ref lime::operator!=(const lime::Timespec&, const lime::Timespec&) "operator!=()"</li>
 *          <li> @ref lime::operator<(const lime::Timespec&, const lime::Timespec&) "operator<()"</li>
 *          <li> @ref lime::operator>(const lime::Timespec&, const lime::Timespec&) "operator>()"</li>
 *          <li> @ref lime::operator+(lime::Timespec, const lime::Timespec&) "operator+()"</li>
 *          <li> @ref lime::operator-(lime::Timespec, const lime::Timespec&) "operator-()"</li>
 *          <li> @ref lime::abs(const lime::Timespec&) "abs()"</li>
 *       </ul>
 *    </li>
 * </ul>
 * 
 * @subsection logging Logger
 * <ul>
 *    <li> @ref lime::registerLogHandler(const lime::LogHandlerCString) "registerLogHandler(const LogHandlerCString)()"</li>
 *    <li> @ref lime::registerLogHandler(const lime::LogHandler) "registerLogHandler(const LogHandler)()"</li>
 *    <li> @ref lime::GetLastErrorMessageCString(void) "GetLastErrorMessageCString()"</li>
 *    <li> @ref lime::GetLastErrorMessage(void) "GetLastErrorMessage()"</li>
 *    <li> @ref lime::critical(const char*, ...) "critical(const char*, ...)"</li>
 *    <li> @ref lime::critical(const std::string&) "critical(const string&)"</li>
 *    <li> @ref lime::error(const char*, ...) "error(const char*, ...)"</li>
 *    <li> @ref lime::error(const std::string&) "error(const string&)"</li>
 *    <li> @ref lime::warning(const char*, ...) "warning(const char*, ...)"</li>
 *    <li> @ref lime::warning(const std::string&) "warning(const string&)"</li>
 *    <li> @ref lime::info(const char*, ...) "info(const char*, ...)"</li>
 *    <li> @ref lime::info(const std::string&) "info(const string&)"</li>
 *    <li> @ref lime::debug(const char*, ...) "debug(const char*, ...)"</li>
 *    <li> @ref lime::debug(const std::string&) "debug(const string&)"</li>
 *    <li> @ref lime::log(const lime::LogLevel, const char*, ...) "log(const LogLevel, const char*, ...)"</li>
 *    <li> @ref lime::log(const lime::LogLevel, const std::string&) "log(const LogLevel, const string&)"</li>
 *    <li> @ref lime::ReportError(const lime::OpStatus) "ReportError(const OpStatus)"</li>
 *    <li> @ref lime::ReportError(const lime::OpStatus, const char*, ...) "ReportError(const OpStatus, const char*, ...)"</li>
 *    <li> @ref lime::ReportError(const lime::OpStatus, const std::string&) "ReportError(const OpStatus, const string&)"</li>
 *    <li> @ref lime::ReportError(const int, const char*, ...) "ReportError(const int, const char*, ...)"</li>
 *    <li> @ref lime::ReportError(const int, const std::string&) "ReportError(const int, const string&)"</li>
 * </ul>
 * 
 * @subsection string_manip String manipulation
 * <ul>
 *    <li> @ref lime::ToString(lime::TRXDir) "ToString(TRXDir)"</li>
 *    <li> @ref lime::ToString(lime::OpStatus) "ToString(OpStatus)"</li>
 *    <li> @ref lime::ToString(lime::eGainTypes) "ToString(eGainTypes)"</li>
 *    <li> @ref lime::ToString(lime::eMemoryDevice) "ToString(eMemoryDevice)"</li>
 * </ul>
 * 
 * 
 * @section class Classes
 * 
 * @subsection If_classes Interface classes
 * <ul>
 *    <li> @ref lime::DeviceRegistry "DeviceRegistry"</li>
 *    <li> @ref lime::SDRDevice "SDRDevice"</li>
 *    <li> @ref lime::RFStream "RFStream"</li>
 * </ul>
 * 
 * @subsection strg_classes Storage classes
 * <ul>
 *    <li><b>SDR device description</b>
 *       <ul>
 *          <li> @ref lime::DeviceHandle "DeviceHandle"</li>
 *       </ul>
 *    </li>
 *    <li><b>Stream meta data</b>
 *       <ul>
 *          <li> @ref lime::StreamTxMeta "StreamTxMeta"</li>
 *          <li> @ref lime::StreamRxMeta "StreamRxMeta"</li>
 *          <li> @ref lime::Timespec "Timespec"</li>
 *       </ul>
 *    </li>
 * </ul>
 * 
 * @section struct Structures
 * 
 * <ul>
 *    <li><b>SDR description</b>
 *       <ul>
 *          <li> @ref lime::SDRDescriptor "SDRDescriptor"</li>
 *          <li> @ref lime::DataStorage "DataStorage"</li>
 *          <li> @ref lime::CustomParameter "CustomParameter"</li>
 *          <li> @ref lime::SDRDevice::GPS_Lock "GPS_Lock"</li>
 *       </ul>
 *    </li>
 *    <li><b>SDR device configuration</b>
 *       <ul>
 *          <li> @ref lime::SDRConfig "SDRConfig"</li>
 *          <li> @ref lime::ChannelConfig "ChannelConfig"</li>
 *          <li> @ref lime::ChannelConfig::Direction "Direction"</li>
 *          <li> @ref lime::ChannelConfig::Direction::GFIRFilter "GFIRFilter"</li>
 *          <li> @ref lime::ChannelConfig::Direction::TestSignal "TestSignal"</li>
 *       </ul>
 *    </li>
 *    <li><b>SDR Stream</b>
 *       <ul>
 *          <li> @ref lime::StreamConfig "StreamConfig"</li>
 *          <li> @ref lime::StreamConfig::Extras "Extras"</li>
 *          <li> @ref lime::StreamConfig::Extras::PacketTransmission "PacketTransmission"</li>
 *          <li> @ref lime::StreamStats "StreamStats"</li>
 *          <li> @ref lime::StreamStats::FIFOStats "FIFOStats"</li>
 *          <li> @ref lime::StreamMeta "StreamMeta"</li>
 *       </ul>
 *    </li>
 *    <li><b>RF SoC description</b>
 *       <ul>
 *          <li> @ref lime::GainValue "GainValue"</li>
 *          <li> @ref lime::RFSOCDescriptor "RFSOCDescriptor"</li>
 *       </ul>
 *    </li>
 *    <li><b>Registers</b>
 *       <ul>
 *          <li> @ref lime::CSRegister "CSRegister"</li>
 *          <li> @ref lime::Register "Register"</li>
 *       </ul>
 *    </li>
 *    <li><b>Complex types</b>
 *       <ul>
 *          <li> @ref lime::complex64f_t "complex64f_t"</li>
 *          <li> @ref lime::complex32f_t "complex32f_t"</li>
 *          <li> @ref lime::complex16_t "complex16_t"</li>
 *          <li> @ref lime::complex12_t "complex12_t"</li>
 *       </ul>
 *    </li>
 *    <li><b>Utility</b>
 *       <ul>
 *          <li> @ref lime::Range "Range"</li>
 *          <li> @ref lime::Region "Region"</li>
 *          <li> @ref lime::CustomParameterIO "CustomParameterIO"</li>
 *       </ul>
 *    </li> 
 * </ul>
 * 
 * @section enums Enumerations 
 * 
 * <ul>
 *    <li> @ref lime::LogLevel "LogLevel"</li>
 *    <li> @ref lime::OpStatus "OpStatus"</li>
 *    <li><b>SDR device options</b>
 *       <ul>
 *          <li> @ref lime::SDRDevice::GPS_Lock::LockStatus "LockStatus"</li>
 *          <li> @ref lime::TRXDir "TRXDir"</li>
 *          <li> @ref lime::DataFormat "DataFormat"</li>
 *          <li> @ref lime::eGainTypes "eGainTypes"</li>
 *          <li> @ref lime::eMemoryDevice "eMemoryDevice</li>
 *       </ul>
 *    </li>
 *    <li><b>SDR stream options</b>
 *       <ul>
 *          <li> @ref lime::TimestampType "TimestampType"</li>
 *       </ul>
 *    </li>
 *    <li><b>SDR test signal options:</b>
 *       <ul>
 *          <li> @ref lime::ChannelConfig::Direction::TestSignal::Divide "Divide"</li>
 *          <li> @ref lime::ChannelConfig::Direction::TestSignal::Scale "Scale"</li>
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
 * Parameter <b>moduleIndex</b> should always be set to device index <b>0</b> for LimeSDR devices that use a single RF chip.
 */