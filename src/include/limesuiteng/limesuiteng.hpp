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
 * @code
 * #include <limesuiteng/limesuiteng.hpp>
 * @endcode
 * 
 * Link your application against limesuiteng library in cmake:
 * 
 * @code{.cmake}
 * find_package(limesuiteng REQUIRED)
 * @endcode
 * 
 * To get started using LimeSuiteNG library public API, visit Topics page. For a full list of public API members, visit @ref api "API" page.
 */

 
/**
 * @page api Application Programming Interface
 * 
 * <p> This page provides a complete list of functions, enumerations, structures that can be accessed through public API:</p>
 * 
 * <h1> Functions </h1>
 * <h3> SDR Device registration </h3>
 * <ul>
 *    <li> lime::DeviceRegistry::enumerate() </li>
 *    <li> lime::DeviceRegistry::enumerate(const lime::DeviceHandle&) </li>
 *    <li> lime::DeviceRegistry::makeDevice(const lime::DeviceHandle&) </li>
 *    <li> lime::DeviceRegistry::freeDevice(lime::SDRDevice*) </li>
 *    <li> lime::DeviceRegistry::moduleNames(void) </li>
 * </ul>
 * 
 * <h3> SDR Device control and configuration </h3>
 * <ul>
 *    <li> lime::SDRDevice::Configure(const lime::SDRConfig&, uint8_t) </li>
 *    <li> lime::SDRDevice::GetDescriptor() </li>
 *    <li> lime::SDRDevice::Init() </li>
 *    <li> lime::SDRDevice::Reset() </li>
 *    <li> lime::SDRDevice::GetGPSLock(lime::SDRDevice::GPS_Lock*) </li>
 *    <li> lime::SDRDevice::EnableChannel(uint8_t, lime::TRXDir, uint8_t, bool) </li>
 *    <li> lime::SDRDevice::GetClockFreq(uint8_t, uint8_t) </li>
 *    <li> lime::SDRDevice::SetClockFreq(uint8_t,double,uint8_t) </li>
 *    <li> lime::SDRDevice::GetFrequency(uint8_t,lime::TRXDir,uint8_t) </li>
 *    <li> lime::SDRDevice::SetFrequency(uint8_t, lime::TRXDir, uint8_t, double) </li>
 *    <li> lime::SDRDevice::GetNCOFrequency(uint8_t, lime::TRXDir, uint8_t, uint8_t, double&) </li>
 *    <li> lime::SDRDevice::SetNCOFrequency(uint8_t, lime::TRXDir, uint8_t, uint8_t, double, double) </li>
 *    <li> lime::SDRDevice::GetNCOOffset(uint8_t, lime::TRXDir, uint8_t) </li>
 *    <li> lime::SDRDevice::GetNCOIndex(uint8_t, lime::TRXDir, uint8_t) </li>
 *    <li> lime::SDRDevice::SetNCOIndex(uint8_t, lime::TRXDir, uint8_t, uint8_t, bool) </li>
 *    <li> lime::SDRDevice::GetSampleRate(uint8_t, lime::TRXDir, uint8_t, uint32_t*) </li>
 *    <li> lime::SDRDevice::SetSampleRate(uint8_t, lime::TRXDir, uint8_t, double, uint8_t) </li>
 *    <li> lime::SDRDevice::GetGain(uint8_t, lime::TRXDir, uint8_t, lime::eGainTypes, double&) </li>
 *    <li> lime::SDRDevice::SetGain(uint8_t, lime::TRXDir, uint8_t, lime::eGainTypes, double) </li>
 *    <li> lime::SDRDevice::GetLowPassFilter(uint8_t, lime::TRXDir, uint8_t) </li>
 *    <li> lime::SDRDevice::SetLowPassFilter(uint8_t, lime::TRXDir, uint8_t, double) </li>
 *    <li> lime::SDRDevice::GetAntenna(uint8_t, lime::TRXDir, uint8_t) </li>
 *    <li> lime::SDRDevice::SetAntenna(uint8_t, lime::TRXDir, uint8_t, uint8_t) </li>
 *    <li> lime::SDRDevice::GetTestSignal(uint8_t, lime::TRXDir, uint8_t) </li>
 *    <li> lime::SDRDevice::GetTestSignal(uint8_t, lime::TRXDir, uint8_t, lime::ChannelConfig::Direction::TestSignal, int16_t, int16_t) </li>
 *    <li> lime::SDRDevice::GetDCOffsetMode(uint8_t, lime::TRXDir, uint8_t) </li>
 *    <li> lime::SDRDevice::SetDCOffsetMode(uint8_t, lime::TRXDir, uint8_t, bool) </li>
 *    <li> lime::SDRDevice::GetDCOffset(uint8_t, lime::TRXDir, uint8_t) </li>
 *    <li> lime::SDRDevice::SetDCOffset(uint8_t, lime::TRXDir, uint8_t, const lime::complex64f_t&) </li>
 *    <li> lime::SDRDevice::GetIQBalance(uint8_t, lime::TRXDir, uint8_t) </li>
 *    <li> lime::SDRDevice::SetIQBalance(uint8_t, lime::TRXDir, uint8_t, const complex64f_t&) </li>
 *    <li> lime::SDRDevice::GetCGENLocked(uint8_t) </li>
 *    <li> lime::SDRDevice::GetTemperature(uint8_t) </li>
 *    <li> lime::SDRDevice::GetSXLocked(uint8_t, lime::TRXDir) </li>
 *    <li> lime::SDRDevice::ReadRegister(uint8_t, unsigned int, bool) </li>
 *    <li> lime::SDRDevice::WriteRegister(uint8_t, unsigned int, bool) </li>
 * </ul>
 * 
 * <h3> Device chip control </h3>
 * <ul>
 *    <li> </li>
 * </ul>
 * 
 * <h1> Structures </h1>
 * 
 * <h1> Enumerations </h1>
*/