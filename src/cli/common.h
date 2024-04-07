#ifndef LIMESUITENG_CLI_COMMON_H
#define LIMESUITENG_CLI_COMMON_H

#include <chrono>
#include <string>
#include <vector>
#include <stdint.h>
#include <iostream>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <signal.h>

#include "limesuiteng/DeviceHandle.h"
#include "limesuiteng/DeviceRegistry.h"
#include "limesuiteng/SDRDevice.h"
#include "limesuiteng/SDRDescriptor.h"

lime::SDRDevice* ConnectToFilteredOrDefaultDevice(const std::string& argument);

#endif