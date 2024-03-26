#pragma once

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

lime::SDRDevice* ConnectUsingNameHint(const std::string& name);
