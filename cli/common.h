#ifndef LIMESUITENG_CLI_COMMON_H
#define LIMESUITENG_CLI_COMMON_H

#include <chrono>
#include <vector>
#include <stdint.h>
#include <iostream>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <signal.h>
#include <string_view>

#include "limesuiteng/DeviceHandle.h"
#include "limesuiteng/DeviceRegistry.h"
#include "limesuiteng/SDRDevice.h"
#include "limesuiteng/SDRDescriptor.h"
#include "limesuiteng/Logger.h"

#include "DeviceTreeNode.h"

lime::SDRDevice* ConnectToFilteredOrDefaultDevice(const std::string_view argument);

template<class T>
void CollectNodesByClass(const lime::DeviceTreeNode& parentNode, lime::eDeviceTreeNodeClass nodeType, std::vector<T*>& results)
{
    if (parentNode.DeviceTreeNodeClass == nodeType)
        results.push_back(reinterpret_cast<T*>(parentNode.ptr));

    for (const auto& node : parentNode.children)
        CollectNodesByClass(*node, nodeType, results);
}

template<class T> std::vector<T*> GetNodeByClass(lime::SDRDevice& dev, lime::eDeviceTreeNodeClass nodeType)
{
    std::vector<T*> foundNodes;
    CollectNodesByClass(*dev.GetDescriptor().socTree.get(), nodeType, foundNodes);
    return foundNodes;
}

int32_t GetChipSelectByName(lime::SDRDevice* device, const std::string_view chipName);

#endif
