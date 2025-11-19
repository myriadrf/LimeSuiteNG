#pragma once

#include <memory>
#include <string>
#include <vector>

namespace lime {

/// @brief Structure describing a device node in the device node tree.
struct DeviceTreeNode {
    /// @brief Default constructor for the node.
    DeviceTreeNode(){};

    /// @brief The constructor for the device node.
    /// @param name The name of the node.
    /// @param nodeClass The device class of the node.
    /// @param ptr The pointer to the device.
    DeviceTreeNode(void* module, std::string_view moduleClass, const std::string& name = "")
        : module(module)
        , moduleClass(moduleClass)
        , name(name)
    {
    }

    void* module; ///< The pointer to the device module.
    std::string moduleClass; ///< unique identifier of device class type
    std::string name; ///< user friendly name, name on the schematic, etc...
    std::vector<std::shared_ptr<DeviceTreeNode>> children; ///< The children of this node in the device tree.
};

} // namespace lime
