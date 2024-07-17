#pragma once

#include <memory>
#include <string>
#include <vector>

namespace lime {

enum class eDeviceTreeNodeClass {
    ADF4002,
    CDCM6208,
    FPGA,
    FPGA_MINI,
    FPGA_X3,
    FPGA_XTRX,
    LMS7002M,
    SDRDevice,
    Equalizer,
};

/// @brief Structure describing a device node in the device node tree.
struct DeviceTreeNode {
    /// @brief Default constructor for the node.
    DeviceTreeNode(){};

    /// @brief The constructor for the device node.
    /// @param name The name of the node.
    /// @param nodeClass The device class of the node.
    /// @param ptr The pointer to the device.
    DeviceTreeNode(const std::string& name, eDeviceTreeNodeClass nodeClass, void* ptr)
        : name(name)
        , DeviceTreeNodeClass(nodeClass)
        , ptr(ptr)
    {
    }
    std::string name; ///< The name of the node.
    eDeviceTreeNodeClass DeviceTreeNodeClass; ///< The device class of the node.
    void* ptr; ///< The pointer to the device.
    std::vector<std::shared_ptr<DeviceTreeNode>> children; ///< The children of this node in the device tree.
};

} // namespace lime
