#ifndef LIMESUITENG_SDRDESCRIPTOR_H
#define LIMESUITENG_SDRDESCRIPTOR_H

#include <string>
#include <map>
#include <memory>

#include "limesuiteng/RFSOCDescriptor.h"

namespace lime {

class SDRDevice;
struct DeviceTreeNode;

/// @brief Structure for the information of a custom parameter.
struct CustomParameter {
    std::string name; ///< The name of the custom parameter
    int32_t id; ///< The identifier of the custom parameter
    int32_t minValue; ///< The minimum possible value of the custom parameter
    int32_t maxValue; ///< The maximum possible value of the custom parameter
    bool readOnly; ///< Denotes whether this value is read only or not
};

/// @brief Describes a data storage of a certain type a device holds.
struct DataStorage {
    SDRDevice* ownerDevice; ///< Pointer to the device that actually owns the data storage
    eMemoryDevice memoryDeviceType; ///< The type of memory being described
    std::unordered_map<std::string, Region> regions; ///< The documented memory regions of the data storage

    /// @brief Constructs a new Data Storage object
    /// @param device The device this storage belongs to.
    /// @param type The type of memory being described in this object.
    /// @param regions The memory regions this memory contains.
    DataStorage(SDRDevice* device = nullptr,
        eMemoryDevice type = eMemoryDevice::COUNT,
        std::unordered_map<std::string, Region> regions = {})
        : ownerDevice(device)
        , memoryDeviceType(type)
        , regions(regions)
    {
    }
};

/// @brief General information about device internals, static capabilities.
struct SDRDescriptor {
    std::string name; ///< The displayable name for the device
    /*! The displayable name for the expansion card
    * Ex: if the RFIC is on a daughter-card.
    */
    std::string expansionName;
    std::string firmwareVersion; ///< The firmware version as a string
    std::string gatewareVersion; ///< Gateware version as a string
    std::string gatewareRevision; ///< Gateware revision as a string
    std::string gatewareTargetBoard; ///< Which board should use this gateware
    std::string hardwareVersion; ///< The hardware version as a string
    std::string protocolVersion; ///< The protocol version as a string
    uint64_t serialNumber{ 0 }; ///< A unique board serial number

    std::map<std::string, uint32_t> spiSlaveIds; ///< Names and SPI bus numbers of internal chips
    std::vector<RFSOCDescriptor> rfSOC; ///< Descriptors of all RFSoC devices within this device
    std::vector<CustomParameter> customParameters; ///< Descriptions of all custom parameters of this device
    /** Descriptions of all memory storage devices on this device */
    std::map<std::string, std::shared_ptr<DataStorage>> memoryDevices;
    std::shared_ptr<DeviceTreeNode> socTree; ///< The device's subdevices tree view representation
};

} // namespace lime

#endif
