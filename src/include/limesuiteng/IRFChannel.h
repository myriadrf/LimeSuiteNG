#ifndef LIMESUITENG_IRFCHANNEL_H
#define LIMESUITENG_IRFCHANNEL_H

#include "limesuiteng/types.h"

#include <cstdint>

namespace lime {

class ILPF;
class INCO;
class IGFIR;
class ILOMixer;
class IDCCorrector;
class IOversampler;
class IQuadratureErrorCorrector;
class IToneGenerator;
class IRFPath;
class IGainStage;

/// @brief A single Rx or Tx channel of an RF SoC, giving access to the building blocks of its signal path.
///
/// The returned objects are owned by the RF SoC and remain valid for as long as it exists.
/// The getters return nullptr if the channel does not have the requested block.
/// Blocks that are shared between channels (e.g. a local oscillator) are returned as the same object by each channel.
class IRFChannel
{
  public:
    virtual ~IRFChannel() = default;

    virtual TRXDir GetDirection() const = 0;
    /// @return The index of this channel within the RF SoC.
    virtual uint8_t GetIndex() const = 0;

    virtual IRFPath* GetRFPath() = 0;
    virtual ILPF* GetLPF() = 0;
    virtual INCO* GetNCO() = 0;

    /// @return The number of general purpose FIR filters in this channel.
    virtual uint8_t GetGFIRCount() const = 0;
    virtual IGFIR* GetGFIR(uint8_t index) = 0;

    /// @return The number of gain stages in this channel.
    virtual uint8_t GetGainStageCount() const = 0;
    /// @return The gain stage, in signal path order, or nullptr if the index is out of range.
    virtual IGainStage* GetGainStage(uint8_t index) = 0;

    virtual ILOMixer* GetLOMixer() = 0;
    virtual IDCCorrector* GetDCCorrector() = 0;
    virtual IOversampler* GetOversampler() = 0;
    virtual IQuadratureErrorCorrector* GetQuadratureErrorCorrector() = 0;
    virtual IToneGenerator* GetToneGenerator() = 0;
};

} // namespace lime

#endif
