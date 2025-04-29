#ifndef LIMESUITENG_STREAMMETA_H
#define LIMESUITENG_STREAMMETA_H

#include "limesuiteng/Timespec.h"

namespace lime {

class StreamTxMeta
{
  public:
    enum Flags {
        EndOfBurst = (1 << 0),
    };
    lime::Timespec timestamp;
    bool hasTimestamp;

    uint32_t flags;
};

class StreamRxMeta
{
  public:
    lime::Timespec timestamp;
    bool hasTimestamp;
};

} // namespace lime

#endif