#ifndef LIMESUITENG_STREAMMETA_H
#define LIMESUITENG_STREAMMETA_H

#include "limesuiteng/Timestamp.h"

namespace lime {

class StreamTxMeta
{
  public:
    enum Flags {
        EndOfBurst = (1 << 0),
    };
    lime::Timestamp timestamp;
    bool hasTimestamp;

    uint32_t flags;
};

class StreamRxMeta
{
  public:
    lime::Timestamp timestamp;
    bool hasTimestamp;
};

} // namespace lime

#endif