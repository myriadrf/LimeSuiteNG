#include "LA9310.h"

#include "limesuiteng/Logger.h"

#ifdef __unix__
    #include <unistd.h>
    #include <fcntl.h>
    #include <poll.h>
    #include <sys/mman.h>
    #include <sys/ioctl.h>
#endif

#include <errno.h>
#include <cstring>

namespace lime {

LA9310::LA9310(const std::string& device_path)
    : mDevice_path(device_path)
{
    // use O_RDWR for now, because MMAP PROT_WRITE implies PROT_READ and will fail if file is opened write only
    uint32_t flags = O_RDWR;
    mFileDescriptor = open(mDevice_path.c_str(), flags);
    if (mFileDescriptor < 0)
    {
        lime::error("LimePCIe: Failed to open (%s), errno(%i) %s", mDevice_path.c_str(), errno, strerror(errno));
    }
}

LA9310::~LA9310()
{
    if (mFileDescriptor)
        close(mFileDescriptor);
}

} // namespace lime