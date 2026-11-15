#include <gtest/gtest.h>

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <fcntl.h>
#include <glob.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <unistd.h>

#include "drivers/linux/la9310_limesdr/la9310_ioctl.h"

namespace {

enum class Pool { Cached, Uncached };

std::string PoolName(const ::testing::TestParamInfo<Pool>& info)
{
    return info.param == Pool::Cached ? "Cached" : "Uncached";
}

// Exercises the la9310_limesdr DMA pools directly through their ioctls. No firmware is needed, but no other application
// can be using the device's pools, as these tests expect to have all of a pool's memory.
class LA9310_DMAPool : public ::testing::TestWithParam<Pool>
{
  protected:
    void SetUp() override
    {
        glob_t matches{};
        if (glob("/dev/limesdr_micro*/control0", 0, nullptr, &matches) != 0)
            GTEST_SKIP() << "No LimeSDR-Micro found";
        path = matches.gl_pathv[0];
        globfree(&matches);

        fd = Open();
        ASSERT_GE(fd, 0) << strerror(errno);

        LA9310_IOCTL_memory_layout layout{};
        ASSERT_EQ(ioctl(fd, LA9310_IOCTL_GET_MEMORY_LAYOUT, &layout), 0) << strerror(errno);
        poolSize = layout.window_size[GetParam() == Pool::Cached ? LA9310_WINDOW_CACHED_POOL : LA9310_WINDOW_UNCACHED_POOL];
        otherPoolSize = layout.window_size[GetParam() == Pool::Cached ? LA9310_WINDOW_UNCACHED_POOL : LA9310_WINDOW_CACHED_POOL];
        ASSERT_GT(poolSize, 0u);
        ASSERT_GT(otherPoolSize, 0u);
        pageSize = sysconf(_SC_PAGESIZE);
    }

    void TearDown() override
    {
        if (fd >= 0)
            close(fd);
    }

    int Open() const { return open(path.c_str(), O_RDWR); }

    static size_t PoolFlags(Pool pool) { return pool == Pool::Uncached ? LA9310_DMA_FLAG_UNCACHED : 0; }

    static int Alloc(int fd, size_t size, la9310_atu& mem, size_t flags)
    {
        mem = la9310_atu{};
        mem.size = size;
        mem.flags = flags;
        return ioctl(fd, LA9310_IOCTL_DMA_ALLOC, &mem);
    }

    // from the pool under test
    static int Alloc(int fd, size_t size, la9310_atu& mem) { return Alloc(fd, size, mem, PoolFlags(GetParam())); }

    static int Free(int fd, const la9310_atu& mem) { return ioctl(fd, LA9310_IOCTL_DMA_FREE, &mem); }

    static void* Map(int fd, const la9310_atu& mem, int flags = MAP_SHARED)
    {
        return mmap(nullptr, mem.size, PROT_READ | PROT_WRITE, flags, fd, mem.mmap_offset);
    }

    std::string path;
    int fd{ -1 };
    size_t poolSize{ 0 };
    size_t otherPoolSize{ 0 };
    size_t pageSize{ 0 };
};

TEST_P(LA9310_DMAPool, AllocationIsMappableAndRoundedToPages)
{
    la9310_atu mem;
    ASSERT_EQ(Alloc(fd, 1, mem), 0) << strerror(errno);
    EXPECT_EQ(mem.size, pageSize);
    EXPECT_EQ(mem.host_bus % pageSize, 0u);
    EXPECT_EQ(mem.ep_pa % pageSize, 0u);

    auto* va = static_cast<volatile uint32_t*>(Map(fd, mem));
    ASSERT_NE(va, MAP_FAILED) << strerror(errno);
    va[0] = 0xA5A5A5A5;
    EXPECT_EQ(va[0], 0xA5A5A5A5u);

    munmap(const_cast<uint32_t*>(va), mem.size);
    EXPECT_EQ(Free(fd, mem), 0) << strerror(errno);
}

TEST_P(LA9310_DMAPool, ReportsPoolAndCoherency)
{
    la9310_atu mem;
    ASSERT_EQ(Alloc(fd, pageSize, mem), 0) << strerror(errno);
    if (GetParam() == Pool::Uncached)
    {
        EXPECT_TRUE(mem.flags & LA9310_DMA_FLAG_UNCACHED);
        EXPECT_TRUE(mem.flags & LA9310_DMA_FLAG_COHERENT);
    }
    else
        EXPECT_FALSE(mem.flags & LA9310_DMA_FLAG_UNCACHED);
    // still accepted on coherent buffers, as a no-op
    EXPECT_EQ(ioctl(fd, LA9310_IOCTL_CACHE_SYNC_FOR_CPU, &mem), 0) << strerror(errno);
    EXPECT_EQ(ioctl(fd, LA9310_IOCTL_CACHE_SYNC_FOR_DEVICE, &mem), 0) << strerror(errno);
}

TEST_P(LA9310_DMAPool, InvalidRequestsAreRejected)
{
    la9310_atu mem;
    EXPECT_EQ(Alloc(fd, 0, mem), -1);
    EXPECT_EQ(errno, EINVAL);
    EXPECT_EQ(Alloc(fd, poolSize + 1, mem), -1);
    EXPECT_EQ(errno, EINVAL);
    EXPECT_EQ(Alloc(fd, SIZE_MAX, mem), -1);
    EXPECT_EQ(errno, EINVAL);
    EXPECT_EQ(Alloc(fd, pageSize, mem, PoolFlags(GetParam()) | (1 << 8)), -1);
    EXPECT_EQ(errno, EINVAL);
}

TEST_P(LA9310_DMAPool, AllocationsDoNotOverlap)
{
    la9310_atu a, b;
    ASSERT_EQ(Alloc(fd, 2 * pageSize, a), 0);
    ASSERT_EQ(Alloc(fd, 2 * pageSize, b), 0);
    EXPECT_TRUE(a.host_bus + a.size <= b.host_bus || b.host_bus + b.size <= a.host_bus);
    EXPECT_TRUE(a.ep_pa + a.size <= b.ep_pa || b.ep_pa + b.size <= a.ep_pa);
    EXPECT_EQ(b.host_bus - a.host_bus, b.ep_pa - a.ep_pa);
    EXPECT_NE(a.mmap_offset, b.mmap_offset);
}

TEST_P(LA9310_DMAPool, PoolsAreSeparate)
{
    const Pool other = GetParam() == Pool::Cached ? Pool::Uncached : Pool::Cached;
    la9310_atu all, fromOther;
    ASSERT_EQ(Alloc(fd, poolSize, all), 0) << strerror(errno);
    ASSERT_EQ(Alloc(fd, otherPoolSize, fromOther, PoolFlags(other)), 0) << strerror(errno);

    EXPECT_TRUE(all.host_bus + all.size <= fromOther.host_bus || fromOther.host_bus + fromOther.size <= all.host_bus);
    EXPECT_TRUE(all.ep_pa + all.size <= fromOther.ep_pa || fromOther.ep_pa + fromOther.size <= all.ep_pa);
    EXPECT_NE(all.mmap_offset, fromOther.mmap_offset);

    void* a = Map(fd, all);
    void* b = Map(fd, fromOther);
    ASSERT_NE(a, MAP_FAILED) << strerror(errno);
    ASSERT_NE(b, MAP_FAILED) << strerror(errno);
    memset(a, 0x11, all.size);
    memset(b, 0x22, fromOther.size);
    EXPECT_EQ(static_cast<uint8_t*>(a)[all.size - 1], 0x11);
    EXPECT_EQ(static_cast<uint8_t*>(b)[0], 0x22);
    munmap(a, all.size);
    munmap(b, fromOther.size);
}

TEST_P(LA9310_DMAPool, PoolsCanNotBeMappedWhole)
{
    for (int window : { LA9310_WINDOW_CACHED_POOL, LA9310_WINDOW_UNCACHED_POOL })
    {
        void* va = mmap(nullptr, pageSize, PROT_READ | PROT_WRITE, MAP_SHARED, fd, window * pageSize);
        EXPECT_EQ(va, MAP_FAILED);
        if (va != MAP_FAILED)
            munmap(va, pageSize);
    }
}

TEST_P(LA9310_DMAPool, ExhaustionFailsAndFreeMakesMemoryAvailable)
{
    la9310_atu all, other;
    ASSERT_EQ(Alloc(fd, poolSize, all), 0) << strerror(errno);
    EXPECT_EQ(Alloc(fd, pageSize, other), -1);
    EXPECT_EQ(errno, ENOMEM);

    ASSERT_EQ(Free(fd, all), 0);
    EXPECT_EQ(Alloc(fd, pageSize, other), 0) << strerror(errno);
}

TEST_P(LA9310_DMAPool, DoubleFreeIsRejected)
{
    la9310_atu mem;
    ASSERT_EQ(Alloc(fd, pageSize, mem), 0);
    EXPECT_EQ(Free(fd, mem), 0);
    EXPECT_EQ(Free(fd, mem), -1);
    EXPECT_EQ(errno, EINVAL);
}

TEST_P(LA9310_DMAPool, MemoryIsZeroedOnReuse)
{
    la9310_atu mem;
    ASSERT_EQ(Alloc(fd, poolSize, mem), 0);
    void* va = Map(fd, mem);
    ASSERT_NE(va, MAP_FAILED);
    memset(va, 0x55, mem.size);
    munmap(va, mem.size);
    ASSERT_EQ(Free(fd, mem), 0);

    ASSERT_EQ(Alloc(fd, poolSize, mem), 0);
    va = Map(fd, mem);
    ASSERT_NE(va, MAP_FAILED);
    const std::vector<uint8_t> zeroes(mem.size, 0);
    EXPECT_EQ(memcmp(va, zeroes.data(), mem.size), 0);
    munmap(va, mem.size);
}

TEST_P(LA9310_DMAPool, MappingLimits)
{
    la9310_atu mem;
    ASSERT_EQ(Alloc(fd, pageSize, mem), 0);

    EXPECT_EQ(Map(fd, mem, MAP_PRIVATE), MAP_FAILED);
    EXPECT_EQ(mmap(nullptr, 2 * mem.size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, mem.mmap_offset), MAP_FAILED);
}

TEST_P(LA9310_DMAPool, BuffersBelongToTheFileThatAllocatedThem)
{
    la9310_atu mem;
    ASSERT_EQ(Alloc(fd, pageSize, mem), 0);

    const int other = Open();
    ASSERT_GE(other, 0);
    EXPECT_EQ(Map(other, mem), MAP_FAILED);
    EXPECT_EQ(ioctl(other, LA9310_IOCTL_CACHE_SYNC_FOR_CPU, &mem), -1);
    EXPECT_EQ(ioctl(other, LA9310_IOCTL_CACHE_SYNC_FOR_DEVICE, &mem), -1);
    EXPECT_EQ(Free(other, mem), -1);
    close(other);

    // still valid for the owner
    EXPECT_EQ(ioctl(fd, LA9310_IOCTL_CACHE_SYNC_FOR_CPU, &mem), 0) << strerror(errno);
    EXPECT_EQ(ioctl(fd, LA9310_IOCTL_CACHE_SYNC_FOR_DEVICE, &mem), 0) << strerror(errno);
}

TEST_P(LA9310_DMAPool, SyncOutsideOfBufferIsRejected)
{
    la9310_atu mem;
    ASSERT_EQ(Alloc(fd, pageSize, mem), 0);

    la9310_atu range = mem;
    range.size = mem.size + 1;
    EXPECT_EQ(ioctl(fd, LA9310_IOCTL_CACHE_SYNC_FOR_CPU, &range), -1);
    range.size = SIZE_MAX;
    EXPECT_EQ(ioctl(fd, LA9310_IOCTL_CACHE_SYNC_FOR_CPU, &range), -1);
}

TEST_P(LA9310_DMAPool, ClosingFileReleasesItsBuffers)
{
    la9310_atu mem, other;
    const int owner = Open();
    ASSERT_GE(owner, 0);
    ASSERT_EQ(Alloc(owner, poolSize, mem), 0);
    EXPECT_EQ(Alloc(fd, pageSize, other), -1);

    close(owner);
    EXPECT_EQ(Alloc(fd, poolSize, other), 0) << strerror(errno);
}

TEST_P(LA9310_DMAPool, FreedBufferStaysReservedUntilUnmapped)
{
    la9310_atu mem, other;
    ASSERT_EQ(Alloc(fd, poolSize, mem), 0);
    auto* va = static_cast<volatile uint32_t*>(Map(fd, mem));
    ASSERT_NE(va, MAP_FAILED);

    ASSERT_EQ(Free(fd, mem), 0);
    EXPECT_EQ(Alloc(fd, pageSize, other), -1);
    va[0] = 1; // mapping must still be usable

    munmap(const_cast<uint32_t*>(va), mem.size);
    EXPECT_EQ(Alloc(fd, poolSize, other), 0) << strerror(errno);
}

TEST_P(LA9310_DMAPool, ClosedFileBufferStaysReservedUntilUnmapped)
{
    la9310_atu mem, other;
    const int owner = Open();
    ASSERT_GE(owner, 0);
    ASSERT_EQ(Alloc(owner, poolSize, mem), 0);
    auto* va = static_cast<volatile uint32_t*>(Map(owner, mem));
    ASSERT_NE(va, MAP_FAILED);

    close(owner);
    EXPECT_EQ(Alloc(fd, pageSize, other), -1);
    va[0] = 1;

    munmap(const_cast<uint32_t*>(va), mem.size);
    EXPECT_EQ(Alloc(fd, poolSize, other), 0) << strerror(errno);
}

TEST_P(LA9310_DMAPool, PartialUnmapKeepsBufferUntilAllPagesAreUnmapped)
{
    la9310_atu mem, other;
    ASSERT_EQ(Alloc(fd, 3 * pageSize, mem), 0);
    auto* va = static_cast<uint8_t*>(Map(fd, mem));
    ASSERT_NE(va, MAP_FAILED);
    ASSERT_EQ(Free(fd, mem), 0);

    // splits the mapping in two
    ASSERT_EQ(munmap(va + pageSize, pageSize), 0);
    ASSERT_EQ(Alloc(fd, poolSize, other), -1);
    ASSERT_EQ(munmap(va, pageSize), 0);
    ASSERT_EQ(Alloc(fd, poolSize, other), -1);
    ASSERT_EQ(munmap(va + 2 * pageSize, pageSize), 0);
    EXPECT_EQ(Alloc(fd, poolSize, other), 0) << strerror(errno);
}

TEST_P(LA9310_DMAPool, KilledProcessBuffersAreReleased)
{
    int pipefd[2];
    ASSERT_EQ(pipe(pipefd), 0);

    const size_t size = poolSize;
    const pid_t child = fork();
    ASSERT_GE(child, 0);
    if (child == 0)
    {
        close(fd);
        char ok = 0;
        const int childFd = Open();
        la9310_atu mem;
        if (childFd >= 0 && Alloc(childFd, size, mem) == 0 && Map(childFd, mem) != MAP_FAILED)
            ok = 1;
        if (write(pipefd[1], &ok, 1) != 1)
            _exit(1);
        pause(); // until killed, without any clean up
        _exit(0);
    }

    char ok = 0;
    ASSERT_EQ(read(pipefd[0], &ok, 1), 1);
    close(pipefd[0]);
    close(pipefd[1]);
    EXPECT_EQ(ok, 1);

    la9310_atu mem;
    EXPECT_EQ(Alloc(fd, pageSize, mem), -1);
    EXPECT_EQ(errno, ENOMEM);

    kill(child, SIGKILL);
    waitpid(child, nullptr, 0);

    EXPECT_EQ(Alloc(fd, poolSize, mem), 0) << strerror(errno);
}

TEST_P(LA9310_DMAPool, ForkedChildKeepsBufferUntilItExits)
{
    la9310_atu mem, other;
    const int owner = Open();
    ASSERT_GE(owner, 0);
    ASSERT_EQ(Alloc(owner, poolSize, mem), 0);
    void* va = Map(owner, mem);
    ASSERT_NE(va, MAP_FAILED);

    int pipefd[2];
    ASSERT_EQ(pipe(pipefd), 0);
    const pid_t child = fork();
    ASSERT_GE(child, 0);
    if (child == 0)
    {
        close(pipefd[1]);
        char byte;
        const ssize_t ignored = read(pipefd[0], &byte, 1); // until parent is done
        (void)ignored;
        _exit(0);
    }
    close(pipefd[0]);

    munmap(va, mem.size);
    close(owner);
    EXPECT_EQ(Alloc(fd, pageSize, other), -1);

    close(pipefd[1]);
    waitpid(child, nullptr, 0);
    EXPECT_EQ(Alloc(fd, poolSize, other), 0) << strerror(errno);
}

INSTANTIATE_TEST_SUITE_P(Pools, LA9310_DMAPool, ::testing::Values(Pool::Cached, Pool::Uncached), PoolName);

// The persistent region is the driver's own, but it stays mappable for inspection
TEST(LA9310_DMAPersistent, IsMappable)
{
    glob_t matches{};
    if (glob("/dev/limesdr_micro*/control0", 0, nullptr, &matches) != 0)
        GTEST_SKIP() << "No LimeSDR-Micro found";
    const int fd = open(matches.gl_pathv[0], O_RDWR);
    globfree(&matches);
    ASSERT_GE(fd, 0) << strerror(errno);

    LA9310_IOCTL_memory_layout layout{};
    ASSERT_EQ(ioctl(fd, LA9310_IOCTL_GET_MEMORY_LAYOUT, &layout), 0) << strerror(errno);
    const size_t size = layout.window_size[LA9310_WINDOW_PERSISTENT];
    ASSERT_GT(size, 0u);
    const size_t pageSize = sysconf(_SC_PAGESIZE);
    void* va = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, LA9310_WINDOW_PERSISTENT * pageSize);
    EXPECT_NE(va, MAP_FAILED) << strerror(errno);
    if (va != MAP_FAILED)
        munmap(va, size);
    close(fd);
}

} // namespace
