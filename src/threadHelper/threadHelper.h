#ifndef LIMESUITE_THREAD_H
#define LIMESUITE_THREAD_H

#include <thread>

namespace lime {

enum class ThreadPriority : uint8_t { LOWEST, LOW, BELOW_NORMAL, NORMAL, ABOVE_NORMAL, HIGH, HIGHEST };

enum class ThreadPolicy : uint8_t {
    DEFAULT,
    REALTIME,
    PREEMPTIVE, // FIFO
};
/**
 * Set priority of current or specified thread
 *
 * @param priority  Thread priority
 * @param policy    Thread scheduling policy, not used on Windows
 * @param thread    Thread to which set the priority to
 *
 * @return          0 on success, (-1) on failure
 */
int SetOSThreadPriority(ThreadPriority priority, ThreadPolicy policy, std::thread* thread);

/**
 * Set priority of current or specified thread
 * @note On Windows systems, policy will be used to set either IDLE or TIME_CRITICAL thread priority
 *
 * @param priority  Thread priority
 * @param policy    Thread scheduling policy, not used on Windows
 *
 * @return          0 on success, (-1) on failure
 */
int SetOSCurrentThreadPriority(ThreadPriority priority, ThreadPolicy policy);
} // namespace lime

#endif
