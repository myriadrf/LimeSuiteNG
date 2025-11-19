#pragma once

#include <memory>
#include <mutex>
#include <queue>
#include <atomic>

#include "utilities/WorkerThread.h"

#include "limesuiteng/Timespec.h"

namespace lime {

class PHYTimer;
class VSPA_iqplayer;

class TransmitControl : public WorkerThread
{
  public:
    enum class Action { TxOff, TxOn };

    struct Event {
        Timespec timestamp;
        Action action;
    };

    TransmitControl(PHYTimer* phytimer, VSPA_iqplayer* vspa);
    virtual ~TransmitControl();

    void Start(uint32_t counter);
    void Stop();

    bool Work() override;

    void ScheduleEvent(const Event& evt);

    Timespec now();

    int PendingEventCount();

  private:
    VSPA_iqplayer* vspa;
    uint32_t startOffset;
    PHYTimer* phytimer;
    std::queue<Event> events_fifo;

    const uint8_t free_running_timer_id;
    const uint8_t tx_timer_id;
    Timespec runtime;
    Timespec phytimerClock;
    Timespec streamClockOffset;
    uint32_t lastCounter;
    Timespec overflow_period;
    std::mutex runtime_mutex;
    std::mutex queue_mutex;
    uint64_t totalTimerTicks;

    std::atomic<int> eventsPassed;
};

} // namespace lime
