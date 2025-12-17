#include "TransmitControl.h"

#include "chips/LA9310/PHYTimer.h"
#include "chips/LA9310/VSPA_iqplayer.h"
#include "limesuiteng/OpStatus.h"

#include <iostream>

#if 0 // print debug messages
    #define printf_dbg_log(...) \
        do \
        { \
            printf(__VA_ARGS__); \
        } while (0)
#else
    #define printf_dbg_log(format, ...)
#endif

namespace lime {

TransmitControl::TransmitControl(PHYTimer* phytimer, VSPA_iqplayer* vspa)
    : vspa(vspa)
    , phytimer(phytimer)
    , free_running_timer_id(10)
    , tx_timer_id(11)
{
}
TransmitControl::~TransmitControl()
{
}

void TransmitControl::Start(uint32_t counter)
{
    startOffset = counter;
    runtime = Timespec(0, 0, phytimer->GetTickRate());
    phytimerClock = Timespec(0, 0, phytimer->GetTickRate());
    streamClockOffset = Timespec(0, counter, phytimer->GetTickRate());

    lastCounter = 0;
    totalTimerTicks = 0;

    phytimer->GetTimerControl(free_running_timer_id).CaptureCounter();
    eventsPassed.store(0);
    WorkerThread::Start();
}

void TransmitControl::Stop()
{
    WorkerThread::Stop();
}

bool TransmitControl::Work()
{
    PHYTimerControl freerunning = phytimer->GetTimerControl(free_running_timer_id);

    Event* scheduledEvent = nullptr;
    while (doWork.load())
    {
        uint32_t counterNow = freerunning.ReadCounter();
        uint32_t ticksPassed = counterNow - lastCounter;
        totalTimerTicks += ticksPassed;
        lastCounter = counterNow;

        phytimerClock.AddTicks(ticksPassed);

        const Timespec streamTime = phytimerClock - streamClockOffset;
        {
            std::unique_lock lck{ runtime_mutex };
            runtime = streamTime;
        }

        if (scheduledEvent)
        {
            if (streamTime > scheduledEvent->timestamp)
            {
                // std::cerr << "\t\tEvent Done @ phy: " << counterNow << "\n";
                // std::cerr << "\t\tT11 " << phytimer->GetTimerControl(tx_timer_id).ToString() << "\n" << std::endl;

                if (scheduledEvent->action == Action::TxOff)
                {
                    OpStatus status = vspa->StopTx();
                    if (status != OpStatus::Success)
                        printf("FAILED TO STOPTX\n");
                }

                scheduledEvent = nullptr;
                std::unique_lock lck2{ queue_mutex };
                events_fifo.pop();
                eventsPassed.fetch_add(1);
            }
            else
                continue;
        }

        std::unique_lock lck2{ queue_mutex };
        while (!events_fifo.empty())
        {
            Event& evt = events_fifo.front();

            const PHYTimerControl::TriggerLogic trigger =
                evt.action == Action::TxOn ? PHYTimerControl::TriggerLogic::ForceOne : PHYTimerControl::TriggerLogic::ForceZero;

            // events that are already late
            if (streamTime > evt.timestamp)
            {
                printf_dbg_log("Event Late\n");
                phytimer->GetTimerControl(tx_timer_id).TriggerDirectly(trigger);
                if (evt.action == Action::TxOn)
                {
                    OpStatus status = vspa->StartTx();
                    if (status != OpStatus::Success)
                        printf("FAILED TO STARTTX %i\n", status);
                }
                else if (evt.action == Action::TxOff)
                {
                    OpStatus status = vspa->StopTx();
                    if (status != OpStatus::Success)
                        printf("FAILED TO STARTTX\n");
                }
                events_fifo.pop();
                continue;
            }

            if (evt.action == Action::TxOn)
            {
                OpStatus status = vspa->StartTx();
                if (status != OpStatus::Success)
                    printf("FAILED TO STARTTX\n");
            }

            Timespec eventTs = (evt.timestamp + streamClockOffset);
            double seconds = eventTs.GetSeconds() + eventTs.GetFracSeconds();
            int64_t ticks = seconds * phytimer->GetTickRate();
            uint32_t counter = ticks & 0xFFFFFFFF;
            phytimer->GetTimerControl(tx_timer_id).TriggerAtCounter(trigger, counter);
            printf_dbg_log("\tTxControl, event scheduled Tx %s @ t:%.6fs ticks:%li, phy:%08X phynow:%08X\n",
                (evt.action == Action::TxOn ? "ON" : "OFF"),
                evt.timestamp.GetSeconds() + evt.timestamp.GetFracSeconds(),
                evt.timestamp.GetTicks(),
                counter,
                counterNow);

            scheduledEvent = &events_fifo.front();
            break;
        }
    }
    return false;
}

void TransmitControl::ScheduleEvent(const Event& evt)
{
    std::unique_lock lck{ queue_mutex };
    printf_dbg_log("TxControl, submitted action:%s @ t:%.6fs ticks:%li\n",
        (evt.action == Action::TxOn ? "ON" : "OFF"),
        evt.timestamp.GetSeconds() + evt.timestamp.GetFracSeconds(),
        evt.timestamp.GetTicks());
    events_fifo.push(evt);
}

Timespec TransmitControl::now()
{
    std::unique_lock lck{ runtime_mutex };
    return runtime;
}

int TransmitControl::PendingEventCount()
{
    std::unique_lock lck2{ queue_mutex };
    return events_fifo.size();
}

} // namespace lime
