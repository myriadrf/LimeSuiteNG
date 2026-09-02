#include "WorkerThread.h"

#include <functional>

namespace lime {

WorkerThread::WorkerThread()
    : doWork(false)
    , terminateWorker(false)
    , workInProgress(false)
{
    mThread = std::thread(std::bind(&WorkerThread::WorkLoop, this));
}

WorkerThread::~WorkerThread()
{
    terminateWorker.store(true);
    Stop();
    if (mThread.joinable())
        mThread.join();
}

void WorkerThread::Start()
{
    doWork.store(true);
    work_cv.notify_all();
}

void WorkerThread::Stop()
{
    doWork.store(false);
}

void WorkerThread::Wait()
{
    std::unique_lock lk{ threadControlsMutex };
    while (workInProgress.load() || doWork.load(std::memory_order_relaxed))
        work_cv.wait_for(lk, std::chrono::milliseconds(100));
}

void WorkerThread::WorkLoop()
{
    while (!terminateWorker.load(std::memory_order_relaxed))
    {
        {
            std::unique_lock lk{ threadControlsMutex };
            while (!doWork.load(std::memory_order_relaxed) && !terminateWorker.load(std::memory_order_relaxed))
                work_cv.wait_for(lk, std::chrono::milliseconds(100));
        }

        while (doWork.load())
        {
            workInProgress.store(true);
            bool continueWork = Work();
            workInProgress.store(false);
            if (!continueWork)
            {
                doWork.store(false);
                break;
            }
        }
        work_cv.notify_all();
    }
}

} // namespace lime
