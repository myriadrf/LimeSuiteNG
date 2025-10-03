#pragma once

#include <mutex>
#include <atomic>
#include <thread>
#include <condition_variable>

#include "limesuiteng/config.h"

namespace lime {

class LIME_API WorkerThread
{
  public:
    WorkerThread();
    virtual ~WorkerThread();

    void Start();
    void Stop();
    void Wait();

    virtual bool Work() = 0;

  protected:
    void WorkLoop();
    std::thread mThread;
    std::condition_variable work_cv;
    std::atomic<bool> doWork;
    std::atomic<bool> terminateWorker;
    std::mutex threadControlsMutex;
};

} // namespace lime
