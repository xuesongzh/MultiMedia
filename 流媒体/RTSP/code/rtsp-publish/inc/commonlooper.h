#ifndef COMMONLOOPER_H
#define COMMONLOOPER_H

#include <thread>

#include "mediabase.h"

class CommonLooper {
 public:
    CommonLooper();
    virtual ~CommonLooper();
    virtual RET_CODE Start();  // 开启线程
    virtual void Stop();       // 停止线程
    virtual bool Running();
    virtual void SetRunning(bool running);
    virtual void Loop() = 0;

 private:
    static void *trampoline(void *p);

 protected:
    std::thread *worker_ = NULL;  // 线程
    bool request_abort_ = false;  // 请求退出线程的标志
    bool running_ = true;         // 线程是否在运行
};

#endif  // COMMONLOOPER_H
