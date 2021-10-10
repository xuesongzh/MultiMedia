#include "commonlooper.h"

#include "dlog.h"

void *CommonLooper::trampoline(void *p) {
    LogInfo("into");
    ((CommonLooper *)p)->SetRunning(true);
    ((CommonLooper *)p)->Loop();
    ((CommonLooper *)p)->SetRunning(false);
    LogInfo("leave");
    return NULL;
}

CommonLooper::CommonLooper() : request_abort_(false), running_(false) {}

CommonLooper::~CommonLooper() { Stop(); }

RET_CODE CommonLooper::Start() {
    LogInfo("into");
    worker_ = new std::thread(trampoline, this);
    if (!worker_) {
        LogError("new std::this_thread failed");
        return RET_FAIL;
    }
    return RET_OK;
}

void CommonLooper::Stop() {
    request_abort_ = true;
    if (worker_) {
        worker_->join();
        delete worker_;
        worker_ = NULL;
    }
}

bool CommonLooper::Running() { return running_; }

void CommonLooper::SetRunning(bool running) { running_ = running; }
