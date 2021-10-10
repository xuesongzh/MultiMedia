#ifndef MESSAGEQUEUE_H
#define MESSAGEQUEUE_H

#include <mutex>
#include <condition_variable>
#include <list>
#include "dlog.h"
extern "C"
{
#include "libavcodec/avcodec.h"
}
#define MSG_FLUSH                   1
#define MSG_RTSP_ERROR              100
#define MSG_RTSP_QUEUE_DURATION     101
typedef struct AVMessage
{
    int what;           // 消息类型
    int arg1;
    int arg2;
    void *obj;          //如果2个参数不够用，则传入结构体
    void (*free_l)(void *obj);
}AVMessage;

static void msg_obj_free_l(void *obj)
{
    av_free(obj);
}
class MessageQueue
{
public:
    MessageQueue() {}
    ~MessageQueue()
    {
        msg_queue_flush();
    }
    inline void msg_init_msg(AVMessage *msg)
    {
        memset(msg, 0, sizeof(AVMessage));
    }

    int msg_queue_put(AVMessage *msg)
    {
        LogInfo("msg_queue_put");
        std::lock_guard<std::mutex> lock(mutex_);
        int ret = msg_queue_put_private(msg);
        if(0 == ret) {
            cond_.notify_one();     // 正常插入队列了才会notify
        }

        return ret;
    }
    // 返回值：-1代表abort; 0 代表没有消息;  1代表读取到了消息
    // timeout: -1代表阻塞等待; 0; 代表非阻塞等待; >0 代表有超时的等待; -2 代表参数异常
    int msg_queue_get(AVMessage *msg, int timeout)
    {
        if(!msg) {
            return -2;
        }
        std::unique_lock<std::mutex> lock(mutex_);
        AVMessage *msg1;
        int ret;
        for(;;) {
            if(abort_request_) {
                ret = -1;
                break;
            }
            if(!queue_.empty()) {
                msg1= queue_.front();
                *msg = *msg1;
                queue_.pop_front();
                av_free(msg1);      // 释放msg1
                ret = 1;
                break;
            } else if(0 == timeout) {
                ret = 0;        // 没有消息
                break;
            } else if(timeout < 0){
                cond_.wait(lock, [this] {
                    return !queue_.empty() | abort_request_;    // 队列不为空或者abort请求才退出wait
                });
            } else if(timeout > 0) {
//                LogInfo("wait_for into");
                cond_.wait_for(lock, std::chrono::milliseconds(timeout), [this] {
//                    LogInfo("wait_for leave");
                    return !queue_.empty() | abort_request_;        // 直接写return true;是错误的
                });
                if(queue_.empty()) {
                    ret = 0;
                    break;
                }
            } else {
                ret = -2;
                break;
            }
        }
        return ret;
    }
    // 把队列里面what类型的消息全部删除
    void msg_queue_remove(int what)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        while(!abort_request_ && !queue_.empty()) {
            std::list<AVMessage *>::iterator it;
            AVMessage *msg = NULL;
            for(it = queue_.begin(); it != queue_.end(); it++) {
                if((*it)->what == what) {
                    msg = *it;
                    break;
                }
            }
            if(msg) {
                if(msg->obj && msg->free_l) {
                    msg->free_l(msg->obj);
                }
                av_free(msg);
                queue_.remove(msg);
            } else {
                break;
            }
        }
    }
    // 只有消息类型
    void notify_msg1(int what)
    {
        AVMessage msg;
        msg_init_msg(&msg);
        msg.what = what;
        msg_queue_put(&msg);
    }

    void notify_msg2(int what, int arg1)
    {
        AVMessage msg;
        msg_init_msg(&msg);
        msg.what = what;
        msg.arg1 = arg1;
        msg_queue_put(&msg);
    }

    void notify_msg3(int what, int arg1, int arg2)
    {
        AVMessage msg;
        msg_init_msg(&msg);
        msg.what = what;
        msg.arg1 = arg1;
        msg.arg2 = arg2;
        msg_queue_put(&msg);
    }
    void notify_msg4(int what, int arg1, int arg2, void *obj, int obj_len)
    {
        AVMessage msg;
        msg_init_msg(&msg);
        msg.what = what;
        msg.arg1 = arg1;
        msg.arg2 = arg2;
        msg.obj = av_malloc(obj_len);
        msg.free_l = msg_obj_free_l;
        memcpy(msg.obj, obj, obj_len);
        msg_queue_put(&msg);
    }

    void msg_queue_abort()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        abort_request_ = 1;
    }

    void msg_queue_flush()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        while (!queue_.empty()) {
            AVMessage *msg = queue_.front();
            if(msg->obj && msg->free_l) {
                msg->free_l(msg->obj);
            }
            queue_.pop_front();
            av_free(msg);
        }
    }

    void msg_queue_destroy(MessageQueue *q)
    {
        msg_queue_flush();
    }
private:

    int msg_queue_put_private(AVMessage *msg)
    {
        if(abort_request_) {
            return -1;
        }
        AVMessage *msg1 = (AVMessage *)av_malloc(sizeof(AVMessage));
        if(!msg1) {
            return -1;
        }
        *msg1 = *msg;
        queue_.push_back(msg1);
        return 0;
    }
    int abort_request_ = 0;
    std::mutex mutex_;
    std::condition_variable cond_;
    std::list<AVMessage *> queue_;
};
#endif // MESSAGEQUEUE_H
