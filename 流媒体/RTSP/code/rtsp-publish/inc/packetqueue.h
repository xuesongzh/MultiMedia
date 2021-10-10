#ifndef PACKETQUEUE_H
#define PACKETQUEUE_H
#include <mutex>
#include <condition_variable>
#include <queue>
#include "mediabase.h"
#include "dlog.h"
extern "C"
{
#include "libavcodec/avcodec.h"
}

typedef struct packet_queue_stats
{
    int audio_nb_packets;   // 音频包数量
    int video_nb_packets;   // 视频包数量
    int audio_size;         // 音频总大小 字节
    int video_size;         // 视频总大小 字节
    int64_t audio_duration; //音频持续时长
    int64_t video_duration; //视频持续时长
}PacketQueueStats;

typedef struct my_avpacket
{
    AVPacket *pkt;
    MediaType media_type;
}MyAVPacket;

class PacketQueue
{
public:
    PacketQueue(double audio_frame_duration, double video_frame_duration)
        :audio_frame_duration_(audio_frame_duration),
        video_frame_duration_(video_frame_duration)
    {
        if(audio_frame_duration_ < 0) {
            audio_frame_duration_ = 0;
        }
        if(video_frame_duration < 0) {
            video_frame_duration = 0;
        }
        memset(&stats_, 0, sizeof(PacketQueueStats));
    }
    ~PacketQueue()
    {

    }
    // 插入packet，需要指明音视频类型
    // 返回0说明正常
    int Push(AVPacket *pkt, MediaType media_type)
    {
        if(!pkt) {
            LogError("pkt is null");
            return -1;
        }
        if(media_type != E_AUDIO_TYPE && media_type != E_VIDEO_TYPE) {
            LogError("media_type:%d is unknown", media_type);
            return -1;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        int ret = pushPrivate(pkt, media_type);
        if(ret < 0) {
            LogError("pushPrivate failed");
            return -1;
        } else {
            cond_.notify_one();
            return 0;
        }
    }

    int pushPrivate(AVPacket *pkt, MediaType media_type)
    {
        if(abort_request_) {
            LogWarn("abort request");
            return -1;
        }
        MyAVPacket *mypkt = (MyAVPacket *)malloc(sizeof(MyAVPacket));
        if(!mypkt) {
            LogError("malloc MyAVPacket failed");
            return -1;
        }
        mypkt->pkt = pkt;
        mypkt->media_type = media_type;
        if(E_AUDIO_TYPE == media_type) {
            stats_.audio_nb_packets++;      // 包数量
            stats_.audio_size += pkt->size;
            // 持续时长怎么统计，不是用pkt->duration
            audio_back_pts_ = pkt->pts;
            if(audio_first_packet) {
                audio_first_packet  = 0;
                audio_front_pts_ = pkt->pts;
            }
        }
        if(E_VIDEO_TYPE == media_type) {
            stats_.video_nb_packets++;      // 包数量
            stats_.video_size += pkt->size;
            // 持续时长怎么统计，不是用pkt->duration
            video_back_pts_ = pkt->pts;
            if(video_first_packet) {
                video_first_packet  = 0;
                video_front_pts_ = pkt->pts;
            }
        }
        queue_.push(mypkt);     // 一定要push
        return 0;
    }

    // 取出packet，并也取出对应的类型
    // 返回值: -1 abort; 1 获取到消息
    int Pop(AVPacket **pkt, MediaType &media_type)
    {
        if(!pkt) {
            LogError("pkt is null");
            return -1;
        }
        std::unique_lock<std::mutex> lock(mutex_);
        if(abort_request_) {
            LogWarn("abort request");
            return -1;
        }
        if(queue_.empty()) {        // 等待唤醒
            // return如果返回false，继续wait, 如果返回true退出wait
            cond_.wait(lock, [this] {
                return !queue_.empty() | abort_request_;
            });
        }
        if(abort_request_) {
            LogWarn("abort request");
            return -1;
        }
        // 真正干活
        MyAVPacket *mypkt = queue_.front(); //读取队列首部元素，这里还没有真正出队列
        *pkt        = mypkt->pkt;
        media_type  = mypkt->media_type;

        if(E_AUDIO_TYPE == media_type) {
            stats_.audio_nb_packets--;      // 包数量
            stats_.audio_size -= mypkt->pkt->size;
            // 持续时长怎么统计，不是用pkt->duration
            audio_front_pts_ = mypkt->pkt->pts;
        }
        if(E_VIDEO_TYPE == media_type) {
            stats_.video_nb_packets--;      // 包数量
            stats_.video_size -= mypkt->pkt->size;
            // 持续时长怎么统计，不是用pkt->duration
            video_front_pts_ = mypkt->pkt->pts;
        }

        queue_.pop();
        free(mypkt);

        return 1;
    }
    // 带超时时间
    // 返回值: -1 abort;  0  没有消息； 1有消息
    int PopWithTimeout(AVPacket **pkt, MediaType &media_type, int timeout)
    {
        if(timeout < 0) {
            return Pop(pkt, media_type);
        }

        std::unique_lock<std::mutex> lock(mutex_);
        if(abort_request_) {
            LogWarn("abort request");
            return -1;
        }
        if(queue_.empty()) {        // 等待唤醒
            // return如果返回false，继续wait, 如果返回true退出wait
            cond_.wait_for(lock, std::chrono::milliseconds(timeout), [this] {
                return !queue_.empty() | abort_request_;
            });
        }
        if(abort_request_) {
            LogWarn("abort request");
            return -1;
        }
        if(queue_.empty()) {
            return 0;
        }
        // 真正干活
        MyAVPacket *mypkt = queue_.front(); //读取队列首部元素，这里还没有真正出队列
        *pkt        = mypkt->pkt;
        media_type  = mypkt->media_type;

        if(E_AUDIO_TYPE == media_type) {
            stats_.audio_nb_packets--;      // 包数量
            stats_.audio_size -= mypkt->pkt->size;
            // 持续时长怎么统计，不是用pkt->duration
            audio_front_pts_ = mypkt->pkt->pts;
        }
        if(E_VIDEO_TYPE == media_type) {
            stats_.video_nb_packets--;      // 包数量
            stats_.video_size -= mypkt->pkt->size;
            // 持续时长怎么统计，不是用pkt->duration
            video_front_pts_ = mypkt->pkt->pts;
        }

        queue_.pop();
        free(mypkt);
        return 1;
    }
    bool Empty()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }
    // 唤醒在等待的线程
    void Abort()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        abort_request_ = true;
        cond_.notify_all();
    }
    // all为true:清空队列;
    // all为false: drop数据，直到遇到I帧, 最大保留remain_max_duration时长;
    int Drop(bool all, int64_t remain_max_duration)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        while (!queue_.empty()) {
            MyAVPacket *mypkt = queue_.front();
            if(!all && mypkt->media_type == E_VIDEO_TYPE && (mypkt->pkt->flags &AV_PKT_FLAG_KEY)) {
                int64_t duration = video_back_pts_ - video_front_pts_;  //以pts为准
                // 也参考帧（包）持续 *帧(包)数
                if(duration < 0     // pts回绕
                        || duration > video_frame_duration_ * stats_.video_nb_packets * 2) {
                    duration =  video_frame_duration_ * stats_.video_nb_packets;
                }
                LogInfo("video duration:%lld", duration);
                if(duration <= remain_max_duration)
                    break;          // 说明可以break 退出while
            }
            if(E_AUDIO_TYPE == mypkt->media_type) {
                stats_.audio_nb_packets--;      // 包数量
                stats_.audio_size -= mypkt->pkt->size;
                // 持续时长怎么统计，不是用pkt->duration
                audio_front_pts_ = mypkt->pkt->pts;
            }
            if(E_VIDEO_TYPE == mypkt->media_type) {
                stats_.video_nb_packets--;      // 包数量
                stats_.video_size -= mypkt->pkt->size;
                // 持续时长怎么统计，不是用pkt->duration
                video_front_pts_ = mypkt->pkt->pts;
            }
            av_packet_free(&mypkt->pkt);        // 先释放AVPacket
            queue_.pop();
            free(mypkt);                        // 再释放MyAVPacket
        }

        return 0;
    }
    // 获取音频持续时间
    int64_t GetAudioDuration()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        int64_t duration = audio_back_pts_ - audio_front_pts_;  //以pts为准
        // 也参考帧（包）持续 *帧(包)数
        if(duration < 0     // pts回绕
                || duration > audio_frame_duration_ * stats_.audio_nb_packets * 2) {
            duration =  audio_frame_duration_ * stats_.audio_nb_packets;
        } else {
            duration += audio_frame_duration_;
        }
        return duration;
    }
    // 获取视频持续时间
    int64_t GetVideoDuration()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        int64_t duration = video_back_pts_ - video_front_pts_;  //以pts为准
        // 也参考帧（包）持续 *帧(包)数
        if(duration < 0     // pts回绕
                || duration > video_frame_duration_ * stats_.video_nb_packets * 2) {
            duration =  video_frame_duration_ * stats_.video_nb_packets;
        }else {
            duration += video_frame_duration_;
        }
        return duration;
    }
    // 获取音频包数量
    int GetAudioPackets()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return stats_.audio_nb_packets;
    }
    // 获取视频包数量
    int GetVideoPackets()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return stats_.video_nb_packets;
    }

    void GetStats(PacketQueueStats *stats)
    {
        if(!stats) {
            LogError("stats is null");
            return;
        }
        std::lock_guard<std::mutex> lock(mutex_);

        int64_t audio_duration = audio_back_pts_ - audio_front_pts_;  //以pts为准
        // 也参考帧（包）持续 *帧(包)数
        if(audio_duration < 0     // pts回绕
                || audio_duration > audio_frame_duration_ * stats_.audio_nb_packets * 2) {
            audio_duration =  audio_frame_duration_ * stats_.audio_nb_packets;
        }else {
            audio_duration += audio_frame_duration_;
        }
        int64_t video_duration = video_back_pts_ - video_front_pts_;  //以pts为准
        // 也参考帧（包）持续 *帧(包)数
        if(video_duration < 0     // pts回绕
                || video_duration > video_frame_duration_ * stats_.video_nb_packets * 2) {
            video_duration =  video_frame_duration_ * stats_.video_nb_packets;
        }else {
            video_duration += video_frame_duration_;
        }
        stats->audio_duration = audio_duration;
        stats->video_duration = video_duration;
        stats->audio_nb_packets = stats_.audio_nb_packets;
        stats->video_nb_packets = stats_.audio_nb_packets;
        stats->audio_size = stats_.audio_size;
        stats->video_size = stats_.video_size;
    }
private:
    std::mutex mutex_;
    std::condition_variable cond_;
    std::queue<MyAVPacket *> queue_;


    bool abort_request_ = false;

    // 统计相关
    PacketQueueStats stats_;
    double audio_frame_duration_ = 23.21995649; // 默认23.2ms 44.1khz  1024*1000ms/44100=23.21995649ms
    double video_frame_duration_ = 40;  // 40ms 视频帧率为25的  ， 1000ms/25=40ms
    // pts记录
    int64_t audio_front_pts_ = 0;
    int64_t audio_back_pts_ = 0;
    int     audio_first_packet = 1;
    int64_t video_front_pts_ = 0;
    int64_t video_back_pts_ = 0;
    int     video_first_packet = 1;

};
#endif // PACKETQUEUE_H
