#ifndef RTSPPUSHER_H
#define RTSPPUSHER_H

#include "mediabase.h"
#include "commonlooper.h"
#include "packetqueue.h"
#include "messagequeue.h"
extern "C" {
#include "libavformat/avformat.h"
#include "libavformat/avio.h"
#include "libavcodec/avcodec.h"
#include "libavutil/opt.h"
}

class RtspPusher: public CommonLooper
{
public:
    RtspPusher( MessageQueue *msg_queue);
    virtual ~RtspPusher();
    RET_CODE Init(const Properties& properties);
    void DeInit();
    RET_CODE Push(AVPacket *pkt, MediaType media_type);
    // 连接服务器，如果连接成功则启动线程
    RET_CODE Connect();

    // 如果有视频成分
    RET_CODE ConfigVideoStream(const AVCodecContext *ctx);
    // 如果有音频成分
    RET_CODE ConfigAudioStream(const AVCodecContext *ctx);
    virtual void Loop();
    bool IsTimeout();
    void RestTiemout();
    int GetTimeout();
    int64_t GetBlockTime();
private:
    int64_t pre_debug_time_ = 0;
    int64_t debug_interval_ = 2000;
    void debugQueue(int64_t interval);  // 按时间间隔打印packetqueue的状况
    // 监测队列的缓存情况
    void checkPacketQueueDuration();
    int sendPacket(AVPacket *pkt, MediaType media_type);
    // 整个输出流的上下文
    AVFormatContext *fmt_ctx_ = NULL;
    // 视频编码器上下文
    AVCodecContext *video_ctx_ = NULL;
    // 音频频编码器上下文
    AVCodecContext *audio_ctx_ = NULL;

    // 流成分
    AVStream *video_stream_ = NULL;
    int video_index_ = -1;
    AVStream *audio_stream_ = NULL;
    int audio_index_ = -1;

    std::string url_ = "";
    std::string rtsp_transport_ = "";

    double audio_frame_duration_ = 23.21995649; // 默认23.2ms 44.1khz  1024*1000ms/44100=23.21995649ms
    double video_frame_duration_ = 40;  // 40ms 视频帧率为25的  ， 1000ms/25=40ms
    PacketQueue *queue_ = NULL;

    // 队列最大限制时长
    int max_queue_duration_ = 500;  // 默认100ms

    // 处理超时
    int timeout_;
    int64_t pre_time_ = 0;      // 记录调用ffmpeg api之前的时间
    MessageQueue *msg_queue_ = NULL;
};

#endif // RTSPPUSHER_H
