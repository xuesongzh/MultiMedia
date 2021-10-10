#ifndef PUSHWORK_H
#define PUSHWORK_H

#include <string>

#include "aacencoder.h"
#include "audiocapturer.h"
#include "h264encoder.h"
#include "messagequeue.h"
#include "rtsppusher.h"
#include "videocapturer.h"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/audio_fifo.h>
#include <libavutil/opt.h>
#include <libswresample/swresample.h>
}

class PushWork {
 public:
    PushWork(MessageQueue *msg_queue);
    ~PushWork();
    RET_CODE Init(const Properties &properties);
    RET_CODE DeInit();

 private:
    void PcmCallback(uint8_t *pcm, int32_t size);
    void YuvCallback(uint8_t *yuv, int32_t size);

 private:
    AudioCapturer *audio_capturer_ = NULL;
    // 音频test模式
    int audio_test_ = 0;
    std::string input_pcm_name_;
    uint8_t *fltp_buf_ = NULL;
    int fltp_buf_size_ = 0;
    // 麦克风采样属性
    int mic_sample_rate_ = 48000;
    int mic_sample_fmt_ = AV_SAMPLE_FMT_S16;
    int mic_channels_ = 2;

    AACEncoder *audio_encoder_;
    // 音频编码参数
    int audio_sample_rate_ = AV_SAMPLE_FMT_S16;
    int audio_bitrate_ = 128 * 1024;
    int audio_channels_ = 2;
    int audio_sample_fmt_;  // 具体由编码器决定，从编码器读取相应的信息
    int audio_ch_layout_;   // 由audio_channels_决定

    // 视频test模式
    int video_test_ = 0;
    std::string input_yuv_name_;

    // 桌面录制属性
    int desktop_x_ = 0;
    int desktop_y_ = 0;
    int desktop_width_ = 1920;
    int desktop_height_ = 1080;
    int desktop_format_ = AV_PIX_FMT_YUV420P;
    int desktop_fps_ = 25;

    // 视频相关参数
    // 视频编码属性
    int video_width_ = 1920;   // 宽
    int video_height_ = 1080;  // 高
    int video_fps_;            // 帧率
    int video_gop_;
    int video_bitrate_;
    int video_b_frames_;  // b帧数量

    // 视频相关
    VideoCapturer *video_capturer_ = NULL;
    H264Encoder *video_encoder_ = NULL;

    // dump 数据
    FILE *pcm_s16le_fp_ = NULL;
    FILE *aac_fp_ = NULL;
    FILE *h264_fp_ = NULL;
    AVFrame *audio_frame_ = NULL;

    // rtsp
    std::string rtsp_url_;
    std::string rtsp_transport_ = "";
    int rtsp_timeout_ = 5000;
    int rtsp_max_queue_duration_ = 500;
    RtspPusher *rtsp_pusher_ = NULL;
    MessageQueue *msg_queue_ = NULL;
};

#endif  // PUSHWORK_H
