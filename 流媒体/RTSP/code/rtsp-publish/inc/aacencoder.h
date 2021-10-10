#ifndef AACENCODER_H
#define AACENCODER_H

extern "C" {
#include <libavcodec/avcodec.h>
}

#include "mediabase.h"
class AACEncoder {
 public:
    AACEncoder();
    virtual ~AACEncoder();
    /**
     * @brief Init
     * @param "sample_rate", 采样率，默认48000
     *        "channels",通道数量 ，默认2
     *        "bitrate"， 比特率，默认128*1024
     *        "channel_layout"， 通道布局，默认根据channels获取缺省的
     * @return
     */
    RET_CODE Init(const Properties &properties);
    /**
     * @brief Encode
     * @param frame     输入帧
     * @param pts       时间戳
     * @param flush     是否flush
     * @param pkt_frame *pkt_frame = 0，receive_packet报错; *pkt_frame = 1, send_frame报错
     * @param ret   只有RET_OK才不需要做异常处理
     * @return
     */
    virtual AVPacket *Encode(AVFrame *frame, const int64_t pts, int flush, int *pkt_frame, RET_CODE *ret);

    RET_CODE GetAdtsHeader(uint8_t *adts_header, int aac_length);

    virtual int GetFormat() {
        return ctx_->sample_fmt;
    }
    virtual int GetChannels() {
        return ctx_->channels;
    }
    virtual int GetChannelLayout() {
        return ctx_->channel_layout;
    }
    // 一帧有多少个采样点
    virtual int GetFrameSamples() {  // 采样点数量，只是说的一个通道
        return ctx_->frame_size;
    }
    virtual int GetSampleRate() {  // 采样点数量，只是说的一个通道
        return ctx_->sample_rate;
    }
    // 一帧占用的字节数
    virtual int GetFrameBytes() {
        return av_get_bytes_per_sample(ctx_->sample_fmt) * ctx_->channels * ctx_->frame_size;
    }
    AVCodecContext *GetCodecContext() {
        return ctx_;
    }
    //    virtual RET_CODE EncodeInput(const AVFrame *frame);
    //    virtual RET_CODE EncodeOutput(AVPacket *pkt);

 private:
    int sample_rate_ = 48000;
    int channels_ = 2;
    int bitrate_ = 128 * 1024;
    int channel_layout_ = AV_CH_LAYOUT_STEREO;

    AVCodec *codec_ = NULL;
    AVCodecContext *ctx_ = NULL;
};

#endif  // AACENCODER_H
