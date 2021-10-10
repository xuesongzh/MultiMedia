#ifndef H264ENCODER_H
#define H264ENCODER_H
#include "mediabase.h"
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
}

class H264Encoder {
 public:
    H264Encoder();
    virtual ~H264Encoder();
    /**
     * @brief Init
     * @param   width 宽
     *          height 高
     *          fps 帧率
     *          b_frames b帧连续数量
     *          bitrate 比特率
     *          gop 多少帧有一个I帧
     *          pix_fmt 像素格式
     * @return
     */
    virtual int Init(const Properties &properties);
    virtual AVPacket *Encode(uint8_t *yuv, int size, int64_t pts, int *pkt_frame, RET_CODE *ret);
    inline uint8_t *get_sps_data() {
        return (uint8_t *)sps_.c_str();
    }
    inline int get_sps_size() {
        return sps_.size();
    }
    inline uint8_t *get_pps_data() {
        return (uint8_t *)pps_.c_str();
    }
    inline int get_pps_size() {
        return pps_.size();
    }
    inline int GetFps() {
        return fps_;
    }
    AVCodecContext *GetCodecContext() {
        return ctx_;
    }

 private:
    int width_ = 0;
    int height_ = 0;
    int fps_ = 0;       // 帧率
    int b_frames_ = 0;  // 连续B帧的数量
    int bitrate_ = 0;   // 码率
    int gop_ = 0;
    bool annexb_ = false;
    int threads_ = 1;
    int pix_fmt_ = 0;
    //    std::string profile_;
    //    std::string level_id_;

    std::string sps_;
    std::string pps_;
    std::string codec_name_;
    AVCodec *codec_ = NULL;
    AVCodecContext *ctx_ = NULL;
    AVDictionary *dict_ = NULL;

    AVFrame *frame_ = NULL;
};

#endif  // H264ENCODER_H
