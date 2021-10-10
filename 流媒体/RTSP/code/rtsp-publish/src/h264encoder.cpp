#include "h264encoder.h"

#include "dlog.h"
H264Encoder::H264Encoder() {
}

H264Encoder::~H264Encoder() {
    if (ctx_) {
        avcodec_free_context(&ctx_);
    }
    if (frame_) {
        av_frame_free(&frame_);
    }
}

int H264Encoder::Init(const Properties &properties) {
    int ret = 0;
    width_ = properties.GetProperty("width", 0);
    if (width_ == 0 || width_ % 2 != 0) {
        LogError("width:%d", width_);
        return RET_ERR_NOT_SUPPORT;
    }

    height_ = properties.GetProperty("height", 0);
    if (height_ == 0 || height_ % 2 != 0) {
        LogError("height:%d", height_);
        return RET_ERR_NOT_SUPPORT;
    }

    fps_ = properties.GetProperty("fps", 25);
    b_frames_ = properties.GetProperty("b_frames", 0);
    bitrate_ = properties.GetProperty("bitrate", 500 * 1024);
    gop_ = properties.GetProperty("gop", fps_);
    pix_fmt_ = properties.GetProperty("pix_fmt", AV_PIX_FMT_YUV420P);

    codec_name_ = properties.GetProperty("codec_name", "default");
    // 查找H264编码器 确定是否存在
    if (codec_name_ == "default") {
        LogInfo("use default encoder");
        codec_ = avcodec_find_encoder(AV_CODEC_ID_H264);
    } else {
        LogInfo("use %s encoder", codec_name_.c_str());
        codec_ = avcodec_find_encoder_by_name(codec_name_.c_str());
    }
    if (!codec_) {
        LogError("can't find encoder");
        return RET_FAIL;
    }
    ctx_ = avcodec_alloc_context3(codec_);
    if (!ctx_) {
        LogError("ctx_ h264 avcodec_alloc_context3 failed");
        return RET_FAIL;
    }
    // 宽高
    ctx_->width = width_;
    ctx_->height = height_;

    // 码率
    ctx_->bit_rate = bitrate_;
    // gop
    ctx_->gop_size = gop_;
    // 帧率
    ctx_->framerate.num = fps_;  // 分子
    ctx_->framerate.den = 1;     // 分母
    // time_base
    ctx_->time_base.num = 1;     // 分子
    ctx_->time_base.den = fps_;  // 分母

    // 像素格式
    ctx_->pix_fmt = (AVPixelFormat)pix_fmt_;

    ctx_->codec_type = AVMEDIA_TYPE_VIDEO;

    ctx_->max_b_frames = b_frames_;

    av_dict_set(&dict_, "preset", "medium", 0);
    av_dict_set(&dict_, "tune", "zerolatency", 0);
    av_dict_set(&dict_, "profile", "high", 0);

    ctx_->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;

    // 初始化音视频编码器
    ret = avcodec_open2(ctx_, codec_, &dict_);
    if (ret < 0) {
        char buf[1024] = {0};
        av_strerror(ret, buf, sizeof(buf) - 1);
        LogError("avcodec_open2 failed:%s", buf);
        return RET_FAIL;
    }

    // 从extradata读取sps pps
    if (ctx_->extradata) {
        LogInfo("extradata_size:%d", ctx_->extradata_size);
        // 第一个为sps 7
        // 第二个为pps 8

        uint8_t *sps = ctx_->extradata + 4;  // 直接跳到数据
        int sps_len = 0;
        uint8_t *pps = NULL;
        int pps_len = 0;
        uint8_t *data = ctx_->extradata + 4;
        for (int i = 0; i < ctx_->extradata_size - 4; ++i) {
            if (0 == data[i] && 0 == data[i + 1] && 0 == data[i + 2] && 1 == data[i + 3]) {
                pps = &data[i + 4];
                break;
            }
        }
        sps_len = int(pps - sps) - 4;  // 4是00 00 00 01占用的字节
        pps_len = ctx_->extradata_size - 4 * 2 - sps_len;
        sps_.append(sps, sps + sps_len);
        pps_.append(pps, pps + pps_len);
    }

    frame_ = av_frame_alloc();
    frame_->width = width_;
    frame_->height = height_;
    frame_->format = ctx_->pix_fmt;
    ret = av_frame_get_buffer(frame_, 0);
    return RET_OK;
}

AVPacket *H264Encoder::Encode(uint8_t *yuv, int size, int64_t pts, int *pkt_frame, RET_CODE *ret) {
    int ret1 = 0;
    *ret = RET_OK;
    *pkt_frame = 0;

    if (yuv) {
        int need_size = 0;
        need_size = av_image_fill_arrays(frame_->data, frame_->linesize, yuv,
                                         (AVPixelFormat)frame_->format,
                                         frame_->width, frame_->height, 1);
        if (need_size != size) {
            LogError("need_size:%d != size:%d", need_size, size);
            *ret = RET_FAIL;
            return NULL;
        }
        frame_->pts = pts;
        frame_->pict_type = AV_PICTURE_TYPE_NONE;
        ret1 = avcodec_send_frame(ctx_, frame_);
    } else {
        ret1 = avcodec_send_frame(ctx_, NULL);
    }
    if (ret1 < 0) {  // <0 不能正常处理该frame
        char buf[1024] = {0};
        av_strerror(ret1, buf, sizeof(buf) - 1);
        LogError("avcodec_send_frame failed:%s", buf);
        *pkt_frame = 1;
        if (ret1 == AVERROR(EAGAIN)) {  // 你赶紧读取packet，我frame send不进去了
            *ret = RET_ERR_EAGAIN;
            return NULL;
        } else if (ret1 == AVERROR_EOF) {
            *ret = RET_ERR_EOF;
            return NULL;
        } else {
            *ret = RET_FAIL;  // 真正报错，这个encoder就只能销毁了
            return NULL;
        }
    }
    AVPacket *packet = av_packet_alloc();
    ret1 = avcodec_receive_packet(ctx_, packet);
    if (ret1 < 0) {
        LogError("AAC: avcodec_receive_packet ret:%d", ret1);
        av_packet_free(&packet);
        *pkt_frame = 0;
        if (ret1 == AVERROR(EAGAIN)) {  // 需要继续发送frame我们才有packet读取
            *ret = RET_ERR_EAGAIN;
            return NULL;
        } else if (ret1 == AVERROR_EOF) {
            *ret = RET_ERR_EOF;  // 不能在读取出来packet来了
            return NULL;
        } else {
            *ret = RET_FAIL;  // 真正报错，这个encoder就只能销毁了
            return NULL;
        }
    } else {
        *ret = RET_OK;
        return packet;
    }
}
