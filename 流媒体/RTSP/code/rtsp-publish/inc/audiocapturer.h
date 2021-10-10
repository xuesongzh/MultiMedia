#ifndef AUDIOCAPTURER_H
#define AUDIOCAPTURER_H

#include <functional>

#include "commonlooper.h"
#include "mediabase.h"
using std::function;

class AudioCapturer : public CommonLooper {
 public:
    AudioCapturer();
    virtual ~AudioCapturer();
    RET_CODE Init(const Properties &properties);

    virtual void Loop();
    void AddCallback(function<void(uint8_t *, int32_t)> callback);

 private:
    // PCM file只是用来测试, 写死为s16格式 2通道 采样率48Khz
    // 1帧1024采样点持续的时间21.333333333333333333333333333333ms
    int openPcmFile(const char *file_name);
    int readPcmFile(uint8_t *pcm_buf, int32_t pcm_buf_size);
    int closePcmFile();

    int audio_test_ = 0;
    std::string input_pcm_name_;  // 输入pcm测试文件的名字
    FILE *pcm_fp_ = NULL;
    int64_t pcm_start_time_ = 0;
    double pcm_total_duration_ = 0;  // 推流时长的统计
    double frame_duration_ = 23.2;

    std::function<void(uint8_t *, int32_t)> callback_get_pcm_;
    uint8_t *pcm_buf_;
    int32_t pcm_buf_size_;
    bool is_first_time_ = false;
    int sample_rate_ = 48000;
    int nb_samples_ = 1024;
    int byte_per_sample_ = 2;
    int format_ = 1;  // 目前固定s16
    int channels_ = 2;
};

#endif  // AUDIOCAPTURER_H
