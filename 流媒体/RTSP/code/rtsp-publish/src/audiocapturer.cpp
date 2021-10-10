#include "audiocapturer.h"

#include "avpublishtime.h"
#include "dlog.h"
#include "timesutil.h"

extern "C" {
#include <libavcodec/avcodec.h>
}

AudioCapturer::AudioCapturer() : CommonLooper() {}

AudioCapturer::~AudioCapturer() {
    if (pcm_buf_) {
        delete[] pcm_buf_;
    }
    if (pcm_fp_) {
        fclose(pcm_fp_);
    }
}

RET_CODE AudioCapturer::Init(const Properties &properties) {
    audio_test_ = properties.GetProperty("audio_test", 0);
    input_pcm_name_ = properties.GetProperty("input_pcm_name", "buweishui_48000_2_s16le.pcm");
    sample_rate_ = properties.GetProperty("sample_rate", 48000);
    channels_ = properties.GetProperty("channels", 2);
    byte_per_sample_ = properties.GetProperty("byte_per_sample", 2);  // 单个采样点
    nb_samples_ = properties.GetProperty("nb_samples", 1024);
    format_ = properties.GetProperty("format", AV_SAMPLE_FMT_S16);  //  2
    pcm_buf_size_ = byte_per_sample_ * channels_ * nb_samples_;
    pcm_buf_ = new uint8_t[pcm_buf_size_];
    if (!pcm_buf_) {
        return RET_ERR_OUTOFMEMORY;
    }
    if (openPcmFile(input_pcm_name_.c_str()) < 0) {
        LogError("openPcmFile %s failed", input_pcm_name_.c_str());
        return RET_FAIL;
    }
    frame_duration_ = 1.0 * nb_samples_ / sample_rate_ * 1000;  // 得到一帧的毫秒时间

    return RET_OK;
}

void AudioCapturer::Loop() {
    LogInfo("into loop");
    pcm_total_duration_ = 0;
    pcm_start_time_ = TimesUtil::GetTimeMillisecond();  // 初始化时间基
    while (true) {
        if (request_abort_) {
            break;  // 请求退出
        }
        if (readPcmFile(pcm_buf_, pcm_buf_size_) == 0) {
            if (!is_first_time_) {
                is_first_time_ = true;
                LogInfo("%s:t%u", AVPublishTime::GetInstance()->getAInTag(),
                        AVPublishTime::GetInstance()->getCurrenTime());
            }
            if (callback_get_pcm_) {
                callback_get_pcm_(pcm_buf_, pcm_buf_size_);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    request_abort_ = false;
    closePcmFile();
}

void AudioCapturer::AddCallback(function<void(uint8_t *, int32_t)> callback) { callback_get_pcm_ = callback; }

int AudioCapturer::openPcmFile(const char *file_name) {
    pcm_fp_ = fopen(file_name, "rb");
    if (!pcm_fp_) {
        return -1;
    }
    return 0;
}

int AudioCapturer::readPcmFile(uint8_t *pcm_buf, int32_t pcm_buf_size) {
    int64_t cur_time = TimesUtil::GetTimeMillisecond();  // 单位毫秒
    int64_t diff = cur_time - pcm_start_time_;           // 目前经过的时间
    if (((int64_t)pcm_total_duration_) > diff) {
        return 1;  // 还没有到读取新一帧的时间
    }
    // 读取数据
    size_t ret = fread(pcm_buf_, 1, pcm_buf_size, pcm_fp_);
    if (ret != pcm_buf_size) {
        ret = fseek(pcm_fp_, 0, SEEK_SET);  // 没有足够数据就从头继续开始读取
        ret = fread(pcm_buf_, 1, pcm_buf_size, pcm_fp_);
        if (ret != pcm_buf_size) {
            return -1;  // 出错
        }
    }

    pcm_total_duration_ += frame_duration_;
    return 0;
}

int AudioCapturer::closePcmFile() {
    if (pcm_fp_)
        fclose(pcm_fp_);
    return 0;
}
