#include <iostream>

#include "dlog.h"
#include "messagequeue.h"
#include "pushwork.h"
using namespace std;

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/audio_fifo.h>
#include <libavutil/opt.h>
#include <libswresample/swresample.h>
}

//减少缓冲 ffplay.exe -i rtmp://xxxxxxx -fflags nobuffer
// 减少码流分析时间 ffplay.exe -i rtmp://xxxxxxx -analyzeduration 1000000 单位为微秒
// ffplay -i rtsp://192.168.2.132/live/livestream -fflags nobuffer -analyzeduration 1000000 -rtsp_transport udp

//#define RTSP_URL "rtsp://111.229.231.225/live/livestream"
#define RTSP_URL "rtsp://192.168.2.132/live/livestream"
// ffmpeg -re -i  rtsp_test_hd.flv  -vcodec copy -acodec copy  -f flv -y rtsp://111.229.231.225/live/livestream
// ffmpeg -re -i  rtsp_test_hd.flv  -vcodec copy -acodec copy  -f flv -y rtsp://192.168.1.12/live/livestream
//// ffmpeg -re -i  1920x832_25fps.flv  -vcodec copy -acodec copy  -f flv -y rtsp://111.229.231.225/live/livestream

int main() {
    cout << "Hello World!" << endl;

    init_logger("rtsp_push.log", S_INFO);
    MessageQueue *msg_queue_ = new MessageQueue();
    //    for(int i = 0; i < 5; i++)
    {
        //        LogInfo("test pushwork:%d", i);

        if (!msg_queue_) {
            LogError("new MessageQueue() failed");
            return -1;
        }
        PushWork push_work(msg_queue_);
        Properties properties;
        // 音频test模式
        properties.SetProperty("audio_test", 1);  // 音频测试模式
        properties.SetProperty("input_pcm_name", "buweishui_48000_2_s16le.pcm");
        // 麦克风采样属性
        properties.SetProperty("mic_sample_fmt", AV_SAMPLE_FMT_S16);
        properties.SetProperty("mic_sample_rate", 48000);
        properties.SetProperty("mic_channels", 2);

        // 音频编码属性
        properties.SetProperty("audio_sample_rate", 48000);
        properties.SetProperty("audio_bitrate", 64 * 1024);
        properties.SetProperty("audio_channels", 2);

        //视频test模式
        properties.SetProperty("video_test", 1);
        properties.SetProperty("input_yuv_name", "720x480_25fps_420p.yuv");
        // 桌面录制属性
        properties.SetProperty("desktop_x", 0);
        properties.SetProperty("desktop_y", 0);
        properties.SetProperty("desktop_width", 720);   //测试模式时和yuv文件的宽度一致
        properties.SetProperty("desktop_height", 480);  //测试模式时和yuv文件的高度一致
        //    properties.SetProperty("desktop_pixel_format", AV_PIX_FMT_YUV420P);
        properties.SetProperty("desktop_fps", 25);  //测试模式时和yuv文件的帧率一致
        // 视频编码属性
        properties.SetProperty("video_bitrate", 512 * 1024);  // 设置码率

        // 配置rtsp
        // 1.url
        // 2.udp
        properties.SetProperty("rtsp_url", RTSP_URL);
        properties.SetProperty("rtsp_transport", "tcp");  // udp or tcp
        properties.SetProperty("rtsp_timeout", 15000);
        properties.SetProperty("rtsp_max_queue_duration", 1000);

        if (push_work.Init(properties) != RET_OK) {
            LogError("PushWork init failed");
            return -1;
        }
        int count = 0;
        AVMessage msg;
        int ret = 0;
        while (true) {
            //            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            ret = msg_queue_->msg_queue_get(&msg, 1000);
            if (1 == ret) {
                switch (msg.what) {
                    case MSG_RTSP_ERROR:
                        LogError("MSG_RTSP_ERROR error:%d", msg.arg1);
                        break;
                    case MSG_RTSP_QUEUE_DURATION:
                        LogError("MSG_RTSP_QUEUE_DURATION a:%d, v:%d", msg.arg1, msg.arg2);
                        break;
                    default:
                        break;
                }
            }
            LogInfo("count:%d, ret:%d", count, ret);
            if (count++ > 100) {
                printf("main break\n");
                break;
            }
        }
        msg_queue_->msg_queue_abort();
    }
    delete msg_queue_;

    LogInfo("main finish");
    return 0;
}
