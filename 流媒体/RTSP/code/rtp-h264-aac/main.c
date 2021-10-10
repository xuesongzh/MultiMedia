/*
c++实现windows和linux之间跨平台的方式
操作系统判定：
Windows:   WIN32
Linux:   linux
Solaris:   __sun

编译器判定：
VC:  _MSC_VER
GCC/G++:   __GNUC__
SunCC:   __SUNPRO_C和__SUNPRO_CC
*/

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>

#include "aac-util.h"
#include "rtp-payload.h"
//#define _LINUX_PLATFORM_ 1

#ifdef linux
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#endif

//#if defined(_WIN32) || defined(_WIN64)

//#define strcasecmp _stricmp
//#endif

#ifdef WIN32
#include <winsock2.h>
#pragma comment(lib, "ws2_32.lib")
#define strcasecmp stricmp
#define strncasecmp strnicmp

BOOL InitWinsock() {
    int Error;
    WORD VersionRequested;
    WSADATA WsaData;
    VersionRequested = MAKEWORD(2, 2);
    Error = WSAStartup(VersionRequested, &WsaData);  //启动WinSock2
    if (Error != 0) {
        return FALSE;
    } else {
        if (LOBYTE(WsaData.wVersion) != 2 || HIBYTE(WsaData.wHighVersion) != 2) {
            WSACleanup();
            return FALSE;
        }
    }
    return TRUE;
}

#endif

struct rtp_aac_test_t  // 这里的aac用 mpeg4-generic
{
    int payload;           // payload type
    const char* encoding;  // 音频、视频的格式，这里需要测试aac
    int fd;
    struct sockaddr_in addr;
    size_t addr_size;

    char* in_file_name;  // aac 文件名
    FILE* in_file;       // aac 裸流文件
    // aac相关的参数
    int profile;
    int sampling_frequency_index;  // 采样率
    int channel_configuration;

    void* encoder_aac;  // 封装

    char* out_file_name;  // 输出的文件
    FILE* out_file;
    void* decoder_aac;  // 解封装
};

static void* rtp_alloc(void* param, int bytes) {
    static uint8_t buffer[2 * 1024 * 1024 + 4] = {
        0,
        0,
        0,
        1,
    };  // 支持2M大小，不包括start code
    assert(bytes <= sizeof(buffer) - 4);
    return buffer + 4;
}

static void rtp_free(void* param, void* packet)  // 因为rtp_alloc是静态分配所以不需要释放
{}

// 拿到一帧RTP序列化后的数据
static int rtp_encode_packet(void* param, const void* packet, int bytes, uint32_t timestamp, int flags) {
    struct rtp_aac_test_t* ctx = (struct rtp_aac_test_t*)param;
    int ret = 0;
#ifdef WIN32
    ret = sendto(ctx->fd, (void*)packet, bytes, 0, (struct sockaddr*)&ctx->addr, ctx->addr_size);
#else
    ret = sendto(ctx->fd, (void*)packet, bytes, MSG_DONTWAIT, (struct sockaddr*)&ctx->addr, ctx->addr_size);
#endif
    //    uint8_t *ptr = (uint8_t *)packet;
    //    for(int i = 0; i < 20; i++)
    //    {
    //        printf("%02x ", ptr[i]);
    //    }
    printf("\nret:%d, rtp send packet -> bytes:%d, timestamp:%u\n", ret, bytes, timestamp);
    ret = rtp_payload_decode_input(ctx->decoder_aac, packet, bytes);  // 解封装
    //    printf("decode ret:%d\n", ret);
    return 0;
}

static int rtp_decode_packet(void* param, const void* packet, int bytes, uint32_t timestamp, int flags) {
    struct rtp_aac_test_t* ctx = (struct rtp_aac_test_t*)param;

    static uint8_t buffer[2 * 1024 * 1024];
    assert(bytes + 4 < sizeof(buffer));
    assert(0 == flags);

    size_t size = 0;
    if (0 == strcasecmp("mpeg4-generic", ctx->encoding)) {
        int len = bytes + 7;
        uint8_t profile = ctx->profile;
        uint8_t sampling_frequency_index = ctx->sampling_frequency_index;  // 本质上这些是从sdp读取的
        uint8_t channel_configuration = ctx->channel_configuration;
        buffer[0] = 0xFF; /* 12-syncword */
        buffer[1] = 0xF0 /* 12-syncword */ | (0 << 3) /*1-ID*/ | (0x00 << 2) /*2-layer*/ | 0x01 /*1-protection_absent*/;
        buffer[2] = ((profile) << 6) | ((sampling_frequency_index & 0x0F) << 2) | ((channel_configuration >> 2) & 0x01);
        buffer[3] = ((channel_configuration & 0x03) << 6) | ((len >> 11) & 0x03);
            /*0-original_copy*/ /*0-home*/ /*0-copyright_identification_bit*/ /*0-copyright_identification_start*/
        buffer[4] = (uint8_t)(len >> 3);
        buffer[5] = ((len & 0x07) << 5) | 0x1F;
        buffer[6] = 0xFC | ((len / 1024) & 0x03);
        size = 7;
    }
    memcpy(buffer + size, packet, bytes);
    size += bytes;
    //    printf("aac get -> bytes:%d, timestamp:%u\n", size, timestamp);
    // TODO:
    // check media file
    fwrite(buffer, 1, size, ctx->out_file);
}
// 获取的时间是ms的，这里目的是为了让音频实时去发送
int64_t get_current_time() {
#ifdef WIN32
    return GetTickCount();
#else
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return ((unsigned long long)tv.tv_sec * 1000 + (long)tv.tv_usec / 1000);
#endif
}

// 发送H264 RTP over UDP
#define DEST_IP "192.168.2.110"
#define DEST_PORT 9832
// ffplay 播放 ffplay aac.sdp -protocol_whitelist "file,http,https,rtp,udp,tcp,tls"
// ffplay 播放 ffplay test.sdp -protocol_whitelist "file,http,https,rtp,udp,tcp,tls" -loglevel 58
int main() {
    // 1.打开本地输入文件，这个文件是要实时发送的
    FILE* bits =
        aac_open_bitstream_file("in.aac");  //打开aac文件，并将文件指针赋给bits,在此修改文件名实现打开别的aac文件。
    if (!bits) {
        printf("open file failed\n");
        return -1;
    }
    // 2.解包后保存的aac文件，主要测试对比in.aac文件是否一致
    FILE* out_file = fopen("out.aac", "wb");
    if (!out_file) {
        printf("open out_file failed\n");
        return -1;
    }

    struct rtp_aac_test_t ctx;  // 封装的测试 带aac RTP封装和解封装
    memset(&ctx, 0, sizeof(struct rtp_aac_test_t));

    ctx.out_file = out_file;  // 输出文件
    ctx.in_file = bits;       // 输入文件

    // AAC RTP encode 回调
    struct rtp_payload_t handler_rtp_encode_aac;
    handler_rtp_encode_aac.alloc = rtp_alloc;
    handler_rtp_encode_aac.free = rtp_free;
    handler_rtp_encode_aac.packet = rtp_encode_packet;

    const char* encoding = "mpeg4-generic";
    ctx.payload = 97;
    ctx.encoding = encoding;
    ctx.encoder_aac = rtp_payload_encode_create(ctx.payload, ctx.encoding, 1, 0x32411, &handler_rtp_encode_aac, &ctx);

    // AAC RTP decode回调
    struct rtp_payload_t handler_rtp_decode_aac;
    handler_rtp_decode_aac.alloc = rtp_alloc;
    handler_rtp_decode_aac.free = rtp_free;
    handler_rtp_decode_aac.packet = rtp_decode_packet;
    ctx.decoder_aac = rtp_payload_decode_create(ctx.payload, ctx.encoding, &handler_rtp_decode_aac, &ctx);

#ifdef WIN32
    InitWinsock();  //初始化套接字库        windows平台需要
#endif

    ctx.addr.sin_family = AF_INET;
    ctx.addr.sin_port = htons(DEST_PORT);
    ctx.addr.sin_addr.s_addr = inet_addr(DEST_IP);
    ctx.fd = socket(AF_INET, SOCK_DGRAM, 0);
#ifdef linux
    int ret = fcntl(ctx.fd, F_GETFL, 0);
    fcntl(ctx.fd, F_SETFL, ret | O_NONBLOCK);
#endif
    ctx.addr_size = sizeof(ctx.addr);

    aac_frame_t* aac_frame = (aac_frame_t*)malloc(sizeof(aac_frame_t));
    if (!aac_frame) {
        printf("malloc aac_frame failed\n");
        return -1;
    }
    memset(aac_frame, 0, sizeof(aac_frame_t));
    if (aac_get_one_frame(aac_frame, bits) < 0)  // 读取一帧aac
    {
        printf("aac_get_one_frame failed\n");
        return -1;
    }

    ctx.profile = aac_frame->header.profile;
    ctx.channel_configuration = aac_frame->header.channel_configuration;
    ctx.sampling_frequency_index = aac_frame->header.sampling_frequency_index;

    aac_rtp_create_sdp("./aac.sdp", DEST_IP, DEST_PORT, aac_frame->header.profile,
                       aac_frame->header.channel_configuration,
                       aac_freq[aac_frame->header.sampling_frequency_index],  // 设置的采样率为 timestamp刻度
                       ctx.payload);                                          // 自定义

    int64_t start_time = get_current_time();
    int64_t cur_time = get_current_time();
    double sum_time = 0;                          // 累加总共可以播放的时长
    double frame_duration = 1024 * 1000 / 44100;  //一帧数据播放的时长
    int64_t frame_samples = (aac_frame->header.adts_buffer_fullness + 1) / 2;

    frame_duration =
        frame_samples * 1000.0 / (aac_freq[aac_frame->header.sampling_frequency_index]);  // 读取aac信息修正帧持续时间

    uint32_t total_size = 0;
    uint32_t timestamp;  //时间戳,us,自增

    while (!feof(bits)) {
        // 传入的aac帧是带adts header的
        int ret = rtp_payload_encode_input(ctx.encoder_aac, aac_frame->frame_buf, aac_frame->frame_len, timestamp);
        timestamp += (aac_frame->header.adts_buffer_fullness + 1) / 2;  // 叠加采样点，单位为1/采样率
                                                                        //        if(total_size > 200*1024)
                                                                        //            break;
        total_size += aac_frame->frame_len;
        printf("ret:%d,frame_len:%d,total_size:%uk, frame_duration:%lf\n", ret, aac_frame->frame_len, total_size / 1024,
               frame_duration);
        sum_time += frame_duration;     // 叠加可以播放的时长
        cur_time = get_current_time();  // darren修正发送aac的时间间隔
        while ((cur_time - start_time) < (int64_t)(sum_time - 50)) {
            //            printf("cur_time - start_time:%ld\n", cur_time - start_time);
            //            printf("sum_time:%lf\n",  sum_time);
#ifdef WIN32
            Sleep(10);
#endif
#ifdef linux
            usleep(10000);
#endif
            cur_time = get_current_time();
            if (feof(bits)) break;
        }
        if (aac_get_one_frame(aac_frame, bits) < 0) {
            printf("aac_get_one_frame failed\n");
            break;
        }
    }

    printf("close file\n");
    fclose(ctx.in_file);
    fclose(ctx.out_file);
    free(aac_frame);
    return 0;
}
