#include "rtp-payload.h"
//#include "rtp-profile.h"
#include <assert.h>
#include <stdlib.h>
#include <string.h>

#include "rtp-packet.h"
#include "rtp-payload-internal.h"

#define TS_PACKET_SIZE 188

//#if defined(OS_WINDOWS)
//#define strcasecmp _stricmp
//#endif
#ifdef WIN32
#define strcasecmp stricmp
#define strncasecmp strnicmp
#endif

struct rtp_payload_delegate_t  // 代理结构体
{
    struct rtp_payload_encode_t* encoder;
    struct rtp_payload_decode_t* decoder;
    void* packer;
};

/// @return 0-ok, <0-error
static int rtp_payload_find(int payload, const char* encoding, struct rtp_payload_delegate_t* codec);

/**
 * @brief rtp_payload_encode_create
 * @param payload  媒体的类型
 * @param name      对应的编码器 H264/H265
 * @param seq
 * @param ssrc
 * @param handler
 * @param cbparam
 * @return
 */
void* rtp_payload_encode_create(int payload, const char* name, uint16_t seq, uint32_t ssrc,
                                struct rtp_payload_t* handler, void* cbparam) {
    int size;
    struct rtp_payload_delegate_t* ctx;

    ctx = calloc(1, sizeof(*ctx));
    if (ctx) {
        size = rtp_packet_getsize();
        if (rtp_payload_find(payload, name, ctx) < 0  // 查找有没有注册该编码器(封装器)
            || NULL == (ctx->packer = ctx->encoder->create(size, (uint8_t)payload, seq, ssrc, handler, cbparam))) {
            free(ctx);
            return NULL;
        }
    }
    return ctx;
}

void rtp_payload_encode_destroy(void* encoder) {
    struct rtp_payload_delegate_t* ctx;
    ctx = (struct rtp_payload_delegate_t*)encoder;
    ctx->encoder->destroy(ctx->packer);
    free(ctx);
}

void rtp_payload_encode_getinfo(void* encoder, uint16_t* seq, uint32_t* timestamp) {
    struct rtp_payload_delegate_t* ctx;
    ctx = (struct rtp_payload_delegate_t*)encoder;
    ctx->encoder->get_info(ctx->packer, seq, timestamp);
}
/**
 * @brief rtp_payload_encode_input 这里是通用的接口
 * @param encoder
 * @param data      data具体是什么媒体类型的数据，接口不关注，具体由ctx->encoder->input去处理
 * @param bytes
 * @param timestamp
 * @return 返回0成功
 */
int rtp_payload_encode_input(void* encoder, const void* data, int bytes, uint32_t timestamp) {
    struct rtp_payload_delegate_t* ctx;
    ctx = (struct rtp_payload_delegate_t*)encoder;
    return ctx->encoder->input(ctx->packer, data, bytes, timestamp);
}

void* rtp_payload_decode_create(int payload, const char* name, struct rtp_payload_t* handler, void* cbparam) {
    struct rtp_payload_delegate_t* ctx;
    ctx = calloc(1, sizeof(*ctx));
    if (ctx) {
        if (rtp_payload_find(payload, name, ctx) < 0 ||
            NULL == (ctx->packer = ctx->decoder->create(handler, cbparam))) {
            free(ctx);
            return NULL;
        }
    }
    return ctx;
}

void rtp_payload_decode_destroy(void* decoder) {
    struct rtp_payload_delegate_t* ctx;
    ctx = (struct rtp_payload_delegate_t*)decoder;
    ctx->decoder->destroy(ctx->packer);
    free(ctx);
}

int rtp_payload_decode_input(void* decoder, const void* packet, int bytes) {
    struct rtp_payload_delegate_t* ctx;
    ctx = (struct rtp_payload_delegate_t*)decoder;
    return ctx->decoder->input(ctx->packer, packet, bytes);
}

// Default max packet size (1500, minus allowance for IP, UDP, UMTP headers)
// (Also, make it a multiple of 4 bytes, just in case that matters.)
// static int s_max_packet_size = 1456; // from Live555 MultiFrameRTPSink.cpp RTP_PAYLOAD_MAX_SIZE
// static size_t s_max_packet_size = 576; // UNIX Network Programming by W. Richard Stevens
static int s_max_packet_size = 1434;  // from VLC

void rtp_packet_setsize(int bytes) { s_max_packet_size = bytes < 564 ? 564 : bytes; }

int rtp_packet_getsize() { return s_max_packet_size; }

static int rtp_payload_find(int payload, const char* encoding, struct rtp_payload_delegate_t* codec) {
    assert(payload >= 0 && payload <= 127);
    if (payload >= 96 && encoding) {
        if (0 == strcasecmp(encoding, "H264")) {
            // H.264 video (MPEG-4 Part 10) (RFC 6184)
            codec->encoder = rtp_h264_encode();
            codec->decoder = rtp_h264_decode();
        } else if (0 == strcasecmp(encoding, "mpeg4-generic")) {
            /// RFC3640 RTP Payload Format for Transport of MPEG-4 Elementary Streams
            /// 4.1. MIME Type Registration (p27)
            codec->encoder = rtp_mpeg4_generic_encode();
            codec->decoder = rtp_mpeg4_generic_decode();
        } else {
            return -1;
        }
    } else {
        return -1;
    }

    return 0;
}
