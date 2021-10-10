#include "aac-util.h"

#include <string.h>
FILE *aac_open_bitstream_file(char *filename) {
    FILE *bits = NULL;
    if (NULL == (bits = fopen(filename, "rb"))) {
        printf("open file error\n");
    }
    return bits;
}

int aac_parse_header(uint8_t *in, aac_header_t *res, uint8_t show) {
    static int frame_number = 0;
    memset(res, 0, sizeof(*res));

    if ((in[0] == 0xFF) && ((in[1] & 0xF0) == 0xF0)) {
        res->id = ((unsigned int)in[1] & 0x08) >> 3;
        res->layer = ((unsigned int)in[1] & 0x06) >> 1;
        res->protection_absent = (unsigned int)in[1] & 0x01;
        res->profile = ((unsigned int)in[2] & 0xc0) >> 6;
        res->sampling_frequency_index = ((unsigned int)in[2] & 0x3c) >> 2;
        res->private_bit = ((unsigned int)in[2] & 0x02) >> 1;
        res->channel_configuration = ((((unsigned int)in[2] & 0x01) << 2) | (((unsigned int)in[3] & 0xc0) >> 6));
        res->original_copy = ((unsigned int)in[3] & 0x20) >> 5;
        res->home = ((unsigned int)in[3] & 0x10) >> 4;
        res->copyright_id = ((unsigned int)in[3] & 0x08) >> 3;
        res->copyright_id_start = (unsigned int)in[3] & 0x04 >> 2;
        res->aac_frame_length = (((((unsigned int)in[3]) & 0x03) << 11) | (((unsigned int)in[4] & 0xFF) << 3) |
                                 ((unsigned int)in[5] & 0xE0) >> 5);
        res->adts_buffer_fullness = (((unsigned int)in[5] & 0x1f) << 6 | ((unsigned int)in[6] & 0xfc) >> 2);
        res->num_raw_data_blocks = ((unsigned int)in[6] & 0x03);

        if (show) {
            printf("adts:id  %d\n", res->id);
            printf("adts:layer  %d\n", res->layer);
            printf("adts:protection_absent  %d\n", res->protection_absent);
            printf("adts:profile  %d\n", res->profile);
            printf("adts:sf_index  %dHz\n", aac_freq[res->sampling_frequency_index]);
            printf("adts:pritvate_bit  %d\n", res->private_bit);
            printf("adts:channel_configuration  %d\n", res->channel_configuration);
            printf("adts:original  %d\n", res->original_copy);
            printf("adts:home  %d\n", res->home);
            printf("adts:copyright_identification_bit  %d\n", res->copyright_id);
            printf("adts:copyright_identification_start  %d\n", res->copyright_id_start);
            printf("adts:aac_frame_length  %d\n", res->aac_frame_length);
            printf("adts:adts_buffer_fullness  %d\n", res->adts_buffer_fullness);
            printf("adts:no_raw_data_blocks_in_frame  %d\n", res->num_raw_data_blocks);
        }

        return 0;
    } else {
        printf("failed to parse adts header\n");
        return -1;
    }
}
/*
static const AttrNameMap attr_names[] = {
    { "SizeLength",       ATTR_NAME_TYPE_INT,
      offsetof(PayloadContext, sizelength) },
    { "IndexLength",      ATTR_NAME_TYPE_INT,
      offsetof(PayloadContext, indexlength) },
    { "IndexDeltaLength", ATTR_NAME_TYPE_INT,
      offsetof(PayloadContext, indexdeltalength) },
    { "profile-level-id", ATTR_NAME_TYPE_INT,
      offsetof(PayloadContext, profile_level_id) },
    { "StreamType",       ATTR_NAME_TYPE_INT,
      offsetof(PayloadContext, streamtype) },
    { "mode",             ATTR_NAME_TYPE_STR,
      offsetof(PayloadContext, mode) },
    { NULL, -1, -1 },
};
可以参考ffmpeg sdp.c
SDP中几个参数含义：config，就是AudioSpecificConfig的十六进制表示
*/
void aac_rtp_create_sdp(uint8_t *file, uint8_t *ip, uint16_t port, uint16_t profile, uint16_t chn, uint16_t freq,
                        uint16_t type) {
    char buff[1024] = {0};
    char typeName[64] = {0};
    // char demo[] =
    //     "m=audio 9832 RTP/AVP 97\n"
    //     "a=rtpmap:97 mpeg4-generic/44100/2\n"
    //     "a=fmtp:97 sizeLength=13;mode=AAC-hbr;config=1210;\n"
    //     "c=IN IP4 127.0.0.1";
    char demo[] =
        "m=audio %d RTP/AVP %d\n"
        "a=rtpmap:%d %s/%d/%d\n"
        "a=fmtp:%d "
        "streamtype=5;profile-level-id=1;sizeLength=13;IndexLength=3;indexDeltaLength=3;mode=AAC-hbr;config=%d;\n"
        "c=IN IP4 %s";
    uint16_t config = 1410, _freq = 8;
    int fd;

    strcpy(typeName, "mpeg4-generic");  // 这里先支持AAC该类型的

    if (freq == 96000)
        _freq = 0;
    else if (freq == 88200)
        _freq = 1;
    else if (freq == 64000)
        _freq = 2;
    else if (freq == 48000)
        _freq = 3;
    else if (freq == 44100)
        _freq = 4;
    else if (freq == 32000)
        _freq = 5;
    else if (freq == 24000)
        _freq = 6;
    else if (freq == 22050)
        _freq = 7;
    else if (freq == 16000)
        _freq = 8;
    else if (freq == 12000)
        _freq = 9;
    else if (freq == 11025)
        _freq = 10;
    else if (freq == 8000)
        _freq = 11;
    else if (freq == 7350)
        _freq = 12;
    // 从aac adts header读取出来的profile是被减1的
    config = profile + 1;
    config <<= 5;  // 5	aac的profile, 通常情况是1, 或者2
    config |= _freq;
    config <<= 4;  // aac的采样频率的索引
    config |= chn;
    config <<= 3;  //
    //转成16进制
    config = ((config >> 12) & 0xF) * 1000 + ((config >> 8) & 0xF) * 100 + ((config >> 4) & 0xF) * 10 +
             ((config >> 0) & 0xF);
    snprintf(buff, sizeof(buff), demo, port, type, type, typeName, freq, chn, type, config, ip);
    remove(file);
    if ((fd = fopen(file, "wt")) > 0) {
        fwrite(buff, strlen(buff), 1, fd);
        fclose(fd);
    }
}

//----------------- AAC -------------------

int aac_freq[] = {96000, 88200, 64000, 48000, 44100, 32000, 24000, 22050, 16000, 12000, 11025, 8000, 7350};

//这里没有去检测sync word，如果中间有数据丢失该函数将不起作用
int aac_get_one_frame(aac_frame_t *aac_frame, FILE *bits) {
    // 1. 先读取7个字节，放到adts_buf
    if (7 != fread(aac_frame->adts_buf, 1, 7, bits))  //从码流中读3个字节
    {
        printf("read adts_buf failed\n");
        return -1;
    }
    // 2.解析adts header
    if (aac_parse_header(aac_frame->adts_buf, &aac_frame->header, 0) < 0) {
        return -1;
    }
    // 3.根据解析结果查看有没有校验
    if (0 == aac_frame->header.protection_absent) {
        aac_frame->adts_len = 9;  // 变成adts header长度就是9个字节
        // 0表示有CRC校验
        if (2 != fread(&aac_frame->adts_buf[7], 1, 2, bits))  //从码流中读2个字节
        {
            return -1;
        }
    } else {
        aac_frame->adts_len = 7;
    }
    aac_frame->frame_len = aac_frame->header.aac_frame_length;               // 整帧长度
    memcpy(aac_frame->frame_buf, aac_frame->adts_buf, aac_frame->adts_len);  // 拷贝adts header

    if ((aac_frame->frame_len - aac_frame->adts_len)  // 从文件读取data部分
        != fread(&aac_frame->frame_buf[aac_frame->adts_len], 1, aac_frame->frame_len - aac_frame->adts_len, bits)) {
        printf("read aac frame data failed\n");
        return -1;
    }

    return 0;
}
