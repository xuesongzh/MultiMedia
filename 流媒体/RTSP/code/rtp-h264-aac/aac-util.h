#ifndef _aac_util_h_
#define _aac_util_h_
#include <stdint.h>
#include <stdio.h>

typedef struct {
    unsigned int syncword;                  // 12 bit 同步字 '1111 1111 1111'，说明一个ADTS帧的开始
    unsigned int id;                        // 1 bit MPEG 标示符， 0 for MPEG-4，1 for MPEG-2
    unsigned int layer;                     // 2 bit 总是'00'
    unsigned int protection_absent;         // 1 bit 1表示没有crc，0表示有crc
    unsigned int profile;                   // 2 bit 表示使用哪个级别的AAC
    unsigned int sampling_frequency_index;  // 4 bit 表示使用的采样频率
    unsigned int private_bit;               // 1 bit
    unsigned int channel_configuration;     // 3 bit 表示声道数
    unsigned int original_copy;             // 1 bit
    unsigned int home;                      // 1 bit

    /*下面的为改变的参数即每一帧都不同*/
    unsigned int copyright_id;          // 1 bit
    unsigned int copyright_id_start;    // 1 bit
    unsigned int aac_frame_length;      // 13 bit 一个ADTS帧的长度包括ADTS头和AAC原始流
    unsigned int adts_buffer_fullness;  // 11 bit 0x7FF adts buffer fullness

    /* number_of_raw_data_blocks_in_frame
     * 表示ADTS帧中有number_of_raw_data_blocks_in_frame + 1个AAC原始帧
     * 所以说number_of_raw_data_blocks_in_frame == 0
     * 表示说ADTS帧中有一个AAC数据块并不是说没有。(一个AAC原始帧包含一段时间内1024个采样及相关数据)
     */
    unsigned int num_raw_data_blocks;  // 2 bit
} aac_header_t;

typedef struct {
    aac_header_t header;      // 存储到是adts头部解析后的参数
    uint8_t adts_buf[9];      // adts header最多9字节，从文件读取出来的数据先放到adts_buf里面
    int adts_len;             // adts 头部长度
    uint8_t frame_buf[8192];  // 包括adts header 13bit最多8192字节
    int frame_len;            // 包括adts header + data length
} aac_frame_t;

extern int aac_freq[];

// 打开文件
FILE *aac_open_bitstream_file(char *filename);
// 根据传入的adts header buffer 解析出来对应的参数， show=1的时候打印解析结果，=0就不打印了
int aac_parse_header(uint8_t *in, aac_header_t *res, uint8_t show);
// 读取一帧完整的AAC帧（adts header + data），不会去查找sync word
int aac_get_one_frame(aac_frame_t *aac_frame, FILE *bits);
void aac_rtp_create_sdp(uint8_t *file, uint8_t *ip, uint16_t port, uint16_t profile, uint16_t chn, uint16_t freq,
                        uint16_t type);
#endif  // _aac_util_h_
