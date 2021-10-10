#ifndef _h264_util_h_
#define _h264_util_h_

#include <stdint.h>
#include <stdio.h>
typedef struct _nalu_t {
    int startcodeprefix_len;      //! 4 for parameter sets and first slice in picture, 3 for everything else (suggested)
    unsigned len;                 //! Length of the NAL unit (include the start code, which does not belong to the NALU)
    unsigned max_size;            //! Nal Unit Buffer size
    int forbidden_bit;            //! should be always FALSE
    int nal_reference_idc;        //! NALU_PRIORITY_xxxx
    int nal_unit_type;            //! NALU_TYPE_xxxx
    char *buf;                    //! include start code
    unsigned short lost_packets;  //! true, if packet loss is detected
} nalu_t;

// 打开文件
FILE *open_bitstream_file(char *filename);
// 分配nalu
nalu_t *alloc_nalu(int buffersize);
// 释放nalu
void free_nalu(nalu_t *n);

void h264_sdp_create(uint8_t *file, uint8_t *ip, uint16_t port, const uint8_t *sps, const int sps_len,
                     const uint8_t *pps, const int pps_len, int payload_type, int time_base, int bitrate);

#endif  // H264UTIL_H
