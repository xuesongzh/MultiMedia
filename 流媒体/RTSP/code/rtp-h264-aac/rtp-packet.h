#ifndef _rtp_packet_h_
#define _rtp_packet_h_

#include "rtp-header.h"

#define RTP_FIXED_HEADER 12  // RTP固定 header的长度

struct rtp_packet_t  // 封装这个RTP 包括 header + [csrc/extension] + payload
{
    rtp_header_t rtp;
    uint32_t csrc[16];      // 最多16个csrc
    const void* extension;  // extension(valid only if rtp.x = 1)
    uint16_t extlen;        // extension length in bytes
    uint16_t reserved;      // extension reserved
    const void* payload;    //  rtp payload
    int payloadlen;         // payload length in bytes
};

///@return 0-ok, other-error解包
int rtp_packet_deserialize(struct rtp_packet_t* pkt, const void* data, int bytes);

///@return <0-error, >0-rtp packet size, =0-impossible 封包
int rtp_packet_serialize(const struct rtp_packet_t* pkt, void* data, int bytes);

#endif /* !_rtp_packet_h_ */
