#ifndef __BMCAN_BUS_H
#define __BMCAN_BUS_H

#include <iostream>
#include <string.h>

#include "../lib/include/bmapi.h" 
#include "../lib/include/bm_usb_def.h"

class BMCANTool
{
public:
    BMCANTool();
    ~BMCANTool();
    BM_NotificationHandle open(BM_ChannelHandle &bm_channel, const char* channelName);
    BM_StatusTypeDef close(BM_ChannelHandle bm_channel);
    BM_StatusTypeDef can_send(BM_ChannelHandle bm_channel, int device_id, uint8_t* txdata, int timeout);
    BM_StatusTypeDef can_receive(BM_ChannelHandle bm_channel, BM_CanMessageTypeDef &msg, int timeout_ms);
    BM_StatusTypeDef can_receive(BM_ChannelHandle bm_channel, int device_id, uint8_t* rxdata, int timeout_ms=10);
private:
    // 总线上设备信息
    int nchannels;
    BM_ChannelInfoTypeDef channelinfos[32];
    BM_StatusTypeDef error;
    BM_NotificationHandle notification;
};
#endif
