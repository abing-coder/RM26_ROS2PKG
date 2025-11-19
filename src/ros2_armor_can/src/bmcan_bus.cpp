#include "bmcan_bus.hpp"
#include <regex>

BMCANTool::BMCANTool(){
    /*初始化bmapi lib*/
    std::cout << "Initializing BMCAN Tool..." << std::endl;
    std::cout << "Initializing BMAPI libary..." << std::endl;

    error = BM_Init();
    if (error != BM_ERROR_OK)
    {
      std::cout << "BMAPI libary init failed" << std::endl;
    }

    /*枚举设备连接个数*/
    std::cout << "Enumerating channels ..." << std::endl;
    nchannels = sizeof(channelinfos) / sizeof(channelinfos[0]);

    error = BM_Enumerate(channelinfos, &nchannels);
    if(error != BM_ERROR_OK)
    {
      std::cout << "enumerate channels failed" << std::endl;
    }
    else{
      std::cout << "Find "<< nchannels << " BMCANFD devices" << std::endl;
      for(int i = 0;i < nchannels; i++)
      {
        std::cout<< "BMCANFD device:" << i <<"  channel name:" << channelinfos[i].name <<std::endl;
      }
    }    
}

BM_NotificationHandle BMCANTool::open(BM_ChannelHandle &bm_channel, const char* channelName){
    int bm_channel_id;
    if(channelName == nullptr || strcmp(channelName, "") == 0){
      if (nchannels < 1){
        return NULL;
        std::cout << "open channel failed, no device found" << std::endl;
      }else if( nchannels == 1 ){
        bm_channel_id = 0;
      }else{
        std::cout << "open channel failed, multiple devices found" << std::endl;
        std::cout << "default open channel 0" << std::endl;
        bm_channel_id = 0;
      }
    }else{
      std::string str_in_string1(channelName);
      for(int i = 0; i < this->nchannels; i++)
      { 
        const char* str = this->channelinfos[i].name;
        std::string str_in_string2(str);
  
        // 使用std::string::find进行查找
        size_t found = str_in_string2.find(str_in_string1);
        if (found != std::string::npos) {
          bm_channel_id = i;
          break;
        }else{
          bm_channel_id = -1;
        }
      }
      /*判断id是否在设备检测范围内*/
      if(bm_channel_id == -1)
      {
        return NULL;
        std::cout << "open channel failed" << std::endl;
      }
    }

    /*配置比特率*/
    BM_BitrateTypeDef bitrate;
    memset(&bitrate, 0, sizeof(bitrate));
    bitrate.nbitrate = 1000;       // 1 Mbps
    bitrate.dbitrate = 2000;       // 1 Mbps (仅 CAN-FD 模式有效)
    bitrate.nsamplepos = 75;       // 采样点位置为 75%
    bitrate.dsamplepos = 75;       // 数据采样点位置为 75%
    // bitrate.clockfreq = 16;     // CAN 控制器时钟频率为 16 MHz
    // bitrate.reserved = 0;       // 保留字段
    // bitrate.nbtr0 = 0x00;       // 标称 BTR0 = 0x00
    // bitrate.nbtr1 = 0x1C;       // 标称 BTR1 = 0x1C
    // bitrate.dbtr0 = 0x00;       // 数据 BTR0（仅 CAN-FD 模式有效）
    // bitrate.dbtr1 = 0x00;       // 数据 BTR1（仅 CAN-FD 模式有效）

    /*开启can通道*/
    error = BM_OpenEx(&bm_channel, &channelinfos[bm_channel_id], BM_CAN_NORMAL_MODE, BM_TRESISTOR_120, &bitrate, NULL,0);
    if (error != BM_ERROR_OK)
    {
      std::cout << "open channel failed" << std::endl;
    } 

    /*获取通道信息*/
    printf("Getting channel notification handle ...\n");
    error = BM_GetNotification(bm_channel, &this->notification);
    if (error != BM_ERROR_OK)
    {
      std::cout << "get channel notification failed" << std::endl;
      return nullptr;
    }

    return notification;
}

BM_StatusTypeDef BMCANTool::close(BM_ChannelHandle bm_channel){
    error = BM_Close(bm_channel);
    return error;
}

BM_StatusTypeDef BMCANTool::can_send(BM_ChannelHandle bm_channel, int device_id, uint8_t* txdata, int timeout){
		/* Transmit the CAN message to bus using opened device */
		BM_CanMessageTypeDef msg;
    memset(&msg, 0, sizeof(msg));
  
    msg.id.SID = device_id;
    msg.ctrl.tx.DLC = 0x08;

    for(int i = 0; i < 8; i++)
    {
      msg.payload[i] = txdata[i];
    }
    // printf("sizeof int: %d\n", sizeof(int));
    printf("\033[36m[DEBUG][CANTX]\033[0m id: 0x%X, msg.payload[0-7]: ", msg.id.SID);
    for (int i = 0; i < 8; ++i) {
      printf("0x%02X ", msg.payload[i]);
    }
    printf("\n");
    
    uint32_t timestamp = 0;
		BM_GetTimestamp(bm_channel, &timestamp);

		error = BM_WriteCanMessage(bm_channel, &msg, 0, timeout, &timestamp);
		if (error == BM_ERROR_BUSTIMEOUT)
		{
			printf("\033[31m[DEBUG][CANTX]\033[0m \033[31m发送超时\033[0m\n");
      // printf("\033[31m[DEBUG][CANTX]\033[0m \033[31m发送失败 通道:%d, 设备id:0x%03X, 错误:0x%08X.\033[0m\n", bm_channel, device_id, error);
		}
		else if (BM_ERROR_OK != error)
		{
			// printf("\033[31m[DEBUG][CANTX]\033[0m \033[31m发送失败 错误:0x%08X.\033[0m\n", error);
		}
    return error;
}



BM_StatusTypeDef BMCANTool::can_receive(BM_ChannelHandle bm_channel, BM_CanMessageTypeDef &msg, int timeout_ms){
    BM_StatusTypeDef error = BM_ERROR_OK;
    uint32_t port;
    /* Wait until a new notification event arrived */
    if (BM_WaitForNotifications(&notification, 1, timeout_ms) < 0) 
    {
      // printf("Receive timeout.\n");
    }
    uint32_t timestamp;
    error = BM_ReadCanMessage(bm_channel, &msg, &port, &timestamp);
    return error;
}

BM_StatusTypeDef BMCANTool::can_receive(BM_ChannelHandle bm_channel, int device_id, uint8_t* rxdata, int timeout_ms){
  BM_CanMessageTypeDef msg;
  memset(&msg, 0, sizeof(msg));
  int found = 0;
  int max_read = 20; // 防止死循环

  for (int i = 0; i < max_read; ++i) {
    BM_StatusTypeDef ret = this->can_receive(bm_channel, msg, i == 0 ? timeout_ms : 0); // 只有第一次等timeout
    if (ret == BM_ERROR_OK) {
      if (msg.id.SID == device_id && msg.ctrl.rx.DLC == 8) {
        for(int j=0; j<8; ++j) rxdata[j] = msg.payload[j];
        found = 1;
        // 不break，继续把队列清空
      }
    } else {
      break; // 没有更多新帧
    }
  }
  if (!found) {
    for(int i=0; i<8; ++i) rxdata[i] = 0;
    return BM_ERROR_BUSTIMEOUT;
  }
  return BM_ERROR_OK;
}

BMCANTool::~BMCANTool(){
  BM_UnInit();
}