#include <iostream>
#include <string>
#include <unistd.h>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/idl/ros2/String_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/g1/audio/g1_audio_client.hpp>

#define AUDIO_SUBSCRIBE_TOPIC "rt/audio_msg"

// 全局变量：AudioClient 用于TTS播报
unitree::robot::g1::AudioClient* g_audio_client = nullptr;

// 预设的三种TTS文本（中文版本）
std::string preset_texts_cn[3] = {
  "收到主人，小优这就前往冰箱寻找牛奶",
  "收到主人，小忧这就前往冰箱寻找果汁",
  "收到主人，小优这就前往冰箱寻找汽水"
};

/**
 * 检查文本中是否包含关键词，并返回对应的选项编号
 * @param text 识别的文本
 * @return 选项编号 (1=牛奶, 2=果汁, 3=汽水, 0=未匹配)
 */
int detect_keyword(const std::string& text) {
  std::string lower_text = text;
  // 转换为小写以便匹配（如果需要）
  // 这里直接匹配中文，不需要转换
  
  // 检查关键词
  if (text.find("牛奶") != std::string::npos) {
    return 1;
  } else if (text.find("果汁") != std::string::npos) {
    return 2;
  } else if (text.find("汽水") != std::string::npos) {
    return 3;
  }
  
  return 0;  // 未匹配
}

/**
 * ASR消息处理回调函数
 * 当收到语音识别结果时，此函数会被调用
 */
void asr_handler(const void *msg) {
  std_msgs::msg::dds_::String_ *resMsg = (std_msgs::msg::dds_::String_ *)msg;
  std::string recognized_text = resMsg->data();
  
  std::cout << "[ASR Result] " << recognized_text << std::endl;
  
  // 检测关键词
  int option = detect_keyword(recognized_text);
  
  if (option > 0 && g_audio_client != nullptr) {
    std::cout << "[Detected] 识别到关键词，选项: " << option << std::endl;
    
    // 播报对应的TTS文本
    std::string tts_text = preset_texts_cn[option - 1];
    std::cout << "[TTS] 播报: " << tts_text << std::endl;
    
    int32_t ret = g_audio_client->TtsMaker(tts_text, 0);  // 0=中文
    std::cout << "[TTS] TtsMaker API ret: " << ret << std::endl;
    
    if (ret == 0) {
      // 等待播放完成
      unitree::common::Sleep(5);
      std::cout << "[TTS] 播报完成" << std::endl;
    } else {
      std::cout << "[TTS] 播报失败，错误码: " << ret << std::endl;
    }
  } else if (option == 0) {
    std::cout << "[Info] 未识别到关键词（牛奶/果汁/汽水）" << std::endl;
  }
}

int main(int argc, char const *argv[]) {
  // 检查命令行参数
  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " [NetWorkInterface(eth0)]" << std::endl;
    std::cout << "Example: " << argv[0] << " eth0" << std::endl;
    std::cout << std::endl;
    std::cout << "功能说明:" << std::endl;
    std::cout << "  1. 监听语音识别结果" << std::endl;
    std::cout << "  2. 识别关键词：牛奶、果汁、汽水" << std::endl;
    std::cout << "  3. 自动播报对应的TTS回复" << std::endl;
    std::cout << std::endl;
    std::cout << "支持的语音命令:" << std::endl;
    std::cout << "  - \"我想喝牛奶\" -> 播报选项1" << std::endl;
    std::cout << "  - \"我想喝果汁\" -> 播报选项2" << std::endl;
    std::cout << "  - \"我想喝汽水\" -> 播报选项3" << std::endl;
    exit(0);
  }

  std::cout << "Initializing Voice Command System..." << std::endl;
  std::cout << "Network interface: " << argv[1] << std::endl;

  int32_t ret;
  /*
   * Initilaize ChannelFactory
   */
  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
  
  /*
   * 初始化 AudioClient
   */
  unitree::robot::g1::AudioClient client;
  client.Init();
  client.SetTimeout(10.0f);
  g_audio_client = &client;  // 设置全局指针

  /*Volume Example - 设置音量 */
  uint8_t volume;
  ret = client.GetVolume(volume);
  std::cout << "GetVolume API ret:" << ret
            << "  volume = " << std::to_string(volume) << std::endl;
  ret = client.SetVolume(100);
  std::cout << "SetVolume to 100% , API ret:" << ret << std::endl;

  /*
   * 订阅ASR消息主题
   * 当机器人识别到语音时，会通过此主题发送识别结果
   */
  std::cout << "Subscribing to ASR topic: " << AUDIO_SUBSCRIBE_TOPIC << std::endl;
  unitree::robot::ChannelSubscriber<std_msgs::msg::dds_::String_> subscriber(
      AUDIO_SUBSCRIBE_TOPIC);
  subscriber.InitChannel(asr_handler);

  std::cout << std::endl;
  std::cout << "========================================" << std::endl;
  std::cout << "Voice Command System Started!" << std::endl;
  std::cout << "Waiting for voice commands..." << std::endl;
  std::cout << "Supported commands:" << std::endl;
  std::cout << "  - 我想喝牛奶" << std::endl;
  std::cout << "  - 我想喝果汁" << std::endl;
  std::cout << "  - 我想喝汽水" << std::endl;
  std::cout << "Press Ctrl+C to exit." << std::endl;
  std::cout << "========================================" << std::endl;
  std::cout << std::endl;

  /*
   * 主循环：持续运行以接收ASR消息
   */
  while (1) {
    sleep(1);  // 等待ASR消息
  }

  return 0;
}

