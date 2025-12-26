#include <iostream>
#include <unistd.h>
#include <unitree/idl/ros2/String_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/g1/audio/g1_audio_client.hpp>

#define AUDIO_SUBSCRIBE_TOPIC "rt/audio_msg"

/**
 * ASR消息处理回调函数
 * 当收到语音识别结果时，此函数会被调用
 */
void asr_handler(const void *msg) {
  std_msgs::msg::dds_::String_ *resMsg = (std_msgs::msg::dds_::String_ *)msg;
  std::cout << "[ASR Result] " << resMsg->data() << std::endl;
}

int main(int argc, char const *argv[]) {
  // 检查命令行参数
  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " [NetWorkInterface(eth0)]" << std::endl;
    std::cout << "Example: " << argv[0] << " eth0" << std::endl;
    exit(0);
  }

  std::cout << "Initializing ASR Client..." << std::endl;

  /*
   * 初始化 ChannelFactory
   * 参数: domain_id, 网络接口名称
   */
  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);

  /*
   * 初始化 AudioClient (可选，但建议初始化以确保通信正常)
   */
  unitree::robot::g1::AudioClient client;
  client.Init();
  client.SetTimeout(10.0f);

  /*
   * 订阅ASR消息主题
   * 当机器人识别到语音时，会通过此主题发送识别结果
   */
  std::cout << "Subscribing to ASR topic: " << AUDIO_SUBSCRIBE_TOPIC << std::endl;
  unitree::robot::ChannelSubscriber<std_msgs::msg::dds_::String_> subscriber(
      AUDIO_SUBSCRIBE_TOPIC);
  subscriber.InitChannel(asr_handler);

  std::cout << "ASR Client started. Waiting for speech recognition results..." << std::endl;
  std::cout << "Press Ctrl+C to exit." << std::endl;

  /*
   * 主循环：持续运行以接收ASR消息
   */
  while (1) {
    sleep(1);  // 等待ASR消息
  }

  return 0;
}

