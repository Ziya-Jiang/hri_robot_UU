#include <iostream>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/idl/ros2/String_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/g1/audio/g1_audio_client.hpp>

#define AUDIO_SUBSCRIBE_TOPIC "rt/audio_msg"

/**
 * TTS客户端示例
 * 仅实现文本转语音功能
 */
int main(int argc, char const *argv[]) {
  // 检查命令行参数
  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " [NetWorkInterface(eth0)] [option|text] [language]" << std::endl;
    std::cout << "  NetWorkInterface: 网络接口名称，如 eth0" << std::endl;
    std::cout << "  option: (可选) 预设选项 1/2/3，或自定义文本" << std::endl;
    std::cout << "    1 - 指令接收，正在前往冰箱寻找牛奶" << std::endl;
    std::cout << "    2 - 指令接收，正在前往冰箱寻找果汁" << std::endl;
    std::cout << "    3 - 指令接收，正在前往冰箱寻找汽水" << std::endl;
    std::cout << "  text: (可选) 自定义文本，如果option不是1/2/3则作为文本使用" << std::endl;
    std::cout << "  language: (可选) 语言类型，0=中文，1=英文，2=日语，默认为0" << std::endl;
    std::cout << std::endl;
    std::cout << "Examples:" << std::endl;
    std::cout << "  " << argv[0] << " eth0 1        # 使用预设选项1" << std::endl;
    std::cout << "  " << argv[0] << " eth0 2        # 使用预设选项2" << std::endl;
    std::cout << "  " << argv[0] << " eth0 3        # 使用预设选项3" << std::endl;
    std::cout << "  " << argv[0] << " eth0 \"你好，世界\"  # 自定义文本" << std::endl;
    std::cout << "  " << argv[0] << " eth0 1 2        # 使用预设选项1，日语" << std::endl;
    exit(0);
  }

  int32_t ret;
  /*
   * Initilaize ChannelFactory
   */
  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
  unitree::robot::g1::AudioClient client;
  client.Init();
  client.SetTimeout(10.0f);

  /*ASR message Example - 按照原始代码，先初始化ASR订阅 */
  unitree::robot::ChannelSubscriber<std_msgs::msg::dds_::String_> subscriber(
      AUDIO_SUBSCRIBE_TOPIC);
  subscriber.InitChannel([](const void *msg) {
    // 空回调，只是为了初始化订阅
  });

  /*Volume Example - 严格按照原始代码格式 */
  uint8_t volume;
  ret = client.GetVolume(volume);
  std::cout << "GetVolume API ret:" << ret
            << "  volume = " << std::to_string(volume) << std::endl;
  ret = client.SetVolume(100);
  std::cout << "SetVolume to 100% , API ret:" << ret << std::endl;

  /*TTS Example - 严格按照原始代码格式 */
  std::string text;
  int language = 0;  // 默认中文

  // 预设的三种TTS文本（中文版本）
  std::string preset_texts_cn[3] = {
    "收到主人，小优这就前往冰箱寻找牛奶",
    "收到主人，小忧这就前往冰箱寻找果汁",
    "收到主人，小优这就前往冰箱寻找汽水"
  };

  // 预设的三种TTS文本（日语版本）
  std::string preset_texts_jp[3] = {
    "承知いたしました、主人様。冷蔵庫へ行って牛乳を探します",
    "承知いたしました、主人様。冷蔵庫へ行ってジュースを探します",
    "承知いたしました、主人様。冷蔵庫へ行って炭酸飲料を探します"
  };

  // 如果提供了参数，判断是预设选项还是自定义文本
  if (argc >= 3) {
    std::string arg = argv[2];
    
    // 检查是否有语言参数（先检查，因为需要根据语言选择文本）
    if (argc >= 4) {
      language = std::stoi(argv[3]);
    }
    
    // 检查是否是预设选项 1/2/3
    if (arg == "1" || arg == "2" || arg == "3") {
      int option = std::stoi(arg);
      // 根据语言选择对应的预设文本
      if (language == 2) {
        // 日语版本
        text = preset_texts_jp[option - 1];
      } else {
        // 中文版本（默认）
        text = preset_texts_cn[option - 1];
      }
    } else {
      // 不是预设选项，作为自定义文本使用
      text = arg;
    }

    // 严格按照原始代码的调用方式
    ret = client.TtsMaker(text, language);
    std::cout << "TtsMaker API ret:" << ret << std::endl;
    
    // 根据文本长度和语言估算等待时间
    int wait_time = 5;  // 默认5秒
    if (language == 1) {
      wait_time = 8;  // 英文稍长
    } else if (language == 2) {
      wait_time = 6;  // 日语中等长度
    }
    // 根据文本长度调整（粗略估算：中文约2字/秒，英文约3词/秒，日文约2字/秒）
    wait_time = 5;
    
    std::cout << "Waiting " << wait_time << " seconds for audio playback..." << std::endl;
    unitree::common::Sleep(wait_time);
  } else {
    // 没有提供参数，显示使用说明
    std::cout << "No option or text provided. Please specify:" << std::endl;
    std::cout << "  1 - 指令接收，正在前往冰箱寻找牛奶" << std::endl;
    std::cout << "  2 - 指令接收，正在前往冰箱寻找果汁" << std::endl;
    std::cout << "  3 - 指令接收，正在前往冰箱寻找汽水" << std::endl;
    std::cout << "  Or provide custom text." << std::endl;
  }

  std::cout << std::endl << "TTS Client finished." << std::endl;
  return 0;
}

