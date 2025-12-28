#include <iostream>
#include <string>
#include <atomic>
#include <thread>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netdb.h>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/idl/ros2/String_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/g1/audio/g1_audio_client.hpp>

#define AUDIO_SUBSCRIBE_TOPIC "rt/audio_msg"

/**
 * 获取指定网络接口的IP地址
 */
std::string get_interface_ip(const std::string& interface_name) {
  struct ifaddrs *ifaddr, *ifa;
  char host[NI_MAXHOST];
  std::string result = "";

  if (getifaddrs(&ifaddr) == -1) {
    return "";
  }

  for (ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
    if (!ifa->ifa_addr || ifa->ifa_addr->sa_family != AF_INET) continue;
    
    // 检查接口名称是否匹配
    if (interface_name.empty() || ifa->ifa_name == interface_name) {
      int s = getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in), 
                         host, NI_MAXHOST, NULL, 0, NI_NUMERICHOST);
      if (s == 0) {
        result = std::string(host);
        if (!interface_name.empty()) {
          break;  // 找到指定接口的IP，退出
        }
      }
    }
  }
  
  freeifaddrs(ifaddr);
  return result;
}

/**
 * 检查网络接口是否存在并获取其IP地址
 */
bool check_network_interface(const std::string& interface_name, std::string& ip_address) {
  ip_address = get_interface_ip(interface_name);
  
  if (ip_address.empty()) {
    std::cout << "[Network] WARNING: Interface '" << interface_name 
              << "' not found or has no IP address!" << std::endl;
    return false;
  }
  
  std::cout << "[Network] Interface '" << interface_name 
            << "' IP address: " << ip_address << std::endl;
  
  // 检查是否在正确的网段
  if (ip_address.find("192.168.123.") == 0) {
    std::cout << "[Network] ✓ Interface is in correct subnet (192.168.123.x)" << std::endl;
    return true;
  } else {
    std::cout << "[Network] WARNING: Interface is NOT in 192.168.123.x subnet!" << std::endl;
    std::cout << "[Network] Robot typically uses 192.168.123.x subnet" << std::endl;
    return true;  // 仍然返回true，因为可能使用其他网段
  }
}

// 全局变量：AudioClient 用于TTS播报
unitree::robot::g1::AudioClient* g_audio_client = nullptr;

// 全局标志：标记TTS是否正在播放（防止TTS声音被ASR误识别）
std::atomic<bool> g_tts_playing(false);

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
  // 获取当前时间戳
  uint64_t timestamp = unitree::common::GetCurrentTimeMillisecond();
  
  if (msg == nullptr) {
    std::cout << "[" << timestamp << "] [ASR] WARNING: Received null message!" << std::endl;
    return;
  }
  
  std_msgs::msg::dds_::String_ *resMsg = (std_msgs::msg::dds_::String_ *)msg;
  std::string recognized_text = resMsg->data();
  
  // 实时打印ASR接收到的信息（带时间戳）
  std::cout << std::endl;
  std::cout << "════════════════════════════════════════" << std::endl;
  std::cout << "[" << timestamp << "] [ASR] ===== NEW MESSAGE RECEIVED =====" << std::endl;
  std::cout << "[" << timestamp << "] [ASR] Raw text: \"" << recognized_text << "\"" << std::endl;
  std::cout << "[" << timestamp << "] [ASR] Text length: " << recognized_text.length() << " characters" << std::endl;
  
  // 检测关键词
  int option = detect_keyword(recognized_text);
  std::cout << "[" << timestamp << "] [ASR] Keyword detection result: " << option;
  if (option == 1) std::cout << " (牛奶)";
  else if (option == 2) std::cout << " (果汁)";
  else if (option == 3) std::cout << " (汽水)";
  else std::cout << " (未匹配)";
  std::cout << std::endl;
  
  if (option > 0 && g_audio_client != nullptr) {
    std::cout << "[" << timestamp << "] [Detected] ✓ 识别到关键词，选项: " << option << std::endl;
    
    // 播报对应的TTS文本
    std::string tts_text = preset_texts_cn[option - 1];
    std::cout << "[" << timestamp << "] [TTS] 准备播报: \"" << tts_text << "\"" << std::endl;
    
    int32_t ret = g_audio_client->TtsMaker(tts_text, 0);  // 0=中文
    std::cout << "[" << timestamp << "] [TTS] TtsMaker API ret: " << ret << std::endl;
    
    if (ret == 0) {
      std::cout << "[" << timestamp << "] [TTS] ✓ TTS请求成功，等待播放..." << std::endl;
      // 等待播放完成
      unitree::common::Sleep(5);
      uint64_t finish_time = unitree::common::GetCurrentTimeMillisecond();
      std::cout << "[" << finish_time << "] [TTS] ✓ 播报完成" << std::endl;
    } else {
      std::cout << "[" << timestamp << "] [TTS] ✗ 播报失败，错误码: " << ret << std::endl;
    }
  } else if (option == 0) {
    std::cout << "[" << timestamp << "] [Info] 未识别到关键词（牛奶/果汁/汽水）" << std::endl;
  } else if (g_audio_client == nullptr) {
    std::cout << "[" << timestamp << "] [ERROR] AudioClient is null! Cannot play TTS." << std::endl;
  }
  std::cout << "════════════════════════════════════════" << std::endl;
  std::cout << std::endl;
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
  std::cout << std::endl;

  /* 网络接口诊断 */
  std::cout << "=== Network Interface Check ===" << std::endl;
  std::string interface_ip;
  if (!check_network_interface(argv[1], interface_ip)) {
    std::cout << "[Network] ERROR: Network interface check failed!" << std::endl;
    std::cout << "[Network] Please verify the interface name is correct." << std::endl;
    return 1;
  }
  std::cout << std::endl;

  int32_t ret;
  /*
   * Initilaize ChannelFactory
   */
  std::cout << "=== Initializing DDS ChannelFactory ===" << std::endl;
  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
  std::cout << "ChannelFactory initialized with domain_id=0, interface=" << argv[1] << std::endl;
  std::cout << std::endl;
  
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
  if (ret != 0) {
    std::cout << "ERROR: GetVolume failed! Cannot connect to robot." << std::endl;
    std::cout << "Please check:" << std::endl;
    std::cout << "  1. Robot is powered on and connected" << std::endl;
    std::cout << "  2. Network interface '" << argv[1] << "' is correct" << std::endl;
    std::cout << "  3. Robot IP is in 192.168.123.x subnet" << std::endl;
    return 1;
  }
  
  ret = client.SetVolume(100);
  std::cout << "SetVolume to 100% , API ret:" << ret << std::endl;
  if (ret != 0) {
    std::cout << "WARNING: SetVolume failed!" << std::endl;
  }

  /*
   * 订阅ASR消息主题
   * 当机器人识别到语音时，会通过此主题发送识别结果
   */
  std::cout << "=== Subscribing to ASR Topic ===" << std::endl;
  std::cout << "Topic name: \"" << AUDIO_SUBSCRIBE_TOPIC << "\"" << std::endl;
  
  unitree::robot::ChannelSubscriber<std_msgs::msg::dds_::String_> subscriber(
      AUDIO_SUBSCRIBE_TOPIC);
  
  // 先测试回调函数是否工作（创建一个测试消息）
  std::cout << "Testing callback function..." << std::endl;
  std_msgs::msg::dds_::String_ test_msg;
  test_msg.data("测试消息");
  asr_handler(&test_msg);
  std::cout << "✓ Callback function test passed." << std::endl;
  
  subscriber.InitChannel(asr_handler);
  std::cout << "✓ ASR subscription initialized successfully." << std::endl;
  
  // 等待一下让订阅有时间建立
  std::cout << "Waiting 2 seconds for subscription to establish..." << std::endl;
  unitree::common::Sleep(2);
  
  // 检查订阅状态
  int64_t last_data_time = subscriber.GetLastDataAvailableTime();
  std::cout << "Subscription status:" << std::endl;
  std::cout << "  - Channel name: " << subscriber.GetChannelName() << std::endl;
  std::cout << "  - Last data available time: " << last_data_time << " (ms since epoch)" << std::endl;
  if (last_data_time < 0) {
    std::cout << "  - Status: No data received yet (this is normal if robot hasn't sent ASR messages)" << std::endl;
  } else {
    std::cout << "  - Status: Data was received at timestamp " << last_data_time << std::endl;
  }
  std::cout << std::endl;

  std::cout << std::endl;
  std::cout << "========================================" << std::endl;
  std::cout << "Voice Command System Started!" << std::endl;
  std::cout << "Waiting for voice commands..." << std::endl;
  std::cout << "Supported commands:" << std::endl;
  std::cout << "  - 我想喝牛奶" << std::endl;
  std::cout << "  - 我想喝果汁" << std::endl;
  std::cout << "  - 我想喝汽水" << std::endl;
  std::cout << std::endl;
  std::cout << "IMPORTANT:" << std::endl;
  std::cout << "  - Make sure robot's ASR service is running" << std::endl;
  std::cout << "  - Speak clearly near the robot's microphone" << std::endl;
  std::cout << "  - Wait for robot to process and send ASR results" << std::endl;
  std::cout << std::endl;
  std::cout << "Press Ctrl+C to exit." << std::endl;
  std::cout << "========================================" << std::endl;
  std::cout << std::endl;

  /*
   * 主循环：持续运行以接收ASR消息
   */
  int heartbeat_counter = 0;
  int64_t last_received_time = -1;
  
  while (1) {
    sleep(1);  // 等待ASR消息
    
    // 检查是否有新数据
    int64_t current_data_time = subscriber.GetLastDataAvailableTime();
    if (current_data_time != last_received_time && current_data_time >= 0) {
      std::cout << "[Monitor] New data detected at timestamp: " << current_data_time << std::endl;
      last_received_time = current_data_time;
    }
    
    // 每10秒输出一次心跳，证明程序在运行
    heartbeat_counter++;
    if (heartbeat_counter >= 10) {
      uint64_t current_time = unitree::common::GetCurrentTimeMillisecond();
      std::cout << "[Heartbeat] System is running, waiting for ASR messages... (" 
                << heartbeat_counter << "s)" << std::endl;
      std::cout << "[Heartbeat] Current time: " << current_time << std::endl;
      std::cout << "[Heartbeat] Last data time: " << current_data_time;
      if (current_data_time < 0) {
        std::cout << " (no data received yet)";
      } else {
        int64_t time_diff = current_time - current_data_time;
        std::cout << " (" << time_diff << "ms ago)";
      }
      std::cout << std::endl;
      std::cout << "[Heartbeat] If you see this but no ASR messages:" << std::endl;
      std::cout << "  - Robot may not be sending ASR messages" << std::endl;
      std::cout << "  - Robot's ASR service may not be running" << std::endl;
      std::cout << "  - Network connection may have issues" << std::endl;
      std::cout << "  - Try speaking to the robot to trigger ASR" << std::endl;
      std::cout << std::endl;
      heartbeat_counter = 0;
    }
  }

  return 0;
}

