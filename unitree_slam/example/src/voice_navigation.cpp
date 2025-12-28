#include <unitree/robot/client/client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/ros2/String_.hpp>
#include <unitree/robot/g1/audio/g1_audio_client.hpp>
#include <unitree/robot/g1/arm/g1_arm_action_client.hpp>
#include <unitree/robot/g1/arm/g1_arm_action_error.hpp>
#include <json.hpp>
#include <string>
#include <future>
#include <thread>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <atomic>
#include <mutex>
#include <chrono>
#include <termio.h>

#define SlamInfoTopic "rt/slam_info"
#define SlamKeyInfoTopic "rt/slam_key_info"
#define AUDIO_SUBSCRIBE_TOPIC "rt/audio_msg"
#define DEFAULT_POSE_SAVE_FILE "saved_poses.json"
#define DEFAULT_MAP_FILE "/home/unitree/test.pcd"

using namespace unitree::robot;
using namespace unitree::common;

// 错误处理函数声明
void printArmActionError(int32_t error_code);

// 全局变量：AudioClient 用于TTS播报
unitree::robot::g1::AudioClient* g_audio_client = nullptr;

// 全局变量：ArmActionClient 用于机械臂动作
unitree::robot::g1::G1ArmActionClient* g_arm_action_client = nullptr;

// 全局标志：标记TTS是否正在播放
std::atomic<bool> g_tts_playing(false);

// 导航控制相关
std::atomic<int> g_current_target_id(0);  // 当前目标点ID
std::atomic<bool> g_navigation_active(false);  // 是否正在导航
std::atomic<bool> g_need_stop_navigation(false);  // 是否需要停止导航
std::mutex g_nav_mutex;  // 导航互斥锁

// 关键词映射：牛奶->5, 果汁->6, 汽水->7
const int KEYWORD_TARGET_MAP[4] = {0, 5, 6, 7};  // 索引0不用，1=牛奶, 2=果汁, 3=汽水

// TTS文本
std::string tts_texts[3] = {
    "收到主人，小优这就前往冰箱寻找牛奶",
    "收到主人，小忧这就前往冰箱寻找果汁",
    "收到主人，小优这就前往冰箱寻找汽水"
};

class poseDate
{
public:
    int id = 0;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float q_x = 0.0f;
    float q_y = 0.0f;
    float q_z = 0.0f;
    float q_w = 1.0f;
    int mode = 1;
    
    std::string toJsonStr() const
    {
        nlohmann::json j;
        j["data"]["targetPose"]["x"] = x;
        j["data"]["targetPose"]["y"] = y;
        j["data"]["targetPose"]["z"] = z;
        j["data"]["targetPose"]["q_x"] = q_x;
        j["data"]["targetPose"]["q_y"] = q_y;
        j["data"]["targetPose"]["q_z"] = q_z;
        j["data"]["targetPose"]["q_w"] = q_w;
        j["data"]["mode"] = mode;
        return j.dump(4);
    }
    
    nlohmann::json toJson() const
    {
        nlohmann::json j;
        j["id"] = id;
        j["x"] = x;
        j["y"] = y;
        j["z"] = z;
        j["q_x"] = q_x;
        j["q_y"] = q_y;
        j["q_z"] = q_z;
        j["q_w"] = q_w;
        j["mode"] = mode;
        return j;
    }
    
    void fromJson(const nlohmann::json& j)
    {
        id = j.value("id", 0);
        x = j.value("x", 0.0f);
        y = j.value("y", 0.0f);
        z = j.value("z", 0.0f);
        q_x = j.value("q_x", 0.0f);
        q_y = j.value("q_y", 0.0f);
        q_z = j.value("q_z", 0.0f);
        q_w = j.value("q_w", 1.0f);
        mode = j.value("mode", 1);
    }
};

namespace unitree::robot::slam
{
    const std::string TEST_SERVICE_NAME = "slam_operate";
    const std::string TEST_API_VERSION = "1.0.0.1";
    
    const int32_t ROBOT_API_ID_STOP_NODE = 1901;
    const int32_t ROBOT_API_ID_START_MAPPING_PL = 1801;
    const int32_t ROBOT_API_ID_END_MAPPING_PL = 1802;
    const int32_t ROBOT_API_ID_START_RELOCATION_PL = 1804;
    const int32_t ROBOT_API_ID_POSE_NAV_PL = 1102;
    const int32_t ROBOT_API_ID_PAUSE_NAV = 1201;
    const int32_t ROBOT_API_ID_RESUME_NAV = 1202;
    
    class VoiceNavClient : public Client
    {
    private:
        ChannelSubscriberPtr<std_msgs::msg::dds_::String_> subSlamInfo;
        ChannelSubscriberPtr<std_msgs::msg::dds_::String_> subSlamKeyInfo;
        ChannelSubscriberPtr<std_msgs::msg::dds_::String_> subAudioMsg;
        
        void slamInfoHandler(const void *message);
        void slamKeyInfoHandler(const void *message);
        void asrHandler(const void *message);
        
        poseDate curPose;
        std::vector<poseDate> poseList;
        bool is_arrived = false;
        bool threadControl = false;
        std::future<void> futThread;
        std::promise<void> prom;
        std::thread controlThread;
        
    public:
        bool relocation_success = false;
        VoiceNavClient();
        ~VoiceNavClient();
        
        void Init();
        void startRelocation();
        void navigateToTarget(int targetId);
        void navigateToTargetAndReturn(int targetId, int returnId);
        void stopNavigation();
        void taskLoopFun(std::promise<void> &prom, int targetId, int returnId);
        void taskThreadRun(int targetId, int returnId);
        void taskThreadStop();
        int detectKeyword(const std::string& text);
        void handleVoiceCommand(int keyword_option);
        void loadPoseList(const std::string& pose_file = "");
        void savePoseList(const std::string& pose_file = "");
        poseDate* findPoseById(int id);
        unsigned char keyDetection();
        void executeArmActions();  // 执行握手和放下手动作
        void setMapFile(const std::string& map_file);  // 设置地图文件路径
        void setPoseFile(const std::string& pose_file);  // 设置位姿文件路径
        
    private:
        std::string map_file_path = DEFAULT_MAP_FILE;  // 地图文件路径
        std::string pose_file_path = DEFAULT_POSE_SAVE_FILE;  // 位姿文件路径
    };
    
    VoiceNavClient::VoiceNavClient() : Client(TEST_SERVICE_NAME, false)
    {
        subSlamInfo = ChannelSubscriberPtr<std_msgs::msg::dds_::String_>(
            new ChannelSubscriber<std_msgs::msg::dds_::String_>(SlamInfoTopic));
        subSlamInfo->InitChannel(std::bind(&VoiceNavClient::slamInfoHandler, this, std::placeholders::_1), 1);
        
        subSlamKeyInfo = ChannelSubscriberPtr<std_msgs::msg::dds_::String_>(
            new ChannelSubscriber<std_msgs::msg::dds_::String_>(SlamKeyInfoTopic));
        subSlamKeyInfo->InitChannel(std::bind(&VoiceNavClient::slamKeyInfoHandler, this, std::placeholders::_1), 1);
        
        subAudioMsg = ChannelSubscriberPtr<std_msgs::msg::dds_::String_>(
            new ChannelSubscriber<std_msgs::msg::dds_::String_>(AUDIO_SUBSCRIBE_TOPIC));
        subAudioMsg->InitChannel(std::bind(&VoiceNavClient::asrHandler, this, std::placeholders::_1), 1);
    }
    
    VoiceNavClient::~VoiceNavClient()
    {
        taskThreadStop();
        savePoseList();  // 程序退出时保存位姿列表
    }
    
    void VoiceNavClient::Init()
    {
        SetApiVersion(TEST_API_VERSION);
        UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_POSE_NAV_PL);
        UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_PAUSE_NAV);
        UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_RESUME_NAV);
        UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_STOP_NODE);
        UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_START_MAPPING_PL);
        UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_END_MAPPING_PL);
        UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_START_RELOCATION_PL);
    }
}

// 临时占位函数，后续会实现
void unitree::robot::slam::VoiceNavClient::slamInfoHandler(const void *message)
{
    std_msgs::msg::dds_::String_ currentMsg = *(std_msgs::msg::dds_::String_ *)message;
    nlohmann::json jsonData = nlohmann::json::parse(currentMsg.data());
    
    if (jsonData["errorCode"] != 0)
    {
        std::cout << "\033[33m" << jsonData["info"] << "\033[0m" << std::endl;
        return;
    }
    
    if (jsonData["type"] == "pos_info")
    {
        curPose.x = jsonData["data"]["currentPose"]["x"];
        curPose.y = jsonData["data"]["currentPose"]["y"];
        curPose.z = jsonData["data"]["currentPose"]["z"];
        curPose.q_x = jsonData["data"]["currentPose"]["q_x"];
        curPose.q_y = jsonData["data"]["currentPose"]["q_y"];
        curPose.q_z = jsonData["data"]["currentPose"]["q_z"];
        curPose.q_w = jsonData["data"]["currentPose"]["q_w"];
    }
}

void unitree::robot::slam::VoiceNavClient::slamKeyInfoHandler(const void *message)
{
    std_msgs::msg::dds_::String_ currentMsg = *(std_msgs::msg::dds_::String_ *)message;
    nlohmann::json jsonData = nlohmann::json::parse(currentMsg.data());
    
    if (jsonData["errorCode"] != 0)
    {
        std::cout << "\033[33m" << jsonData["info"] << "\033[0m" << std::endl;
        return;
    }
    
    if (jsonData["type"] == "task_result")
    {
        is_arrived = jsonData["data"]["is_arrived"];
        if (is_arrived)
        {
            std::cout << "I arrived " << jsonData["data"]["targetNodeName"] << std::endl;
        }
        else
        {
            std::cout << "I not arrived " << jsonData["data"]["targetNodeName"] << std::endl;
        }
    }
}

void unitree::robot::slam::VoiceNavClient::asrHandler(const void *message)
{
    if (message == nullptr || g_audio_client == nullptr)
    {
        return;
    }
    
    std_msgs::msg::dds_::String_ *resMsg = (std_msgs::msg::dds_::String_ *)message;
    std::string recognized_text = resMsg->data();
    
    std::cout << "[ASR] Received: \"" << recognized_text << "\"" << std::endl;
    
    // 检测关键词
    int option = detectKeyword(recognized_text);
    if (option > 0)
    {
        std::cout << "[ASR] Keyword detected: option " << option << std::endl;
        handleVoiceCommand(option);
    }
}

void unitree::robot::slam::VoiceNavClient::setMapFile(const std::string& map_file)
{
    map_file_path = map_file;
    std::cout << "[Config] Map file set to: " << map_file_path << std::endl;
}

void unitree::robot::slam::VoiceNavClient::setPoseFile(const std::string& pose_file)
{
    pose_file_path = pose_file;
    std::cout << "[Config] Pose file set to: " << pose_file_path << std::endl;
}

void unitree::robot::slam::VoiceNavClient::startRelocation()
{
    std::cout << "[Relocation] Starting relocation..." << std::endl;
    std::cout << "[Relocation] Using map file: " << map_file_path << std::endl;
    
    std::string parameter, data;
    nlohmann::json param_json;
    param_json["data"]["x"] = 0.0;
    param_json["data"]["y"] = 0.0;
    param_json["data"]["z"] = 0.0;
    param_json["data"]["q_x"] = 0.0;
    param_json["data"]["q_y"] = 0.0;
    param_json["data"]["q_z"] = 0.0;
    param_json["data"]["q_w"] = 1.0;
    param_json["data"]["address"] = map_file_path;
    parameter = param_json.dump();
    
    int32_t statusCode = Call(ROBOT_API_ID_START_RELOCATION_PL, parameter, data);
    std::cout << "[Relocation] statusCode: " << statusCode << std::endl;
    std::cout << "[Relocation] data: " << data << std::endl;
    
    // 等待重定位结果
    unitree::common::Sleep(3);
    
    // 解析返回结果判断是否成功
    try
    {
        nlohmann::json result = nlohmann::json::parse(data);
        if (statusCode == 0 && result.contains("errorCode") && result["errorCode"] == 0)
        {
            relocation_success = true;
            std::cout << "[Relocation] ✓ Relocation successful!" << std::endl;
            
            // TTS播报成功
            if (g_audio_client != nullptr)
            {
                std::string success_msg = "重定位成功，开始接受语音指令";
                g_audio_client->TtsMaker(success_msg, 0);
                unitree::common::Sleep(3);
            }
        }
        else
        {
            relocation_success = false;
            std::cout << "[Relocation] ✗ Relocation failed!" << std::endl;
            
            // TTS播报失败
            if (g_audio_client != nullptr)
            {
                std::string fail_msg = "重定位失败，请检查环境";
                g_audio_client->TtsMaker(fail_msg, 0);
                unitree::common::Sleep(3);
            }
        }
    }
    catch (const std::exception& e)
    {
        relocation_success = false;
        std::cout << "[Relocation] Error parsing result: " << e.what() << std::endl;
    }
}
void unitree::robot::slam::VoiceNavClient::stopNavigation()
{
    std::lock_guard<std::mutex> lock(g_nav_mutex);
    
    std::cout << "[Navigation] Stopping navigation..." << std::endl;
    taskThreadStop();
    
    std::string parameter, data;
    parameter = R"({"data": {}})";
    int32_t statusCode = Call(ROBOT_API_ID_STOP_NODE, parameter, data);
    
    g_navigation_active = false;
    g_current_target_id = 0;
    std::cout << "[Navigation] Navigation stopped" << std::endl;
}

void unitree::robot::slam::VoiceNavClient::navigateToTarget(int targetId)
{
    poseDate* targetPose = findPoseById(targetId);
    if (targetPose == nullptr)
    {
        std::cout << "[Navigation] Target pose " << targetId << " not found!" << std::endl;
        return;
    }
    
    std::string data;
    is_arrived = false;
    g_current_target_id = targetId;
    
    std::cout << "[Navigation] Navigating to target " << targetId << std::endl;
    int32_t statusCode = Call(ROBOT_API_ID_POSE_NAV_PL, targetPose->toJsonStr(), data);
    std::cout << "[Navigation] statusCode: " << statusCode << ", data: " << data << std::endl;
    
    // 等待到达
    int timeout = 0;
    while (!is_arrived && threadControl && !g_need_stop_navigation.load())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        timeout++;
        if (timeout > 300)  // 30秒超时
        {
            std::cout << "[Navigation] Timeout waiting for arrival" << std::endl;
            break;
        }
    }
    
    // 到达目标位置后，执行机械臂动作（包括返回点1）
    if (is_arrived && !g_need_stop_navigation.load())
    {
        std::cout << "[Navigation] Arrived at target " << targetId << ", executing arm actions..." << std::endl;
        executeArmActions();
    }
}

void unitree::robot::slam::VoiceNavClient::navigateToTargetAndReturn(int targetId, int returnId)
{
    std::lock_guard<std::mutex> lock(g_nav_mutex);
    
    g_need_stop_navigation = false;
    g_navigation_active = true;
    
    std::cout << "[Navigation] Starting navigation: target " << targetId << " -> return " << returnId << std::endl;
    
    taskThreadRun(targetId, returnId);
}
void unitree::robot::slam::VoiceNavClient::taskLoopFun(std::promise<void> &prom, int targetId, int returnId)
{
    threadControl = true;
    
    // 导航到目标点
    if (!g_need_stop_navigation.load())
    {
        navigateToTarget(targetId);
    }
    
    // 如果被中断，直接返回
    if (g_need_stop_navigation.load())
    {
        std::cout << "[Navigation] Navigation interrupted" << std::endl;
        g_navigation_active = false;
        prom.set_value();
        return;
    }
    
    // 导航到返回点
    if (!g_need_stop_navigation.load())
    {
        navigateToTarget(returnId);
    }
    
    g_navigation_active = false;
    std::cout << "[Navigation] Navigation completed" << std::endl;
    prom.set_value();
}

void unitree::robot::slam::VoiceNavClient::taskThreadRun(int targetId, int returnId)
{
    taskThreadStop();
    prom = std::promise<void>();
    futThread = prom.get_future();
    controlThread = std::thread(&VoiceNavClient::taskLoopFun, this, std::ref(prom), targetId, returnId);
    controlThread.detach();
}

void unitree::robot::slam::VoiceNavClient::taskThreadStop()
{
    threadControl = false;
    if (futThread.valid())
    {
        auto status = futThread.wait_for(std::chrono::milliseconds(0));
        if (status != std::future_status::ready)
        {
            futThread.wait();
        }
    }
}
int unitree::robot::slam::VoiceNavClient::detectKeyword(const std::string& text)
{
    // 检查关键词：牛奶->1, 果汁->2, 汽水->3
    if (text.find("牛奶") != std::string::npos)
    {
        return 1;
    }
    else if (text.find("果汁") != std::string::npos)
    {
        return 2;
    }
    else if (text.find("汽水") != std::string::npos)
    {
        return 3;
    }
    return 0;  // 未匹配
}

void unitree::robot::slam::VoiceNavClient::handleVoiceCommand(int keyword_option)
{
    if (keyword_option < 1 || keyword_option > 3)
    {
        return;
    }
    
    // 如果正在导航，需要先停止
    if (g_navigation_active.load())
    {
        std::cout << "[Voice] Navigation interrupted by new command!" << std::endl;
        g_need_stop_navigation = true;
        stopNavigation();
        unitree::common::Sleep(1);  // 等待停止完成
    }
    
    // 获取目标点ID
    int targetId = KEYWORD_TARGET_MAP[keyword_option];
    int returnId = 1;  // 总是回到点1
    
    std::cout << "[Voice] Handling command: option " << keyword_option 
              << " -> target " << targetId << " -> return " << returnId << std::endl;
    
    // TTS播报
    if (g_audio_client != nullptr)
    {
        std::string tts_text = tts_texts[keyword_option - 1];
        std::cout << "[TTS] Playing: \"" << tts_text << "\"" << std::endl;
        g_tts_playing = true;
        int32_t ret = g_audio_client->TtsMaker(tts_text, 0);
        if (ret == 0)
        {
            unitree::common::Sleep(5);  // 等待TTS播放完成
        }
        g_tts_playing = false;
    }
    
    // 开始导航
    navigateToTargetAndReturn(targetId, returnId);
}
void unitree::robot::slam::VoiceNavClient::loadPoseList(const std::string& pose_file)
{
    std::string file_path = pose_file.empty() ? pose_file_path : pose_file;
    
    if (!std::filesystem::exists(file_path))
    {
        std::cout << "[Pose] No saved poses file found: " << file_path << std::endl;
        return;
    }
    
    std::ifstream file(file_path);
    if (!file.is_open())
    {
        std::cout << "[Pose] Failed to open " << file_path << std::endl;
        return;
    }
    
    try
    {
        nlohmann::json j;
        file >> j;
        file.close();
        
        if (j.contains("poses") && j["poses"].is_array())
        {
            poseList.clear();
            for (const auto& poseJson : j["poses"])
            {
                poseDate pose;
                pose.fromJson(poseJson);
                poseList.push_back(pose);
            }
            std::cout << "[Pose] Loaded " << poseList.size() << " poses from " << file_path << std::endl;
        }
    }
    catch (const std::exception& e)
    {
            std::cout << "[Pose] Error loading poses: " << e.what() << std::endl;
    }
}

void unitree::robot::slam::VoiceNavClient::savePoseList(const std::string& pose_file)
{
    std::string file_path = pose_file.empty() ? pose_file_path : pose_file;
    
    nlohmann::json j;
    j["poses"] = nlohmann::json::array();
    for (const auto& pose : poseList)
    {
        j["poses"].push_back(pose.toJson());
    }
    
    std::ofstream file(file_path);
    if (file.is_open())
    {
        file << j.dump(4);
        file.close();
        std::cout << "[Pose] Saved " << poseList.size() << " poses to " << file_path << std::endl;
    }
    else
    {
        std::cout << "[Pose] Failed to save poses to " << file_path << std::endl;
    }
}

poseDate* unitree::robot::slam::VoiceNavClient::findPoseById(int id)
{
    for (auto& pose : poseList)
    {
        if (pose.id == id)
        {
            return &pose;
        }
    }
    return nullptr;
}

unsigned char unitree::robot::slam::VoiceNavClient::keyDetection()
{
    termios tms_old, tms_new;
    tcgetattr(0, &tms_old);
    tms_new = tms_old;
    tms_new.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(0, TCSANOW, &tms_new);
    unsigned char ch = getchar();
    tcsetattr(0, TCSANOW, &tms_old);
    return ch;
}

void unitree::robot::slam::VoiceNavClient::executeArmActions()
{
    if (g_arm_action_client == nullptr)
    {
        std::cout << "[Arm] Arm action client not initialized!" << std::endl;
        return;
    }
    
    // 执行握手动作 (action id 27)
    std::cout << "[Arm] Executing handshake action (id: 27)..." << std::endl;
    int32_t ret = g_arm_action_client->ExecuteAction(27);
    if (ret != 0)
    {
        std::cout << "[Arm] Failed to execute handshake action, error code: " << ret << std::endl;
        printArmActionError(ret);
    }
    else
    {
        std::cout << "[Arm] Handshake action started, waiting 2 seconds..." << std::endl;
        unitree::common::Sleep(2);  // 等待2秒
    }
    
    // 执行放下手动作 (action id 99)
    std::cout << "[Arm] Executing put down hand action (id: 99)..." << std::endl;
    ret = g_arm_action_client->ExecuteAction(99);
    if (ret != 0)
    {
        std::cout << "[Arm] Failed to execute put down hand action, error code: " << ret << std::endl;
        printArmActionError(ret);
    }
    else
    {
        std::cout << "[Arm] Put down hand action executed" << std::endl;
        unitree::common::Sleep(1);  // 等待动作完成
    }
}

void printArmActionError(int32_t error_code)
{
    using namespace unitree::robot::g1;
    
    switch (error_code)
    {
    case UT_ROBOT_ARM_ACTION_ERR_ARMSDK:
        std::cout << "[Arm] Error: " << UT_ROBOT_ARM_ACTION_ERR_ARMSDK_DESC << std::endl;
        break;
    case UT_ROBOT_ARM_ACTION_ERR_HOLDING:
        std::cout << "[Arm] Error: " << UT_ROBOT_ARM_ACTION_ERR_HOLDING_DESC << std::endl;
        break;
    case UT_ROBOT_ARM_ACTION_ERR_INVALID_ACTION_ID:
        std::cout << "[Arm] Error: " << UT_ROBOT_ARM_ACTION_ERR_INVALID_ACTION_ID_DESC << std::endl;
        break;
    case UT_ROBOT_ARM_ACTION_ERR_INVALID_FSM_ID:
        std::cout << "[Arm] Error: Invalid FSM state!" << std::endl;
        std::cout << "[Arm] Actions are only supported in FSM id {500, 501, 801}" << std::endl;
        std::cout << "[Arm] In state 801, actions are only supported in FSM mode {0, 3}" << std::endl;
        std::cout << "[Arm] Please check robot's FSM state by subscribing to rt/sportmodestate" << std::endl;
        std::cout << "[Arm] Note: This error may occur if robot is in wrong mode. Action will be skipped." << std::endl;
        break;
    default:
        std::cout << "[Arm] Unknown error code: " << error_code << std::endl;
        break;
    }
}

int main(int argc, const char **argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " networkInterface [map_file] [pose_file]" << std::endl;
        std::cout << "  networkInterface: Network interface name (e.g., eth0)" << std::endl;
        std::cout << "  map_file: (optional) Path to PCD map file (default: /home/unitree/test.pcd)" << std::endl;
        std::cout << "  pose_file: (optional) Path to saved poses JSON file (default: saved_poses.json)" << std::endl;
        std::cout << std::endl;
        std::cout << "Examples:" << std::endl;
        std::cout << "  " << argv[0] << " eth0" << std::endl;
        std::cout << "  " << argv[0] << " eth0 /home/unitree/room1.pcd room1_poses.json" << std::endl;
        std::cout << "  " << argv[0] << " eth0 /home/unitree/room2.pcd room2_poses.json" << std::endl;
        exit(-1);
    }
    
    std::string map_file = (argc >= 3) ? argv[2] : DEFAULT_MAP_FILE;
    std::string pose_file = (argc >= 4) ? argv[3] : DEFAULT_POSE_SAVE_FILE;
    
    std::cout << "Initializing Voice Navigation System..." << std::endl;
    std::cout << "Configuration:" << std::endl;
    std::cout << "  Network Interface: " << argv[1] << std::endl;
    std::cout << "  Map File: " << map_file << std::endl;
    std::cout << "  Pose File: " << pose_file << std::endl;
    std::cout << std::endl;
    
    // 初始化ChannelFactory
    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
    
    // 初始化AudioClient
    unitree::robot::g1::AudioClient audio_client;
    audio_client.Init();
    audio_client.SetTimeout(10.0f);
    g_audio_client = &audio_client;
    
    // 初始化ArmActionClient（使用静态变量确保在整个程序运行期间有效）
    static auto arm_action_client = std::make_shared<unitree::robot::g1::G1ArmActionClient>();
    arm_action_client->Init();
    arm_action_client->SetTimeout(10.0f);
    g_arm_action_client = arm_action_client.get();
    std::cout << "[Arm] Arm action client initialized" << std::endl;
    
    // 初始化SLAM客户端
    unitree::robot::slam::VoiceNavClient client;
    client.Init();
    client.SetTimeout(10.0f);
    
    // 设置地图文件和位姿文件路径
    client.setMapFile(map_file);
    client.setPoseFile(pose_file);
    
    // 加载位姿列表
    client.loadPoseList(pose_file);
    
    std::cout << "System initialized." << std::endl;
    std::cout << std::endl;
    
    // 先自动尝试一次重定位
    std::cout << "========================================" << std::endl;
    std::cout << "Attempting automatic relocation..." << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;
    
    client.startRelocation();
    
    // 如果重定位失败，等待用户按'a'键手动重定位
    bool relocation_done = client.relocation_success;
    
    while (!relocation_done)
    {
        std::cout << std::endl;
        std::cout << "========================================" << std::endl;
        std::cout << "Relocation failed!" << std::endl;
        std::cout << "Please adjust robot position and press 'a' to retry relocation" << std::endl;
        std::cout << "========================================" << std::endl;
        std::cout << std::endl;
        
        unsigned char key = client.keyDetection();
        
        if (key == 'a' || key == 'A')
        {
            std::cout << "\033[1;32m" << "Key 'a' pressed. Starting relocation..." << "\033[0m" << std::endl;
            
            // 执行重定位
            client.startRelocation();
            
            if (client.relocation_success)
            {
                relocation_done = true;
            }
        }
        else
        {
            std::cout << "Please press 'a' to retry relocation (you pressed: " << key << ")" << std::endl;
        }
    }
    
    // 重定位成功后，显示系统就绪信息
    std::cout << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Voice Navigation System Ready!" << std::endl;
    std::cout << "Waiting for voice commands..." << std::endl;
    std::cout << "Supported commands:" << std::endl;
    std::cout << "  - 牛奶 -> Navigate to point 5 -> Return to point 1" << std::endl;
    std::cout << "  - 果汁 -> Navigate to point 6 -> Return to point 1" << std::endl;
    std::cout << "  - 汽水 -> Navigate to point 7 -> Return to point 1" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;
    
    // 重定位成功后，开始监听语音指令
    // 主循环：持续监听语音指令
    while (1)
    {
        unitree::common::Sleep(1);
    }
    
    return 0;
}

