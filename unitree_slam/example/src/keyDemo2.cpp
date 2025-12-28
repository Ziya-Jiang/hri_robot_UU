#include <unitree/robot/client/client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/ros2/String_.hpp>
#include <json.hpp>
#include <termio.h>
#include <string>
#include <future>
#include <thread>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <filesystem>

#define SlamInfoTopic "rt/slam_info"
#define SlamKeyInfoTopic "rt/slam_key_info"
#define POSE_SAVE_FILE "saved_poses.json"
using namespace unitree::robot;
using namespace unitree::common;
unsigned char currentKey;

class poseDate
{
public:
    int id = 0;  // 目标点标记ID
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float q_x = 0.0f;
    float q_y = 0.0f;
    float q_z = 0.0f;
    float q_w = 1.0f;
    int mode = 1;
    // float speed = 0.5f;
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
        // j["data"]["speed"] = speed;
        return j.dump(4);
    }
    void printInfo() const
    {
        std::cout << "ID:" << id << " x:" << x << " y:" << y << " z:" << z << " q_x:"
                  << q_x << " q_y:" << q_y << " q_z:" << q_z << " q_w:" << q_w << std::endl;
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

    class TestClient : public Client
    {
    private:
        ChannelSubscriberPtr<std_msgs::msg::dds_::String_> subSlamInfo;
        ChannelSubscriberPtr<std_msgs::msg::dds_::String_> subSlamKeyInfo;

        void slamInfoHandler(const void *message);
        void slamKeyInfoHandler(const void *message);

        poseDate curPose;
        std::vector<poseDate> poseList;
        bool is_arrived = false;
        bool threadControl = false;
        std::future<void> futThread;
        std::promise<void> prom;
        std::thread controlThread;

    public:
        TestClient();
        ~TestClient();

        void Init();
        unsigned char keyDetection();
        unsigned char keyExecute();
        void stopNodeFun();
        void startMappingPlFun();
        void endMappingPlFun();
        void relocationPlFun();
        void taskLoopFun(std::promise<void> &prom, int targetId = -1);
        void pauseNavFun();
        void resumeNavFun();
        void taskThreadRun(int targetId = -1);
        void taskThreadStop();
        void printPoseList();
        int selectTargetPose();
        void savePoseList();
        void loadPoseList();
        void initHardcodedPoses();
        int getNextPoseId();
    };

    TestClient::TestClient() : Client(TEST_SERVICE_NAME, false)
    {
        subSlamInfo = ChannelSubscriberPtr<std_msgs::msg::dds_::String_>(new ChannelSubscriber<std_msgs::msg::dds_::String_>(SlamInfoTopic));
        subSlamInfo->InitChannel(std::bind(&unitree::robot::slam::TestClient::slamInfoHandler, this, std::placeholders::_1), 1);
        subSlamKeyInfo = ChannelSubscriberPtr<std_msgs::msg::dds_::String_>(new ChannelSubscriber<std_msgs::msg::dds_::String_>(SlamKeyInfoTopic));
        subSlamKeyInfo->InitChannel(std::bind(&unitree::robot::slam::TestClient::slamKeyInfoHandler, this, std::placeholders::_1), 1);
        
        // 初始化硬编码位姿
        initHardcodedPoses();
        
        // 加载保存的位姿列表
        loadPoseList();
        
        std::cout << "***********************  Unitree SLAM Demo ***********************\n";
        std::cout << "---------------            q    w                -----------------\n";
        std::cout << "---------------            a    s   d   f        -----------------\n";
        std::cout << "---------------            z    x                -----------------\n";
        std::cout << "------------------------------------------------------------------\n";
        std::cout << "------------------ q: Start mapping            -------------------\n";
        std::cout << "------------------ w: End mapping              -------------------\n";
        std::cout << "------------------ a: Start relocation         -------------------\n";
        std::cout << "------------------ s: Add pose to task list    -------------------\n";
        std::cout << "------------------ d: Navigate to target point -------------------\n";
        std::cout << "------------------ f: Clear task list          -------------------\n";
        std::cout << "------------------ l: List all target points   -------------------\n";
        std::cout << "------------------ z: Pause navigation         -------------------\n";
        std::cout << "------------------ x: Resume navigation        -------------------\n";
        std::cout << "---------------- Press any other key to stop SLAM ----------------\n";
        std::cout << "------------------------------------------------------------------\n";
        std::cout << "------------------------------------------------------------------\n";
        std::cout << "--------------- Press 'Ctrl + C' to exit the program -------------\n";
        std::cout << "------------------------------------------------------------------\n";
        std::cout << "------------------------------------------------------------------\n"
                  << std::endl;
    }

    TestClient::~TestClient()
    {
        savePoseList();  // 程序退出时保存位姿列表
        stopNodeFun();
    }

    void TestClient::Init()
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
void unitree::robot::slam::TestClient::taskThreadRun(int targetId)
{
    taskThreadStop();
    prom = std::promise<void>();
    futThread = prom.get_future();
    controlThread = std::thread(&unitree::robot::slam::TestClient::taskLoopFun, this, std::ref(prom), targetId);
    controlThread.detach();
}

void unitree::robot::slam::TestClient::taskLoopFun(std::promise<void> &prom, int targetId)
{
    std::string data;
    threadControl = true;
    
    if (targetId > 0)
    {
        // 导航到指定的目标点
        auto it = std::find_if(poseList.begin(), poseList.end(), 
                              [targetId](const poseDate& p) { return p.id == targetId; });
        if (it != poseList.end())
        {
            is_arrived = false;
            std::cout << "Navigating to target point ID: " << targetId << std::endl;
            int32_t statusCode = Call(ROBOT_API_ID_POSE_NAV_PL, it->toJsonStr(), data);
            std::cout << "parameter:" << it->toJsonStr() << std::endl;
            std::cout << "statusCode:" << statusCode << std::endl;
            std::cout << "data:" << data << std::endl;

            while (!is_arrived && threadControl)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }
        else
        {
            std::cout << "Target point ID " << targetId << " not found!" << std::endl;
        }
    }
    else
    {
        // 原有的遍历所有目标点的逻辑（保留作为备用）
        std::cout << "task list num:" << poseList.size() << std::endl;
        for (int i = 0; i < poseList.size(); i++)
        {
            is_arrived = false;
            int32_t statusCode = Call(ROBOT_API_ID_POSE_NAV_PL, poseList[i].toJsonStr(), data);
            std::cout << "parameter:" << poseList[i].toJsonStr() << std::endl;
            std::cout << "statusCode:" << statusCode << std::endl;
            std::cout << "data:" << data << std::endl;

            while (!is_arrived)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                if (!threadControl)
                    break;
            }

            if (i == poseList.size() - 1)
            {
                i = 0;
                std::reverse(poseList.begin(), poseList.end());
            }
            if (!threadControl)
                break;
        }
    }

    prom.set_value();
}

void unitree::robot::slam::TestClient::taskThreadStop()
{
    threadControl = false;
    if (futThread.valid())
    {
        auto status = futThread.wait_for(std::chrono::milliseconds(0));
        if (status != std::future_status::ready)
            futThread.wait();
    }
}

void unitree::robot::slam::TestClient::printPoseList()
{
    if (poseList.empty())
    {
        std::cout << "Task list is empty!" << std::endl;
        return;
    }
    std::cout << "\n========== Target Points List ==========" << std::endl;
    for (const auto& pose : poseList)
    {
        pose.printInfo();
    }
    std::cout << "========================================\n" << std::endl;
}

int unitree::robot::slam::TestClient::selectTargetPose()
{
    if (poseList.empty())
    {
        std::cout << "Task list is empty! Please add poses first." << std::endl;
        return -1;
    }
    
    printPoseList();
    std::cout << "Please enter the target point ID to navigate to: ";
    
    // 恢复终端属性以读取数字输入
    termios tms_old, tms_new;
    tcgetattr(0, &tms_old);
    tms_new = tms_old;
    tms_new.c_lflag |= (ICANON | ECHO);  // 启用回显和规范模式
    tcsetattr(0, TCSANOW, &tms_new);
    
    int targetId;
    std::cin >> targetId;
    
    // 恢复原始终端属性
    tcsetattr(0, TCSANOW, &tms_old);
    
    // 验证ID是否存在
    auto it = std::find_if(poseList.begin(), poseList.end(), 
                          [targetId](const poseDate& p) { return p.id == targetId; });
    if (it == poseList.end())
    {
        std::cout << "Invalid target point ID: " << targetId << std::endl;
        return -1;
    }
    
    return targetId;
}

void unitree::robot::slam::TestClient::slamInfoHandler(const void *message)
{
    std_msgs::msg::dds_::String_ currentMsg = *(std_msgs::msg::dds_::String_ *)message;
    nlohmann::json jsonData = nlohmann::json ::parse(currentMsg.data());

    // errorCode
    if (jsonData["errorCode"] != 0)
    {
        std::cout << "\033[33m" << jsonData["info"] << "\033[0m" << std::endl;
        return;
    }

    // pose
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

void unitree::robot::slam::TestClient::slamKeyInfoHandler(const void *message)
{
    std_msgs::msg::dds_::String_ currentMsg = *(std_msgs::msg::dds_::String_ *)message;
    nlohmann::json jsonData = nlohmann::json ::parse(currentMsg.data());

    // errorCode
    if (jsonData["errorCode"] != 0)
    {
        std::cout << "\033[33m" << jsonData["info"] << "\033[0m" << std::endl;
        return;
    }

    // task_result
    if (jsonData["type"] == "task_result")
    {
        is_arrived = jsonData["data"]["is_arrived"];
        if (is_arrived)
        {
            std::cout << "I arrived " << jsonData["data"]["targetNodeName"] << std::endl;
        }
        else
        {
            std::cout << "I not arrived " << jsonData["data"]["targetNodeName"] << "  Please help me!!  (T_T)   (T_T)   (T_T) " << std::endl;
        }
    }
}

void unitree::robot::slam::TestClient::stopNodeFun()
{
    std::string parameter, data;
    parameter = R"({"data": {}})"; // Fixed data content
    int32_t statusCode = Call(ROBOT_API_ID_STOP_NODE, parameter, data);
    std::cout << "statusCode:" << statusCode << std::endl;
    std::cout << "data:" << data << std::endl;
}

void unitree::robot::slam::TestClient::startMappingPlFun()
{
    std::string parameter, data;
    parameter = R"({"data": {"slam_type": "indoor"}})"; // Fixed data content
    int32_t statusCode = Call(ROBOT_API_ID_START_MAPPING_PL, parameter, data);
    std::cout << "statusCode:" << statusCode << std::endl;
    std::cout << "data:" << data << std::endl;
}

void unitree::robot::slam::TestClient::endMappingPlFun()
{
    std::string parameter, data;
    parameter = R"({"data": {"address": "/home/unitree/test.pcd"}})"; // address:pcd file save address
    int32_t statusCode = Call(ROBOT_API_ID_END_MAPPING_PL, parameter, data);
    std::cout << "statusCode:" << statusCode << std::endl;
    std::cout << "data:" << data << std::endl;
}

void unitree::robot::slam::TestClient::relocationPlFun()
{
    std::string parameter, data;
    parameter =
        R"({
        "data": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "q_x": 0.0,
            "q_y": 0.0,
            "q_z": 0.0,
            "q_w": 1.0,
            "address": "/home/unitree/test.pcd"
        }
        })"; // x/y/z/q_x/q_y/q_z/q_w:Initialize pose information    address:pcd file reading address
    int32_t statusCode = Call(ROBOT_API_ID_START_RELOCATION_PL, parameter, data);
    std::cout << "statusCode:" << statusCode << std::endl;
    std::cout << "data:" << data << std::endl;
}

void unitree::robot::slam::TestClient::pauseNavFun()
{
    std::string parameter, data;
    parameter = R"({"data": {}})"; // Fixed data content
    int32_t statusCode = Call(ROBOT_API_ID_PAUSE_NAV, parameter, data);
    std::cout << "statusCode:" << statusCode << std::endl;
    std::cout << "data:" << data << std::endl;
}

void unitree::robot::slam::TestClient::resumeNavFun()
{
    std::string parameter, data;
    parameter = R"({"data": {}})"; // Fixed data content
    int32_t statusCode = Call(ROBOT_API_ID_RESUME_NAV, parameter, data);
    std::cout << "statusCode:" << statusCode << std::endl;
    std::cout << "data:" << data << std::endl;
}

unsigned char unitree::robot::slam::TestClient::keyDetection()
{
    termios tms_old, tms_new;
    tcgetattr(0, &tms_old);
    tms_new = tms_old;
    tms_new.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(0, TCSANOW, &tms_new);
    unsigned char ch = getchar();
    tcsetattr(0, TCSANOW, &tms_old);
    std::cout << "\033[1;32m"
              << "Key " << ch << " pressed."
              << "\033[0m" << std::endl;
    return ch;
}

unsigned char unitree::robot::slam::TestClient::keyExecute()
{
    unsigned char currentKey;
    while (true)
    {
        currentKey = keyDetection();
        switch (currentKey)
        {
        case 'q':
            startMappingPlFun();
            break;
        case 'w':
            endMappingPlFun();
            break;
        case 'a':
            relocationPlFun();
            break;
        case 's':
        {
            curPose.id = poseList.size() + 1;  // 自动分配递增的ID
            poseList.push_back(curPose);
            std::cout << "Added target point with ID: " << curPose.id << std::endl;
            curPose.printInfo();
            break;
        }
        case 'd':
        {
            int targetId = selectTargetPose();
            if (targetId > 0)
            {
                taskThreadRun(targetId);
            }
            break;
        }
        case 'f':
            poseList.clear();
            std::cout << "Clear task list" << std::endl;
            break;
        case 'l':  // 添加一个快捷键来显示目标点列表
            printPoseList();
            break;
        case 'z':
            pauseNavFun();
            break;
        case 'x':
            resumeNavFun();
            break;
        default:
            taskThreadStop();
            stopNodeFun();
            break;
        }
    }
}

int main(int argc, const char **argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        exit(-1);
    }
    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]); // argv[1]：The name of the network card with network segment 123
    unitree::robot::slam::TestClient tc;

    tc.Init();
    tc.SetTimeout(10.0f);

    tc.keyExecute();
    return 0;
}
