# unitree_sdk2
Unitree robot sdk version 2.

### Prebuild environment
* OS  (Ubuntu 20.04 LTS)  
* CPU  (aarch64 and x86_64)   
* Compiler  (gcc version 9.4.0) 

### Environment Setup

Before building or running the SDK, ensure the following dependencies are installed:

- CMake (version 3.10 or higher)
- GCC (version 9.4.0)
- Make

You can install the required packages on Ubuntu 20.04 with:

```bash
apt-get update
apt-get install -y cmake g++ build-essential libyaml-cpp-dev libeigen3-dev libboost-all-dev libspdlog-dev libfmt-dev
```

### Build examples

To build the examples inside this repository:

```bash
mkdir build
cd build
cmake ..
make
```

### Installation

To build your own application with the SDK, you can install the unitree_sdk2 to your system directory:

```bash
mkdir build
cd build
cmake ..
sudo make install
```

Or install unitree_sdk2 to a specified directory:

```bash
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics
sudo make install
```

You can refer to `example/cmake_sample` on how to import the unitree_sdk2 into your CMake project. 

Note that if you install the library to other places other than `/opt/unitree_robotics`, you need to make sure the path is added to "${CMAKE_PREFIX_PATH}" so that cmake can find it with "find_package()".

### Notice
For more reference information, please go to [Unitree Document Center](https://support.unitree.com/home/zh/developer).

## 2025.12.28 开发日志

### RGB-D 序列采集
将头顶 type C 接口通过 typeC-USB 转接线（支持 USB 3.0 以上最佳）连接到电脑，即可通过 `catkin_Realsense_ws` 中的 `realsense_data_collector.py`（使用详见统一目录下的 README）进行 TUM 格式的 RGB-D 采集。

### SLAM 程序入口
`unitree_slam` 目录下的 `keyDemo eno1` 是用来建图和位姿收集的，收集好位姿后可以使用 `Voice_Navigation` 进行语音导航。任意点导航直接使用 `KeyDemo` 即可。地图你就换个命名就可以，我们没有访问权限，也不再 164 的 JNix 里面，已知位姿都在 `saved_pose.json` 里面，建议在代码里面改一个名字，`keyDemo` 和 `voice_navigation` 保持一致。
