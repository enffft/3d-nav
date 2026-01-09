## 📌 项目介绍

本项目是一个 **基于 ROS 的机器狗 3D 导航仿真系统**，面向 **跨楼层三维环境**，集成了  
**3D 感知、全局路径规划、局部运动控制与强化学习控制器**，并在 Gazebo 中完成端到端验证。

---
## 导航框架
### 整体架构概述
本项目的导航框架 **基于 ROS `move_base` 架构进行改造**，核心思路如下：
- 使用 **3D LiDAR + FAST-LIO** 构建稠密点云地图并提供里程计
- 通过 `pluginlib` 接口将**PCT-Planner**注册为 `move_base` 的 **GlobalPlanner**  进行 3D 全局路径规划
- 将 全局路径交由 2D 局部规划器DWA进行时序跟踪
- 控制层由 **强化学习策略** 输出底层运动指令
---
> ⚠️ DWA 本身是 **2D 局部规划器**，  
> 本项目并 **没有将 DWA 拓展为完整 3D 规划器**，而是由于DWA局部规划器会对全局路径进行时序跟踪故在此也可以用于3d导航。

以下是改造后的move_base框架

## 控制模块
控制模块使用了[rl_sar](https://github.com/fan-ziqi/rl_sar?tab=readme-ov-file)的工作，该项目实现了gazebo环境下的强化学习策略的sim-to-sim，用其作为四足机器人控制器提供跨楼层的运动能力，Lite3使用的是自己训练的策略，效果不是很好，故下文和演示都是使用自带的Unitree a1进行的。

该控制器使用：

以上开源项目github汇总：
- [rl_sar](https://github.com/fan-ziqi/rl_sar?tab=readme-ov-file)
- [FAST_LIO](https://github.com/hku-mars/FAST_LIO)
- [PCT-planner](https://github.com/byangw/PCT_planner)

---
## 🚀 3D 导航示例

<div align="center">
  <img src="src/image/3d.gif" width="800"/> 
</div>
---

## 📦 下载与依赖

### ROS 导航相关
```
sudo apt update

sudo apt install -y ros-noetic-navigation
sudo apt install -y ros-noetic-teb-local-planner
sudo apt install -y ros-noetic-pointcloud-to-laserscan
```
### 规划器PCT-Planner
#### Environment
- Ubuntu >= 20.04
- ROS >= Noetic with ros-desktop-full installation
- CUDA >= 11.7
#### Python
- Python >= 3.8
- [CuPy](https://docs.cupy.dev/en/stable/install.html) with CUDA >= 11.7
- Open3d
### 状态估计FAST-LIO
⚠️ **建议单独工作空间编译**

### 控制rl_sar
需要的依赖为：
```
sudo apt install cmake g++ build-essential libyaml-cpp-dev libeigen3-dev libboost-all-dev libspdlog-dev libfmt-dev libtbb-dev liblcm-dev
```

```
# ros-noetic (Ubuntu20.04)

sudo apt install ros-noetic-teleop-twist-keyboard ros-noetic-controller-interface ros-noetic-gazebo-ros-control ros-noetic-joint-state-controller ros-noetic-effort-controllers ros-noetic-joint-trajectory-controller ros-noetic-joy ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-controller-manager
```
---

## 🛠 安装步骤

### 1️⃣ 配置 FAST-LIO
1. 安装[Livox-SDK](https://github.com/Livox-SDK/Livox-SDK)
2. **新建工作空间**安装[livox-ros-driver](https://github.com/Livox-SDK/livox_ros_driver?tab=readme-ov-file)
3. 安装完`livox-ros-driver`后`source $Livox_ros_driver_dir$/devel/setup.bash`
4. 安装FAST-LIO
```
cd ~/$A_ROS_DIR$/src
git clone https://github.com/hku-mars/FAST_LIO.git
cd FAST_LIO
git submodule update --init
cd ../..
catkin_make
source devel/setup.bash
```
5. 修改`src/FAST-LIO/config/velodyne.yaml`,雷达话题改为：`/velodyne_points_with_time`
6. 修改`src/FAST_LIO/src/laserMapping.cpp`，将其中的所有`camera_init`坐标系改为`odom`，将所有的`body`坐标系改为`trunk`
7. 重新编译
---
### 2️⃣ 克隆项目
```
git clone https://github.com/enffft/3d-nav.git
```
### 3️⃣ 安装 PCT-Planner

```
cd /home/yifeiy/rl_sar/src/pct_planner/PCT_planner-main/planner
./build_thirdparty.sh
./build.sh
```
---
### 4️⃣ 编译主工程

```
cd ~/3d-nav 
catkin_make source 
devel/setup.bash
```
---
## ▶️ 使用说明

### 1. 启动 Gazebo 仿真环境和控制器
```
roslaunch rl_sar gazebo.launch rname:=a2
# 打开新终端，运行
rosrun rl_sar rl_sim
```
### 2. 使用PCT-planner加载点云地图
```
cd ~/3d-nav/src/pct_planner/PCT_planner-main/tomography/scripts
python3 tomography.py --scene Second
```
### 3.启动FAST_LIO
```
roslaunch fast_lio mapping_velodyne.launch
```
键盘按‘0’使得机器人进行初始程序姿态移动到 `base.yaml` 中定义的 `default_dof_pos`
按‘1’使得机器人进入基本运动模式
按‘N’使得机器人进入navigation模式，开始接受`movebase`发送的`cmd_vel`

---
### 4. 启动 PCT-Planner规划全局路径
```
cd ~/3d-nav/src/pct_planner/PCT_planner-main/planner/scripts/

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:home/YOUR-NAME/3d-nav/PCT_planner/planner/lib/3rdparty/gtsam-4.1.1/install/lib

#发布plan任务
python3 plan.py --scene Second
```
⚠️ **注意**：如果地图配置更改，需重新生成地图，路径规划器才会重新规划路径。

---
### 5. 启动movebase导航节点
```
roslaunch rl_sar nav.launch
```