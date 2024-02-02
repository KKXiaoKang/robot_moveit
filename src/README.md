# moveit轨迹规划
### 内容
- kuavo_moveit_config: kuavo的左手臂urdf配置
- kuavo_arm_moveit_config: kuavo的左右手urdf配置
- kuavo_urdf: kuavo的urdf源文件
- kuavo_urdf_with_hand: kuavo加上夹爪的urdf源文件
- kuavo_control: kuavo轨迹规划源码
- kuavo_arm_traj_planner: kuavo双手轨迹规划源码

### 源码架构
类和方法功能注释详尽
- base.py: 基类
- core.py: 规划核心
- scene.py: 更新场景
- optimizer.py: 轨迹优化
- axis.py: 坐标变换
- utils.py:
    - json_*: json文件处理封装
    - TransformPos: 姿态表示方式变换
    - rad_to_angle: 弧度转角度
- interface.py: 调用接口封装
- test.py: 测试用例
- kuavo_arm_traj_plan: 启动程序

### 使用手册
1. 克隆仓库
```
git clone https://www.lejuhub.com/highlydynamic/humanoidmanipulation.git ./
```
2. 安装依赖
```
确保安装了ros-noetic
```
```
sudo apt install ros-noetic-moveit
sudo apt install ros-noetic-trac-ik
pip install rospy-message-converter
```
```
cd cassie_alip_mpc-main/external_packages/casadi/
mkdir build
cd build
cmake ..
make
make install
```
```
cd nlohmann_json
mkdir build
cd build
cmake ..
make
make install
```
3. 编译
```
cd ~/ws_moveit
catkin build
source ~/ws_moveit/devel/setup.bash
```
4. 运行moveit规划示例程序
```
roslaunch kuavo_arm_moveit_config demo.launch
```
```
rosrun kuavo_arm_traj_planner test.py
注意：test.py中的动作都被注释了，在最下面可以打开一些注释，执行一些demo
```
5. 与drake对接示例
```
clone fandes/kuavo_merge_mutil_arms这个分支到ws_moveit/src目录下
修改ros_package/src/main.cc中的joint2CommandDesiredCallback函数为
```
```
// 处理收到的消息
ROS_INFO("Received joint_command_desired l_arm: [%f, %f, %f, %f, %f, %f, %f...]\n", \
    msg->position[0], msg->position[1], msg->position[2], msg->position[3], msg->position[4], msg->position[5], msg->position[6]);
ROS_INFO("Received joint_command_desired r_arm: [%f, %f, %f, %f, %f, %f, %f...]\n", \
    msg->position[7], msg->position[8], msg->position[9], msg->position[10], msg->position[11], msg->position[12], msg->position[13]);

// 长度为 14
// 检查 msg 的维度是否符合要求
if (msg->position.size() == 14)
{
    // 将 std::vector<double> 转换为 Eigen::VectorXd
    Eigen::VectorXd targetRosPosition(14);
    
    // 1-7为左手 8-14为右手
    for (int i = 0; i < msg->position.size(); i++)
    {
        targetRosPosition[i] = msg->position[i];
    }

    // 调用 setROSArmPose
    robot_ptr->rosSetMoveitMotr(targetRosPosition);
}
else
{
    ROS_WARN("Invalid arm_command_desired data. Expected 14 elements, but received %lu elements.", msg->position.size());
}
```
```
cd ~/ws_moveit/
catkin build
source ./devel/setup.bash
```
```
export ROBOT_VERSION=33
```
```
roslaunch kuavo_arm_moveit_config demo.launch
drake-visualizer
rosrun dynamic_biped highlyDynamicRobot_node
```
```
等待roslaunch kuavo_arm_moveit_config demo.launch终端出现   You can start planning now!    绿色字体后
rosrun kuavo_arm_traj_planner bainian.py
```

