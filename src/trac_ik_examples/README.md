# Usage
```
roslaunch kuavo_with_finger display.launch
roslaunch trac_ik_examples arm_traj.launch  # In arm_traj.cpp -> kuavo_flag = false; /data决定路径;
```

Real robot: kuavo/build
1. 打开3个终端

1）第一个终端运行
```
roscore
```
2）第二个终端运行，注意，需要按下o站立键，再按v 订阅/kuavo_arm_traj话题才能控制手臂运行；
```
sudo su  # 进入root
rosrun dynamic_biped highlyDynamicRobot_node --real
```
3）第三个终端运行
```
roslaunch trac_ik_examples arm_traj.launch
```

## 注意事项：
1. 零点配置文件在
```
~/.config/lejuconfig/offset.csv 中，注意行走的零位和urdf定义零位不同，已在3.5代NUC上修正(手臂竖直为0)。
```
2. 注意肘关节限位，确定机器人肘关节和urdf中的关节转动角度一致，如-90°肘关节
3. loop_rate()为话题发送频率，与关节运动速度有关；
4. 

# dependence
install nlohmann_json
```
git clone https://gitee.com/yejiqin/nlohmann_json.git
cd nlohmann_json
mkdir build
cd build/
cmake ../
make
sudo make install
```