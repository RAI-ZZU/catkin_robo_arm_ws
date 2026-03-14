

# 预配置

## Panda机械臂配置

1. 设置实时内核 https://www.franka.cn/FCI/installation_linux.html

1. 设置英伟达补丁

   
   
1. 安装底层驱动0.9.2， 得安装到系统底层，否则编译ros包还要手动给定libfranka的路径

   ```bash
   git clone --recursive --branch 0.9.2  https://github.com/frankarobotics/libfranka.git
   cd libfranka/
   cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
   make -j$(nproc)
   sudo make install  
   # 检查是否成功
   ls /usr/local/lib | grep franka 
   ```

   

2. 网页激活FCI

   ![image-20260312193536930](README.assets/image-20260312193536930.png)

3. panda机械臂测试

   ```bash
   # 测试panda延迟
   sudo ping 192.168.5.245 -i 0.001 -D -c 10000 -s 1200
   # 测试panda状态通讯
   cd /libfranka/build/examples
   ./echo_robot_state  192.168.5.245
   ```

   

# ROS配置

1. 安装前置包

   ```bash
   # 速度控制器
   sudo apt install ros-noetic-velocity-controllers
   ```

   

2. 安装ROS相关包

   ```bash
   cd ~/
   git clone git@github.com:RAI-ZZU/catkin_robo_arm_ws.git
   cd catkin_robo_arm_ws/
   catkin build   --cmake-args -DCMAKE_BUILD_TYPE=Release 
   source ~/catkin_robo_arm_ws/devel/setup.bash
   ```

3. 向`~/bashrc` 添加source 路径

   ```bash
   source /opt/ros/noetic/setup.bash
   source ~/catkin_ws/devel/setup.bash
   source ~/catkin_robo_arm_ws/devel/setup.bash
   ```

## Panda ROS测试

1. 返回初始位置

   ```bash
   roslaunch franka_example_controllers  move_to_start.launch robot_ip:=192.168.5.245
   ```

2. moveit拖拽运动测试

   ```bash
   roslaunch panda_moveit_config  franka_control.launch robot_ip:=192.168.5.245
   ```

3. 伺服模式测试

   ```bash
   # 返回初始位置
   roslaunch franka_example_controllers  move_to_start.launch robot_ip:=192.168.5.245
   # 执行demo
   roslaunch moveit_servo panda_servo_system.launch
   ```

   

   











