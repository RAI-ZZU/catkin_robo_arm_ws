#include "interbotix_xs_ros_control/xs_hardware_interface_obj.h"

XSHardwareInterface::XSHardwareInterface(ros::NodeHandle& nh) : nh(nh)
{
    init();
    controller_manager.reset(new controller_manager::ControllerManager(this, nh));
    ros::Duration update_freq = ros::Duration(1.0/loop_hz);
    tmr_control_loop = nh.createTimer(update_freq, &XSHardwareInterface::update, this);
}

XSHardwareInterface::~XSHardwareInterface(){}

void XSHardwareInterface::init()
{
  std::string js_topic;
  nh.getParam("hardware_interface/loop_hz", loop_hz);
  nh.getParam("hardware_interface/group_name", group_name);
  nh.getParam("hardware_interface/gripper_name", gripper_name);
  nh.getParam("hardware_interface/joint_states_topic", js_topic);
  pub_group = nh.advertise<interbotix_xs_msgs::JointGroupCommand>("commands/joint_group", 1);
  pub_gripper = nh.advertise<interbotix_xs_msgs::JointSingleCommand>("commands/joint_single", 1);
  sub_joint_states = nh.subscribe(js_topic, 1, &XSHardwareInterface::joint_state_cb, this);
  srv_robot_info = nh.serviceClient<interbotix_xs_msgs::RobotInfo>("get_robot_info");

  // ---> 修改部位 1：增加夹爪存在性判断标志
  bool has_gripper = !gripper_name.empty();

  interbotix_xs_msgs::RobotInfo group_info_srv, gripper_info_srv;
  group_info_srv.request.cmd_type = "group";
  group_info_srv.request.name = group_name;
  srv_robot_info.waitForExistence();
  srv_robot_info.call(group_info_srv);

  // ---> 修改部位 2：只有当夹爪名字非空时，才去请求夹爪信息
  if (has_gripper) {
    gripper_info_srv.request.cmd_type = "single";
    gripper_info_srv.request.name = gripper_name;
    srv_robot_info.call(gripper_info_srv);
  }

  // ---> 修改部位 3：动态计算关节总数，避免越界崩溃
  num_joints = group_info_srv.response.num_joints + (has_gripper ? 1 : 0);
  joint_state_indices = group_info_srv.response.joint_state_indices;
  std::vector<std::string> joint_names = group_info_srv.response.joint_names;

  // ---> 修改部位 4：只有在有夹爪时，才去提取 .at(0) 的数据
  if (has_gripper) {
    joint_state_indices.push_back(gripper_info_srv.response.joint_state_indices.at(0));
    joint_names.push_back(gripper_info_srv.response.joint_names.at(0));
  }

  // Resize vectors
  joint_positions.resize(num_joints);
  joint_velocities.resize(num_joints);
  joint_efforts.resize(num_joints);
  joint_position_commands.resize(num_joints);
  joint_velocity_commands.resize(num_joints); // ---> 新增：初始化速度指令向量
  joint_commands_prev.resize(num_joints);

  ros::Rate loop_rate(loop_hz);
  while (joint_states.position.size() == 0 && ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  // Initialize the joint_position_commands vector to the current joint states
  for (size_t i{0}; i < num_joints; i++)
  {
    joint_position_commands.at(i) = joint_states.position.at(joint_state_indices.at(i));
    joint_commands_prev.at(i) = joint_position_commands.at(i);
  }

  // ---> 修改部位 5：安全调整 prev 数组的大小，并隔离夹爪逻辑
  size_t num_group_joints = has_gripper ? (num_joints - 1) : num_joints;
  joint_commands_prev.resize(num_group_joints);

  if (has_gripper) {
    gripper_cmd_prev = joint_states.position.at(joint_state_indices.back()) * 2;
  }

  urdf::Model model;
  std::string robot_name = nh.getNamespace();
  urdf::JointConstSharedPtr ptr;
  model.initParam(robot_name + "/robot_description");

  // Initialize Controller
  for (int i = 0; i < num_joints; ++i) {
     // Create joint state interface
    hardware_interface::JointStateHandle jointStateHandle(joint_names.at(i), &joint_positions.at(i), &joint_velocities.at(i), &joint_efforts.at(i));
    joint_state_interface.registerHandle(jointStateHandle);

    hardware_interface::JointHandle jointVelocityHandle(jointStateHandle, &joint_velocity_commands.at(i));
    velocity_joint_interface.registerHandle(jointVelocityHandle);

    joint_limits_interface::JointLimits limits;
    ptr = model.getJoint(joint_names.at(i));
    getJointLimits(ptr, limits);
    getJointLimits(joint_names.at(i), nh, limits);

    // Create position joint interface
    hardware_interface::JointHandle jointPositionHandle(jointStateHandle, &joint_position_commands.at(i));
    joint_limits_interface::PositionJointSaturationHandle jointPositionSaturationHandle(jointPositionHandle, limits);
    position_joint_saturation_interface.registerHandle(jointPositionSaturationHandle);
    position_joint_interface.registerHandle(jointPositionHandle);
  }

  registerInterface(&joint_state_interface);
  registerInterface(&position_joint_interface);
  registerInterface(&position_joint_saturation_interface);
  registerInterface(&velocity_joint_interface); // ---> 关键：这行不加，速度控制器就加载不出来
}

// ---> 新增：实现 doSwitch 监听控制器切换
void XSHardwareInterface::doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                                   const std::list<hardware_interface::ControllerInfo>& stop_list)
{
  for (const auto& controller : start_list)
  {
    for (const auto& resource : controller.claimed_resources)
    {
      if (resource.hardware_interface == "hardware_interface::VelocityJointInterface")
      {
        current_control_mode_ = "velocity";
        ROS_INFO("[xs_hardware_interface] Switched to VELOCITY command mode.");
      }
      else if (resource.hardware_interface == "hardware_interface::PositionJointInterface")
      {
        current_control_mode_ = "position";
        ROS_INFO("[xs_hardware_interface] Switched to POSITION command mode.");
      }
    }
  }
}


void XSHardwareInterface::update(const ros::TimerEvent& e)
{
    elapsed_time = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager->update(ros::Time::now(), elapsed_time);
    write(elapsed_time);
}

void XSHardwareInterface::read()
{
  for (int i = 0; i < num_joints; i++)
  {
    joint_positions.at(i) = joint_states.position.at(joint_state_indices.at(i));
    joint_velocities.at(i) = joint_states.velocity.at(joint_state_indices.at(i));
    joint_efforts.at(i) = joint_states.effort.at(joint_state_indices.at(i));
  }
}

void XSHardwareInterface::write(ros::Duration elapsed_time)
{
  interbotix_xs_msgs::JointGroupCommand group_msg;
  interbotix_xs_msgs::JointSingleCommand gripper_msg;

  // ---> 修改部位 6：增加判断标志，决定写回逻辑
  bool has_gripper = !gripper_name.empty();
  size_t num_group_joints = has_gripper ? (num_joints - 1) : num_joints;  

  group_msg.name = group_name;
  if (has_gripper) {
    gripper_msg.name = gripper_name;
    gripper_msg.cmd = joint_position_commands.back() * 2;
  }


  // ---> 修改部位：根据当前激活的控制器类型，选择性下发指令
  if (current_control_mode_ == "velocity") {
    for (size_t i{0}; i < num_group_joints; i++)
      group_msg.cmd.push_back(joint_velocity_commands.at(i));
  } 
  else {
    // 默认的 Position 模式
    position_joint_saturation_interface.enforceLimits(elapsed_time);
    for (size_t i{0}; i < num_group_joints; i++)
      group_msg.cmd.push_back(joint_position_commands.at(i));
  }


  if (joint_commands_prev != group_msg.cmd)
  {
    pub_group.publish(group_msg);
    joint_commands_prev = group_msg.cmd;
  }
  // ---> 修改部位 8：只有在有夹爪时，才发布 * 2 指令
  if (has_gripper && gripper_cmd_prev != gripper_msg.cmd)
  {
    pub_gripper.publish(gripper_msg);
    gripper_cmd_prev = gripper_msg.cmd;
  }
}



void XSHardwareInterface::joint_state_cb(const sensor_msgs::JointState &msg)
{
  joint_states = msg;
}
