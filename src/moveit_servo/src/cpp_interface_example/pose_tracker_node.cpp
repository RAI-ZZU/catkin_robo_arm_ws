#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/TransformStamped.h>

#include <moveit_servo/servo.h>
#include <moveit_servo/pose_tracking.h>
#include <moveit_servo/status_codes.h>
#include <moveit_servo/make_shared_from_pool.h>

static const std::string LOGNAME = "panda_pose_tracker_node";

// 状态监听器保持不变，用于打印底层警告（如接近奇点、碰撞）
class StatusMonitor
{
public:
  StatusMonitor(ros::NodeHandle& nh, const std::string& topic)
  {
    sub_ = nh.subscribe(topic, 1, &StatusMonitor::statusCB, this);
  }
private:
  void statusCB(const std_msgs::Int8ConstPtr& msg)
  {
    moveit_servo::StatusCode latest_status = static_cast<moveit_servo::StatusCode>(msg->data);
    if (latest_status != status_)
    {
      status_ = latest_status;
      const auto& status_str = moveit_servo::SERVO_STATUS_CODE_MAP.at(status_);
      ROS_INFO_STREAM_NAMED(LOGNAME, "Servo status: " << status_str);
    }
  }
  moveit_servo::StatusCode status_ = moveit_servo::StatusCode::INVALID;
  ros::Subscriber sub_;
};

int main(int argc, char** argv)
{
  // 1. 初始化 ROS 节点
  ros::init(argc, argv, "pose_tracker");
  ros::NodeHandle nh("~"); // 使用私有命名空间，追踪话题将变为 /pose_tracker/target_pose
  ros::AsyncSpinner spinner(8);
  spinner.start();

  // 2. 加载规划场景监控器 (用于实时碰撞检测，核心护城河)
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
  planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
  if (!planning_scene_monitor->getPlanningScene())
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Error in setting up the PlanningSceneMonitor.");
    exit(EXIT_FAILURE);
  }

  planning_scene_monitor->startSceneMonitor();
  planning_scene_monitor->startWorldGeometryMonitor(
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
      false /* skip octomap monitor */);
  planning_scene_monitor->startStateMonitor();

  // 3. 实例化 Pose Tracker (它会自动订阅 nh 下的 "target_pose" 话题)
  moveit_servo::PoseTracking tracker(nh, planning_scene_monitor);

  // 监控 Servo 状态
  StatusMonitor status_monitor(nh, "status");

  // 设置追踪的容差 (到达这个误差范围内就算追踪成功并停止微调)
  Eigen::Vector3d lin_tol{ 0.005, 0.005, 0.005 }; // 5mm 位置容差
  double rot_tol = 0.05;                          // 弧度姿态容差

  // 清除任何历史残留的目标位姿
  tracker.resetTargetPose();
  ROS_INFO_STREAM_NAMED(LOGNAME, "Pose Tracker Server is ready! Listening to: " << nh.getNamespace() << "/target_pose");

  // 4. 核心守护循环：永远准备好追踪新目标
  while (ros::ok())
  {
    // moveToPose 是阻塞的，它会尝试向目标移动。
    // 如果达到了容差，或者 0.1秒 内没有收到新的目标更新，它会退出并进行下一次 while 循环。
    // 这完美契合了持续监听的 Daemon 需求。
    tracker.moveToPose(lin_tol, rot_tol, 0.1 /* target pose timeout */);
  }

  // 节点被干掉时，确保停止运动
  tracker.stopMotion();

  return EXIT_SUCCESS;
}