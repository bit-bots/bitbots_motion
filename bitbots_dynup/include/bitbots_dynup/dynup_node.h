#ifndef BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_NODE_H_
#define BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_NODE_H_

#include <cmath>
#include <string>
#include <optional>
#include <ros/ros.h>
#include <ros/console.h>

#include <dynamic_reconfigure/server.h>
#include <bitbots_dynup/DynUpConfig.h>
#include <actionlib/server/simple_action_server.h>

#include <std_msgs/Char.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <bitbots_msgs/DynUpAction.h>
#include <bitbots_msgs/JointCommand.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include "bitbots_dynup/visualizer.h"
#include "bitbots_dynup/dynup_engine.h"
#include "bitbots_dynup/dynup_ik.h"
#include "bitbots_dynup/dynup_stabilizer.h"

namespace bitbots_dynup {

typedef actionlib::SimpleActionServer<bitbots_msgs::DynUpAction> ActionServer;

/**
 * DynUpNode is that part of bitbots_dynamic_DynUp which takes care of interacting with ROS and utilizes a DynUpEngine
 * to calculate actual DynUp behavior.
 *
 * It provides an ActionServer for the bitbots_msgs::DynUpAction.
 * This actionServer accepts new goals in any tf frame, and sets up the DynUpEngines to work towards this new goal
 *
 * Additionally it publishes the DynUpEngines motor-goals back into ROS
 */
class DynUpNode {
 public:
  DynUpNode();

  /** Callback for dynamic reconfigure */
  void reconfigureCallback(bitbots_dynup::DynUpConfig &config, uint32_t level);

  /**
   * Callback that gets executed whenever #m_server receives a new goal.
   * @param goal New goal to process
   */
  void executeCb(const bitbots_msgs::DynUpGoalConstPtr &goal);

  void imuCallback(const sensor_msgs::Imu &msg);

  void jointStateCallback(const sensor_msgs::JointState &jointstates);

 private:
  ros::Publisher debug_publisher_;
  ros::NodeHandle node_handle_;
  ros::Publisher joint_goal_publisher_;
  ros::Publisher support_foot_publisher_;
  ros::Subscriber cop_subscriber_;
  ros::Subscriber joint_state_subscriber_;

  ActionServer server_;
  DynupEngine engine_;
  Stabilizer stabilizer_;
  Visualizer visualizer_;
  DynupIK ik_;
  DynUpConfig params_;
  int stable_duration_;
  int engine_rate_;
  double last_ros_update_time_;
  double start_time_;
  bool debug_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener listener_;
  robot_model_loader::RobotModelLoader robot_model_loader_;

  std::string base_link_frame_, l_sole_frame_, r_sole_frame_, l_wrist_frame_, r_wrist_frame_;

  /**
   * Do main loop in which DynUpEngine::tick() gets called repeatedly.
   * The ActionServer's state is taken into account meaning that a cancelled goal no longer gets processed.
   */
  void loopEngine(ros::Rate loop_rate);

  /**
   * Retrieve current positions of left foot and trunk relative to right foot
   *
   * @return The pair of (right foot, left foot) poses if transformation was successfull
   */
  std::optional<std::tuple<geometry_msgs::Pose,
                           geometry_msgs::Pose,
                           geometry_msgs::Pose,
                           geometry_msgs::Pose>> getCurrentPoses();

  /**
   * Publish the current support_foot so that a correct base_footprint can be calculated
   * @param is_left_dyn_up Whether the left foot is the current DynUping foot, meaning it is in the air
   */
  void publishSupportFoot(bool is_left_dyn_up);

  /**
   * Publish goals to ROS
   */
  void publishGoals(const bitbots_splines::JointGoals &goals);

  /**
   * Helper method to achieve correctly sampled rate
   */
  double getTimeDelta();

};

}

#endif  //BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_NODE_H_
