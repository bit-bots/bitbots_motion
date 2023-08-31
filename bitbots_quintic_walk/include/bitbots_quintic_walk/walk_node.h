/*
This code is based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#ifndef BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_NODE_H_
#define BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_NODE_H_

#include <iostream>
#include <string>
#include <Eigen/Dense>
#include <chrono>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/char.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include "humanoid_league_msgs/msg/robot_control_state.hpp"
#include "bitbots_msgs/msg/joint_command.hpp"
#include "bitbots_msgs/msg/foot_pressure.hpp"
#include "biped_interfaces/msg/phase.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>

#include "bitbots_quintic_walk/walk_engine.h"
#include "bitbots_quintic_walk/walk_stabilizer.h"
#include "bitbots_quintic_walk/walk_ik.h"
#include "bitbots_splines/abstract_ik.h"
#include "bitbots_quintic_walk/walk_visualizer.h"
#include <control_toolbox/pid_ros.hpp>
#include "bitbots_quintic_walk_parameters.hpp"

namespace bitbots_quintic_walk {

class WalkNode : public rclcpp::Node {
 public:
  explicit WalkNode(std::string ns = "", std::vector<rclcpp::Parameter> parameters = {});
  bitbots_msgs::msg::JointCommand step(double dt);
  bitbots_msgs::msg::JointCommand step(double dt,
                                       geometry_msgs::msg::Twist::SharedPtr cmdvel_msg,
                                       sensor_msgs::msg::Imu::SharedPtr imu_msg,
                                       sensor_msgs::msg::JointState::SharedPtr jointstate_msg,
                                       bitbots_msgs::msg::FootPressure::SharedPtr pressure_left,
                                       bitbots_msgs::msg::FootPressure::SharedPtr pressure_right);
  bitbots_msgs::msg::JointCommand step_relative(double dt,
                                                geometry_msgs::msg::Twist::SharedPtr step_msg,
                                                sensor_msgs::msg::Imu::SharedPtr imu_msg,
                                                sensor_msgs::msg::JointState::SharedPtr jointstate_msg,
                                                bitbots_msgs::msg::FootPressure::SharedPtr pressure_left,
                                                bitbots_msgs::msg::FootPressure::SharedPtr pressure_right);
  geometry_msgs::msg::PoseArray step_open_loop(double dt, geometry_msgs::msg::Twist::SharedPtr cmdvel_msg);

  /**
   * Small helper method to get foot position via python wrapper
   */
  geometry_msgs::msg::Pose get_right_foot_pose();
  geometry_msgs::msg::Pose get_left_foot_pose();

  /**
   * Reset everything to initial idle state.
   */
  void reset();

  /**
   * Reset walk to any given state. Necessary for using this as reference in learning.
   */
  void reset(WalkState state, double phase, geometry_msgs::msg::Twist::SharedPtr cmd_vel, bool reset_odometry);

  /**
   * This is the main loop which takes care of stopping and starting of the walking.
   * A small state machine is tracking in which state the walking is and builds the trajectories accordingly.
   */
  void run();

  /**
   * Initialize internal WalkEngine to correctly zeroed, usable state
   */
  void initializeEngine();

  /**
   * Sets the current state of the robot
   * @param msg The current state
   */
  void robotStateCb(humanoid_league_msgs::msg::RobotControlState::SharedPtr msg);

  WalkEngine *getEngine();
  WalkIK *getIk();
  moveit::core::RobotModelPtr *get_kinematic_model();

  nav_msgs::msg::Odometry getOdometry();

  rcl_interfaces::msg::SetParametersResult onSetParameters(const std::vector<rclcpp::Parameter> &parameters);

  void publish_debug();
  rclcpp::TimerBase::SharedPtr startTimer();
  double getTimerFreq();

 private:
  std::vector<double> get_step_from_vel(geometry_msgs::msg::Twist::SharedPtr msg);
  void stepCb(geometry_msgs::msg::Twist::SharedPtr msg);
  void cmdVelCb(geometry_msgs::msg::Twist::SharedPtr msg);

  void imuCb(sensor_msgs::msg::Imu::SharedPtr msg);

  void checkPhaseRestAndReset();
  void pressureRightCb(bitbots_msgs::msg::FootPressure::SharedPtr msg);
  void pressureLeftCb(bitbots_msgs::msg::FootPressure::SharedPtr msg);

  void jointStateCb(sensor_msgs::msg::JointState::SharedPtr msg);

  void kickCb(std_msgs::msg::Bool::SharedPtr msg);

  OnSetParametersCallbackHandle::SharedPtr callback_handle_;

  double getTimeDelta();

  // Declare parameter listener and struct from the generate_parameter_library
  walking::ParamListener param_listener_;
  // Datastructure to hold all parameters, which is build from the schema in the 'parameters.yaml'
  walking::Params config_;

  std::string odom_frame_, base_link_frame_, l_sole_frame_, r_sole_frame_;

  WalkRequest current_request_;

  bool first_run_;

  double last_ros_update_time_;

  double pitch_error_avg_ = 0;
  double pitch_error_derivative_ = 0;
  double pitch_error_integral_ = 0;
  std::vector<std::pair<double, rclcpp::Time>> pitch_error_buffer_;

  int robot_state_;

  int current_support_foot_;

  int odom_counter_;
  WalkRequest last_request_;

  WalkResponse current_response_;
  WalkResponse current_stabilized_response_;
  bitbots_splines::JointGoals motor_goals_;

  /**
   * Saves max values we can move in a single step as [x-direction, y-direction, z-rotation].
   * Is used to limit _currentOrders to sane values
   */
  Eigen::Vector3d max_step_linear_;

  bitbots_quintic_walk::WalkEngine walk_engine_;

  nav_msgs::msg::Odometry odom_msg_;

  rclcpp::Publisher<bitbots_msgs::msg::JointCommand>::SharedPtr pub_controller_command_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_;
  rclcpp::Publisher<biped_interfaces::msg::Phase>::SharedPtr pub_support_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr step_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<humanoid_league_msgs::msg::RobotControlState>::SharedPtr robot_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr kick_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<bitbots_msgs::msg::FootPressure>::SharedPtr pressure_sub_left_;
  rclcpp::Subscription<bitbots_msgs::msg::FootPressure>::SharedPtr pressure_sub_right_;

  // MoveIt!
  std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  moveit::core::RobotModelPtr kinematic_model_;
  moveit::core::RobotStatePtr current_state_;

  WalkStabilizer stabilizer_;
  WalkIK ik_;
  WalkVisualizer visualizer_;

  double current_trunk_fused_pitch_;
  double current_trunk_fused_roll_;

  double current_fly_pressure_;
  double current_fly_effort_;

  double roll_vel_;
  double pitch_vel_;

  bool got_new_goals_;

};

} // namespace bitbots_quintic_walk

#endif // BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_NODE_H_
