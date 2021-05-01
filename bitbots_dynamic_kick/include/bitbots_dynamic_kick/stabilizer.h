#ifndef BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_STABILIZER_H_
#define BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_STABILIZER_H_

#include <optional>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_listener.h>
#include <control_toolbox/pid.h>
#include <bitbots_splines/abstract_stabilizer.h>
#include <rot_conv/rot_conv.h>
#include <sensor_msgs/Imu.h>
#include "kick_utils.h"
#include "visualizer.h"

namespace bitbots_dynamic_kick {

class Stabilizer :
    public bitbots_splines::AbstractStabilizer<KickPositions> {
 public:
  Stabilizer();

  geometry_msgs::Point cop_left;
  geometry_msgs::Point cop_right;
  sensor_msgs::Imu imu_;

  /**
   * Calculate required IK goals to reach foot_goal with a foot while keeping the robot as stable as possible.
   * @param positions a description of the required positions
   * @return BioIK Options that can be used by an instance of AbstractIK
   */

  KickPositions stabilize(const KickPositions &positions, const ros::Duration &dt) override;
  void reset() override;
  void useCop(bool use);
  void setRobotModel(moveit::core::RobotModelPtr model);
  /*
   * Whether the robot is currently stable, according to IMU and stable_threshold
   */
  bool isStable() const;
  /*
   * Differences between imu rotation and goal rotation below this threshold are considered stable
   */
  void setStableThreshold(double threshold);
  /*
   * Whether to use final stabilizing with the IMU
   */
  void setUseFinalStabilizing(bool use_final_stabilizing);
 private:
  moveit::core::RobotModelPtr kinematic_model_;
  control_toolbox::Pid pid_x_;
  control_toolbox::Pid pid_y_;
  control_toolbox::Pid pid_pitch_;
  control_toolbox::Pid pid_roll_;

  bool use_cop_;
  bool use_final_stabilizing_;
  double stable_threshold_;
  bool is_stable_;
};
}

#endif  //BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_STABILIZER_H_
