#ifndef BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_STABILIZER_H_
#define BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_STABILIZER_H_

#include <optional>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_listener.h>
#include <control_toolbox/pid.h>
#include <bitbots_splines/abstract_stabilizer.h>
#include <bitbots_splines/abstract_ik.h>
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

  /**
   * Calculate required IK goals to reach foot_goal with a foot while keeping the robot as stable as possible.
   * @param positions a description of the required positions
   * @return BioIK Options that can be used by an instance of AbstractIK
   */

  KickPositions stabilize(const KickPositions &positions, const ros::Duration &dt) override;
  bitbots_splines::JointGoals stabilizeGoals(const bitbots_splines::JointGoals &goals,
                                             const ros::Duration &dt,
                                             const KickPositions &positions);
  void IMUCallback(const sensor_msgs::Imu &imu_msg);
  void reset() override;
  void useCop(bool use);
  void setRobotModel(moveit::core::RobotModelPtr model);
 private:
  KickPositions cartesianImuOrientation(const KickPositions &positions, const ros::Duration &dt);
  KickPositions cartesianImuOrientationFused(const KickPositions &positions, const ros::Duration &dt);
  KickPositions cartesianImuVelocity(const KickPositions &positions, const ros::Duration &dt);

  /**
   * calculate stabilizing target from center of pressure
   * the cop is in corresponding sole frame
   * optimal stabilizing would be centered above sole center
   *
   * @param positions Position goals which the stabilizer ultimately tries to reach
   * @param dt Change in time since last call
   * @returns Adjusted position goals
   */
  KickPositions cartesianHipCop(const KickPositions &positions, const ros::Duration &dt);

  bitbots_splines::JointGoals ankleImuOrientation(const bitbots_splines::JointGoals &goals,
                                                  const ros::Duration &dt,
                                                  const KickPositions &positions);
  bitbots_splines::JointGoals ankleImuOrientationFused(const bitbots_splines::JointGoals &goals,
                                                       const ros::Duration &dt,
                                                       const KickPositions &positions);
  bitbots_splines::JointGoals ankleImuVelocity(const bitbots_splines::JointGoals &goals,
                                               const ros::Duration &dt,
                                               const KickPositions &positions);
  bitbots_splines::JointGoals ankleCop(const bitbots_splines::JointGoals &goals,
                                       const ros::Duration &dt,
                                       const KickPositions &positions);

  bitbots_splines::JointGoals hipImuOrientation(const bitbots_splines::JointGoals &goals,
                                                const ros::Duration &dt,
                                                const KickPositions &positions);
  bitbots_splines::JointGoals hipImuOrientationFused(const bitbots_splines::JointGoals &goals,
                                                     const ros::Duration &dt,
                                                     const KickPositions &positions);
  bitbots_splines::JointGoals hipImuVelocity(const bitbots_splines::JointGoals &goals,
                                             const ros::Duration &dt,
                                             const KickPositions &positions);
  bitbots_splines::JointGoals hipCop(const bitbots_splines::JointGoals &goals,
                                     const ros::Duration &dt,
                                     const KickPositions &positions);

  moveit::core::RobotModelPtr kinematic_model_;
  control_toolbox::Pid pid_x_;
  control_toolbox::Pid pid_y_;
  control_toolbox::Pid pid_imu_roll_velocity_;
  control_toolbox::Pid pid_imu_pitch_velocity_;
  control_toolbox::Pid pid_imu_roll_;
  control_toolbox::Pid pid_imu_pitch_;

  bool use_cop_;

  double imu_roll_velocity_;
  double imu_pitch_velocity_;
  double imu_roll_;
  double imu_pitch_;
};
}

#endif  //BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_STABILIZER_H_
