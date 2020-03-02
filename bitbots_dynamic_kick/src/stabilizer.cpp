#include "bitbots_dynamic_kick/stabilizer.h"

namespace bitbots_dynamic_kick {

Stabilizer::Stabilizer() {
  pid_cartesian_trunk_cop_x_.init({"dynamic_kick/pid_cartesian_trunk_x"}, false);
  pid_cartesian_trunk_cop_y_.init({"dynamic_kick/pid_cartesian_trunk_y"}, false);
  pid_joint_hip_cop_x_.init({"dynamic_kick/pid_joint_hip_x"}, false);
  pid_joint_hip_cop_y_.init({"dynamic_kick/pid_joint_hip_y"}, false);
  pid_joint_ankle_imu_orientation_roll_.init({"dynamic_kick/pid_joint_ankle_imu_orientation_roll"}, false);
  pid_joint_ankle_imu_orientation_pitch_.init({"dynamic_kick/pid_joint_ankle_imu_orientation_pitch"}, false);
  pid_joint_ankle_imu_velocity_roll_.init({"dynamic_kick/pid_joint_ankle_imu_velocity_roll"}, false);
  pid_joint_ankle_imu_velocity_pitch_.init({"dynamic_kick/pid_joint_ankle_imu_velocity_pitch"}, false);
  pid_joint_hip_imu_velocity_roll_.init({"dynamic_kick/pid_joint_hip_imu_velocity_roll"}, false);
  pid_joint_hip_imu_velocity_pitch_.init({"dynamic_kick/pid_joint_hip_imu_velocity_pitch"}, false);
  reset();
}

void Stabilizer::reset() {
  pid_cartesian_trunk_cop_x_.reset();
  pid_cartesian_trunk_cop_y_.reset();
  pid_joint_hip_cop_x_.reset();
  pid_joint_hip_cop_y_.reset();
  pid_joint_ankle_imu_orientation_roll_.reset();
  pid_joint_ankle_imu_orientation_pitch_.reset();
  pid_joint_ankle_imu_velocity_roll_.reset();
  pid_joint_ankle_imu_velocity_pitch_.reset();
  pid_joint_hip_imu_velocity_roll_.reset();
  pid_joint_hip_imu_velocity_pitch_.reset();
}

KickPositions Stabilizer::stabilize(const KickPositions &positions, const ros::Duration &dt) {
  return positions;
}

bitbots_splines::JointGoals Stabilizer::stabilizeGoals(const bitbots_splines::JointGoals &goals,
                                                       const ros::Duration &dt, const KickPositions &positions) {
  return goals;
  //return jointAnkleImuVelocity(goals, dt, positions);
}

bitbots_splines::JointGoals Stabilizer::jointAnkleCop(const bitbots_splines::JointGoals &goals,
                                                      const ros::Duration &dt,
                                                      const KickPositions &positions) {
  return goals;
}

bitbots_splines::JointGoals Stabilizer::jointAnkleImuOrientation(const bitbots_splines::JointGoals &goals,
                                                                 const ros::Duration &dt,
                                                                 const KickPositions &positions) {
  //double

  // error is target - state
  double roll_correction = pid_joint_ankle_imu_orientation_roll_.computeCommand(imu_roll_, dt);
  double pitch_correction = pid_joint_ankle_imu_orientation_pitch_.computeCommand(imu_pitch_, dt);

  bitbots_splines::JointGoals stabilized_goals = goals;

  std::string support_roll_joint, support_pitch_joint;
  if (positions.is_left_kick) {
    support_roll_joint = "RAnkleRoll";
    support_pitch_joint = "RAnklePitch";
  } else {
    support_roll_joint = "LAnkleRoll";
    support_pitch_joint = "LAnklePitch";
  }

  for (int i = 0; i < stabilized_goals.first.size(); ++i) {
    if (stabilized_goals.first.at(i) == support_roll_joint) {
      stabilized_goals.second.at(i) += roll_correction;
    } else if (stabilized_goals.first.at(i) == support_pitch_joint) {
      stabilized_goals.second.at(i) += pitch_correction;
    }
  }

  return stabilized_goals;
}

bitbots_splines::JointGoals Stabilizer::jointAnkleImuOrientationFused(const bitbots_splines::JointGoals &goals,
                                                                      const ros::Duration &dt,
                                                                      const KickPositions &positions) {
  return goals;
}

bitbots_splines::JointGoals Stabilizer::jointAnkleImuVelocity(const bitbots_splines::JointGoals &goals,
                                                              const ros::Duration &dt,
                                                              const KickPositions &positions) {
  // error is target - state, here target is 0
  double roll_correction = pid_joint_ankle_imu_velocity_roll_.computeCommand(0 - imu_roll_velocity_, dt);
  double pitch_correction = pid_joint_ankle_imu_velocity_pitch_.computeCommand(0 - imu_pitch_velocity_, dt);

  bitbots_splines::JointGoals stabilized_goals = goals;

  std::string support_roll_joint, support_pitch_joint;
  if (positions.is_left_kick) {
    support_roll_joint = "RAnkleRoll";
    support_pitch_joint = "RAnklePitch";
  } else {
    support_roll_joint = "LAnkleRoll";
    support_pitch_joint = "LAnklePitch";
  }

  for (int i = 0; i < stabilized_goals.first.size(); ++i) {
    if (stabilized_goals.first.at(i) == support_roll_joint) {
      stabilized_goals.second.at(i) += roll_correction;
    } else if (stabilized_goals.first.at(i) == support_pitch_joint) {
      stabilized_goals.second.at(i) += pitch_correction;
    }
  }

  return stabilized_goals;
}

bitbots_splines::JointGoals Stabilizer::jointHipCop(const bitbots_splines::JointGoals &goals,
                                                    const ros::Duration &dt,
                                                    const KickPositions &positions) {
  bitbots_splines::JointGoals stabilized_goals = goals;
  if (positions.cop_support_point && use_cop_) {
    auto[cop_x_error, cop_y_error] = getCopErrors(positions);

    double x_correction = pid_cartesian_trunk_cop_x_.computeCommand(cop_x_error, dt);
    double y_correction = pid_cartesian_trunk_cop_y_.computeCommand(cop_y_error, dt);

    auto[roll_joint_name, pitch_joint_name] = getHipJoints(positions);

    stabilized_goals = bitbots_splines::joint_goals_update_diff(stabilized_goals,
                                                                {roll_joint_name, pitch_joint_name},
                                                                {x_correction, y_correction});
  }

  return stabilized_goals;
}

bitbots_splines::JointGoals Stabilizer::jointHipImuOrientation(const bitbots_splines::JointGoals &goals,
                                                               const ros::Duration &dt,
                                                               const KickPositions &positions) {
  return goals;
}

bitbots_splines::JointGoals Stabilizer::jointHipImuOrientationFused(const bitbots_splines::JointGoals &goals,
                                                                    const ros::Duration &dt,
                                                                    const KickPositions &positions) {
  return goals;
}

bitbots_splines::JointGoals Stabilizer::jointHipImuVelocity(const bitbots_splines::JointGoals &goals,
                                                            const ros::Duration &dt,
                                                            const KickPositions &positions) {
  bitbots_splines::JointGoals stabilized_goals = goals;
  auto[roll_joint_name, pitch_joint_name] = getHipJoints(positions);

  // error is target - state, here target is 0
  double roll_correction = pid_joint_hip_imu_velocity_roll_.computeCommand(0 - imu_roll_velocity_, dt);
  double pitch_correction = pid_joint_hip_imu_velocity_pitch_.computeCommand(0 - imu_pitch_velocity_, dt);
  
  stabilized_goals = bitbots_splines::joint_goals_update_diff(stabilized_goals,
                                                              {roll_joint_name, pitch_joint_name},
                                                              {roll_correction, pitch_correction});

  return stabilized_goals;
}

KickPositions Stabilizer::cartesianTrunkCop(const bitbots_dynamic_kick::KickPositions &positions,
                                            const ros::Duration &dt) {
  KickPositions stabilized_positions = positions;
  if (positions.cop_support_point && use_cop_) {
    auto[cop_x_error, cop_y_error] = getCopErrors(positions);

    double x_correction = pid_joint_hip_cop_x_.computeCommand(cop_x_error, dt);
    double y_correction = pid_joint_hip_cop_y_.computeCommand(cop_y_error, dt);

    stabilized_positions.trunk_pose.getOrigin() += {x_correction, y_correction, 0};
  }
  return stabilized_positions;
}

KickPositions Stabilizer::cartesianImuOrientation(const bitbots_dynamic_kick::KickPositions &positions,
                                                  const ros::Duration &dt) {
  return positions;
}

KickPositions Stabilizer::cartesianImuOrientationFused(const bitbots_dynamic_kick::KickPositions &positions,
                                                       const ros::Duration &dt) {
  return positions;
}

KickPositions Stabilizer::cartesianImuVelocity(const bitbots_dynamic_kick::KickPositions &positions,
                                               const ros::Duration &dt) {
  return positions;

}

std::pair<double, double> Stabilizer::getCopErrors(const bitbots_dynamic_kick::KickPositions &positions) {
  double cop_x, cop_y, cop_x_error, cop_y_error;
  if (positions.is_left_kick) {
    cop_x = cop_right.x;
    cop_y = cop_right.y;
  } else {
    cop_x = cop_left.x;
    cop_y = cop_left.y;
  }
  cop_x_error = positions.trunk_pose.getOrigin().getX() - cop_x;
  cop_y_error = positions.trunk_pose.getOrigin().getY() - cop_y;

  return {cop_x_error, cop_y_error};
}

std::pair<std::string, std::string> Stabilizer::getHipJoints(const KickPositions &positions) {
  std::string roll_joint_name, pitch_joint_name;
  if (positions.is_left_kick) {
    roll_joint_name = "LHipRoll";
    pitch_joint_name = "LHipPitch";
  } else {
    roll_joint_name = "RHipRoll";
    pitch_joint_name = "LHipPitch";
  }

  return std::pair<std::string, std::string> {roll_joint_name, pitch_joint_name};
}

void Stabilizer::useCop(bool use) {
  use_cop_ = use;
}

void Stabilizer::setRobotModel(moveit::core::RobotModelPtr model) {
  kinematic_model_ = std::move(model);
}

void Stabilizer::IMUCallback(const sensor_msgs::Imu &imu_msg) {
  imu_roll_velocity_ = imu_msg.angular_velocity.x;
  imu_pitch_velocity_ = imu_msg.angular_velocity.y;
  imu_roll_ = imu_msg.orientation.x;
  imu_pitch_ = imu_msg.orientation.y;
}

}
