#include "bitbots_dynamic_kick/stabilizer.h"

namespace bitbots_dynamic_kick {

Stabilizer::Stabilizer() {
  ros::NodeHandle pid_x_nh = ros::NodeHandle("dynamic_kick/pid_x");
  ros::NodeHandle pid_y_nh = ros::NodeHandle("dynamic_kick/pid_y");
  ros::NodeHandle pid_roll_nh = ros::NodeHandle("dynamic_kick/pid_roll");
  ros::NodeHandle pid_pitch_nh = ros::NodeHandle("dynamic_kick/pid_pitch");
  pid_x_.init(pid_x_nh, false);
  pid_y_.init(pid_y_nh, false);
  pid_roll_.init(pid_roll_nh, false);
  pid_pitch_.init(pid_pitch_nh, false);
  reset();
}

void Stabilizer::reset() {
  pid_x_.reset();
  pid_y_.reset();
  pid_roll_.reset();
  pid_pitch_.reset();
}

KickPositions Stabilizer::stabilize(const KickPositions &positions, const ros::Duration &dt) {
  KickPositions stabilized_positions = positions;
  if (positions.cop_support_point && use_cop_) {
    /* calculate stabilizing target from center of pressure
     * the cop is in corresponding sole frame
     * optimal stabilizing would be centered above sole center */
    double cop_x, cop_y, cop_x_error, cop_y_error;
    if (positions.is_left_kick) {
      cop_x = cop_right.x;
      cop_y = cop_right.y;
    } else {
      cop_x = cop_left.x;
      cop_y = cop_left.y;
    }
    cop_x_error = cop_x - positions.trunk_pose.translation().x();
    cop_y_error = cop_y - positions.trunk_pose.translation().y();

    double x_correction = pid_x_.computeCommand(cop_x_error, dt);
    double y_correction = pid_y_.computeCommand(cop_y_error, dt);

    stabilized_positions.trunk_pose.translation().x() += x_correction;
    stabilized_positions.trunk_pose.translation().y() += y_correction;
  }
  if (use_final_stabilizing_ && positions.engine_phase == KickPhase::DONE) {
    Eigen::Quaterniond imu_orientation;
    tf2::convert(imu_.orientation, imu_orientation);
    rot_conv::FusedAngles current_orientation = rot_conv::FusedFromQuat(imu_orientation);

    rot_conv::FusedAngles
        goal_orientation = rot_conv::FusedFromQuat(Eigen::Quaterniond(positions.trunk_pose.rotation()));
    // adapt trunk based on PID controller
    goal_orientation.fusedPitch +=
        pid_pitch_.computeCommand(goal_orientation.fusedPitch - current_orientation.fusedPitch, dt);
    goal_orientation.fusedRoll +=
        pid_roll_.computeCommand(goal_orientation.fusedRoll - current_orientation.fusedRoll, dt);

    is_stable_ = (abs(goal_orientation.fusedPitch - current_orientation.fusedPitch) < stable_threshold_) &&
        (abs(goal_orientation.fusedRoll - current_orientation.fusedRoll) < stable_threshold_);
  }

  return stabilized_positions;
}

void Stabilizer::useCop(bool use) {
  use_cop_ = use;
}

void Stabilizer::setRobotModel(moveit::core::RobotModelPtr model) {
  kinematic_model_ = std::move(model);
}

bool Stabilizer::isStable() const {
  return is_stable_;
}

void Stabilizer::setStableThreshold(double threshold) {
  stable_threshold_ = threshold;
}

void Stabilizer::setUseFinalStabilizing(bool use_final_stabilizing) {
  use_final_stabilizing_ = use_final_stabilizing;
}

}
