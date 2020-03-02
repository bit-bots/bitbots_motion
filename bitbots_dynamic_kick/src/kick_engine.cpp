#include "bitbots_dynamic_kick/kick_engine.h"

namespace bitbots_dynamic_kick {

KickEngine::KickEngine() :
    listener_(tf_buffer_) {
}

void KickEngine::reset() {
  time_ = 0;
  trunk_spline_ = bitbots_splines::PoseSpline();
  flying_foot_spline_ = bitbots_splines::VelocityPoseSpline();
}

void KickEngine::setGoals(const KickGoals &goals) {
  is_left_kick_ = calcIsLeftFootKicking(goals.header,
                                        goals.ball_position,
                                        goals.kick_direction);
  // TODO Internal state is dirty when goal transformation fails

  /* Save given goals because we reuse them later */
  auto transformed_goal = transformGoal((is_left_kick_) ? "r_sole" : "l_sole",
                                        goals.header, goals.ball_position, goals.kick_direction);
  tf2::convert(transformed_goal.first, ball_position_);
  tf2::convert(transformed_goal.second, kick_direction_);
  kick_direction_.normalize();
  kick_speed_ = goals.kick_speed;

  time_ = 0;

  /* Plan new splines according to new goal */
  calcSplines(is_left_kick_ ? goals.l_foot_pose : goals.r_foot_pose, getTrunkPose());
}

KickPositions KickEngine::update(double dt) {
  /* Only do an actual update when splines are present */
  KickPositions positions;
  /* Get should-be pose from planned splines (every axis) at current time */
  positions.trunk_pose = trunk_spline_.getTfTransform(time_);
  positions.flying_foot_pose = flying_foot_spline_.getTfTransform(time_);
  positions.is_left_kick = is_left_kick_;

  /* calculate if we want to use center-of-pressure in the current phase
   * use COP based support point only when the weight is on the support foot
   * while raising/lowering the foot, the weight is not completely on the support foot (that's why /2.0)*/
  if (time_ > phase_timings_.move_trunk + phase_timings_.raise_foot / 2.0 &&
      time_ < phase_timings_.move_back + phase_timings_.lower_foot / 2.0) {
    positions.cop_support_point = true;
  } else {
    positions.cop_support_point = false;
  }

  time_ += dt;

  /* Stabilize and return result */
  return positions;
}

geometry_msgs::Transform KickEngine::getTrunkPose() {
  geometry_msgs::TransformStamped trunk_frame;
  if (is_left_kick_) {
    trunk_frame = tf_buffer_.lookupTransform("r_sole", "base_link", ros::Time(0));
  } else {
    trunk_frame = tf_buffer_.lookupTransform("l_sole", "base_link", ros::Time(0));
  }
  return trunk_frame.transform;
}

void KickEngine::calcSplines(const geometry_msgs::Pose &flying_foot_pose, const geometry_msgs::Transform &trunk_pose) {
  /*
   * Add current position, target position and current position to splines so that they describe a smooth
   * curve to the ball and back
   */
  /* Splines:
   * - stand
   * - move trunk
   * - raise foot
   * - kick
   * - move foot back
   * - lower foot
   *  - move trunk back
   */

  int kick_foot_sign;
  if (is_left_kick_) {
    kick_foot_sign = 1;
  } else {
    kick_foot_sign = -1;
  }

  windup_point_ = calcKickWindupPoint();

  /* build vector of speeds in each direction */
  //double speed_yaw = tf2::getYaw(kick_direction_);
  //tf2::Vector3 speed_vector(cos(speed_yaw), sin(speed_yaw), 0);

  /* Flying foot orientation */
  /* Construct a start_rotation as quaternion from Pose msg */
  tf2::Quaternion start_rotation(flying_foot_pose.orientation.x, flying_foot_pose.orientation.y,
                                 flying_foot_pose.orientation.z, flying_foot_pose.orientation.w);
  double start_r, start_p, start_y;
  tf2::Matrix3x3(start_rotation).getRPY(start_r, start_p, start_y);

  double target_y = calcKickFootYaw();

  /* Flying foot position */
  flying_foot_spline_.setVelocity(params_.velocity);
  flying_foot_spline_.setTimeOffset(params_.move_trunk_time);
  // start pose
  flying_foot_spline_.addPoint(flying_foot_pose);
  // move trunk
  flying_foot_spline_.addPoint(0, kick_foot_sign * params_.foot_distance, 0, flying_foot_pose.orientation);
  // raise foot
  flying_foot_spline_
      .addPoint(0, kick_foot_sign * params_.foot_distance, params_.foot_rise, flying_foot_pose.orientation);
  // windup point
  flying_foot_spline_.addPoint(windup_point_.x(), windup_point_.y(), params_.foot_rise, start_r, start_p, target_y);
  // kick
  flying_foot_spline_.addPoint(ball_position_.x(), ball_position_.y(), params_.foot_rise, start_r, start_p, target_y);
  // move foot back
  flying_foot_spline_
      .addPoint(0, kick_foot_sign * params_.foot_distance, params_.foot_rise, flying_foot_pose.orientation);
  // lower foot
  flying_foot_spline_
      .addPoint(0, kick_foot_sign * params_.foot_distance, 0.4 * params_.foot_rise, flying_foot_pose.orientation);
  // move trunk back
  flying_foot_spline_.addPoint(0, kick_foot_sign * params_.foot_distance, 0, flying_foot_pose.orientation);

  std::vector times = flying_foot_spline_.getTimes();
  phase_timings_.move_trunk = times.at(1);
  phase_timings_.raise_foot = times.at(2);
  phase_timings_.windup = times.at(3);
  phase_timings_.kick = times.at(4);
  phase_timings_.move_back = times.at(5);
  phase_timings_.lower_foot = times.at(6);
  phase_timings_.move_trunk_back = times.at(7);

  /* Construct quaternion for trunk rotation */
  tf2::Quaternion trunk_rotation(trunk_pose.rotation.x, trunk_pose.rotation.y,
                                 trunk_pose.rotation.z, trunk_pose.rotation.w);
  double trunk_r, trunk_p, trunk_y;
  tf2::Matrix3x3(trunk_rotation).getRPY(trunk_r, trunk_p, trunk_y);

  // start pose
  trunk_spline_.addPoint(0,
                         0,
                         kick_foot_sign * (params_.foot_distance / 2.0),
                         trunk_pose.translation.z,
                         trunk_r,
                         trunk_p,
                         trunk_y);
  // move trunk
  trunk_spline_.addPoint(phase_timings_.move_trunk,
                         params_.stabilizing_point_x,
                         -kick_foot_sign * params_.stabilizing_point_y,
                         params_.trunk_height,
                         trunk_r,
                         trunk_p,
                         trunk_y);
  // raise foot
  trunk_spline_.addPoint(phase_timings_.raise_foot,
                         params_.stabilizing_point_x,
                         -kick_foot_sign * params_.stabilizing_point_y,
                         params_.trunk_height,
                         kick_foot_sign * params_.trunk_roll,
                         params_.trunk_pitch,
                         kick_foot_sign * params_.trunk_yaw);
  // windup point
  trunk_spline_.addPoint(phase_timings_.windup,
                         params_.stabilizing_point_x,
                         -kick_foot_sign * params_.stabilizing_point_y,
                         params_.trunk_height,
                         kick_foot_sign * params_.trunk_roll,
                         params_.trunk_pitch,
                         kick_foot_sign * params_.trunk_yaw);
  // kick
  trunk_spline_.addPoint(phase_timings_.kick,
                         params_.stabilizing_point_x,
                         -kick_foot_sign * params_.stabilizing_point_y,
                         params_.trunk_height,
                         kick_foot_sign * params_.trunk_roll,
                         params_.trunk_pitch,
                         kick_foot_sign * params_.trunk_yaw);
  // move foot back
  trunk_spline_.addPoint(phase_timings_.move_back,
                         params_.stabilizing_point_x,
                         -kick_foot_sign * params_.stabilizing_point_y,
                         params_.trunk_height,
                         kick_foot_sign * params_.trunk_roll,
                         params_.trunk_pitch,
                         kick_foot_sign * params_.trunk_yaw);
  // lower foot
  trunk_spline_.addPoint(phase_timings_.lower_foot,
                         params_.stabilizing_point_x,
                         -kick_foot_sign * params_.stabilizing_point_y,
                         params_.trunk_height,
                         kick_foot_sign * params_.trunk_roll,
                         params_.trunk_pitch,
                         kick_foot_sign * params_.trunk_yaw);
  // move trunk back
  trunk_spline_.addPoint(phase_timings_.move_trunk_back,
                         0,
                         kick_foot_sign * (params_.foot_distance / 2.0),
                         trunk_pose.translation.z,
                         trunk_r,
                         trunk_p,
                         trunk_y);
}

std::pair<geometry_msgs::Point, geometry_msgs::Quaternion> KickEngine::transformGoal(
    const std::string &support_foot_frame,
    const std_msgs::Header &header,
    const geometry_msgs::Vector3 &ball_position,
    const geometry_msgs::Quaternion &kick_direction) {
  /* construct stamped goals so that they can be transformed */ // TODO Extract this into own function because we do it multiple times
  geometry_msgs::PointStamped
      stamped_position;       // TODO Make KickGoal a point as well so we dont have to do transformations here
  stamped_position.point.x = ball_position.x;
  stamped_position.point.y = ball_position.y;
  stamped_position.point.z = ball_position.z;
  stamped_position.header = header;
  //stamped_position.vector = ball_position;
  geometry_msgs::QuaternionStamped stamped_direction;
  stamped_direction.header = header;
  stamped_direction.quaternion = kick_direction;

  /* do transform into support_foot frame */
  geometry_msgs::PointStamped transformed_position;
  geometry_msgs::QuaternionStamped transformed_direction;

  tf_buffer_.transform(stamped_position, transformed_position, support_foot_frame, ros::Duration(0.2));
  tf_buffer_.transform(stamped_direction, transformed_direction, support_foot_frame, ros::Duration(0.2));

  auto x = tf_buffer_.lookupTransform(support_foot_frame, header.frame_id, header.stamp, ros::Duration(0.2));

  return std::pair(transformed_position.point, transformed_direction.quaternion);
}

tf2::Vector3 KickEngine::calcKickWindupPoint() {
  /* retrieve yaw from kick_direction_ */
  double yaw = tf2::getYaw(kick_direction_);

  /* create a vector which points in the negative direction of kick_direction_ */
  tf2::Vector3 vec(cos(yaw), sin(yaw), 0);
  vec.normalize();

  /* take windup distance into account */
  vec *= -params_.kick_windup_distance;

  /* add the ball position because the windup point is in support_foot_frame and not ball_frame */
  vec += ball_position_;

  vec.setZ(params_.foot_rise);

  return vec;
}

bool KickEngine::calcIsLeftFootKicking(const std_msgs::Header &header,
                                       const geometry_msgs::Vector3 &ball_position,
                                       const geometry_msgs::Quaternion &kick_direction) {
  /* prepare variables with stamps */
  geometry_msgs::Vector3Stamped stamped_position;
  stamped_position.header = header;
  stamped_position.vector = ball_position;
  geometry_msgs::QuaternionStamped stamped_direction;
  stamped_direction.header = header;
  stamped_direction.quaternion = kick_direction;

  /* transform ball data into frame where we want to apply it */
  tf2::Stamped<tf2::Vector3> transformed_ball_position;
  tf_buffer_.transform(stamped_position, transformed_ball_position, "base_footprint", ros::Duration(0.2));
  tf2::Stamped<tf2::Quaternion> transformed_direction;
  tf_buffer_.transform(stamped_direction, transformed_direction, "base_footprint", ros::Duration(0.2));

  /*
   * check if ball is outside of an imaginary corridor
   * if it is not, we use a more fined grained criterion which takes kick_direction into account
   */
  if (transformed_ball_position.y() > params_.choose_foot_corridor_width / 2)
    return true;
  else if (transformed_ball_position.y() < -params_.choose_foot_corridor_width / 2)
    return false;

  /* use the more fine grained angle based criterion
   * angle_1 = angle between "forward" and "origin-to-ball-position"
   * angle_2 = yaw of kick_direction
   * angle_diff = difference between the two on which the decision happens
   */
  double angle_1 = transformed_ball_position.angle({1, 0, 0});
  angle_1 *= transformed_ball_position.y() < 0 ? -1 : 1;

  double angle_2 = tf2::getYaw(transformed_direction);
  double angle_diff = angle_2 - angle_1;

  ROS_INFO_STREAM("Choosing " << ((angle_diff < 0) ? "left" : "right") << " foot to kick");

  return angle_diff < 0;
}

double KickEngine::calcKickFootYaw() {
  double kick_roll_angle, kick_pitch_angle, kick_yaw_angle;
  tf2::Matrix3x3(kick_direction_).getRPY(kick_roll_angle, kick_pitch_angle, kick_yaw_angle);

  if (kick_yaw_angle > M_PI_4) {
    return kick_yaw_angle - M_PI_2;
  } else if (kick_yaw_angle < -M_PI_4) {
    return kick_yaw_angle + M_PI_2;
  } else {
    return kick_yaw_angle;
  }

}

bool KickEngine::isLeftKick() {
  return is_left_kick_;
}

int KickEngine::getPercentDone() const {
  return std::min(int(time_ / phase_timings_.move_trunk_back * 100), 100);
}

bitbots_splines::PoseSpline KickEngine::getFlyingSplines() const {
  return flying_foot_spline_;
}

bitbots_splines::PoseSpline KickEngine::getTrunkSplines() const {
  return trunk_spline_;
}

KickPhase KickEngine::getPhase() const {
  if (time_ == 0)
    return KickPhase::INITIAL;
  else if (time_ <= phase_timings_.move_trunk)
    return KickPhase::MOVE_TRUNK;
  else if (time_ <= phase_timings_.raise_foot)
    return KickPhase::RAISE_FOOT;
  else if (time_ <= phase_timings_.windup)
    return KickPhase::WINDUP;
  else if (time_ <= phase_timings_.kick)
    return KickPhase::KICK;
  else if (time_ <= phase_timings_.move_back)
    return KickPhase::MOVE_BACK;
  else if (time_ <= phase_timings_.lower_foot)
    return KickPhase::LOWER_FOOT;
  else if (time_ <= phase_timings_.move_trunk_back)
    return KickPhase::MOVE_TRUNK_BACK;
  else
    return KickPhase::DONE;
}

void KickEngine::setParams(KickParams params) {
  params_ = params;
}

tf2::Vector3 KickEngine::getWindupPoint() {
  return windup_point_;
}

}
