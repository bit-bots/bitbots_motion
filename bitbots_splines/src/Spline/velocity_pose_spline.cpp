#include <bitbots_splines/velocity_pose_spline.h>

namespace bitbots_splines {
VelocityPoseSpline::VelocityPoseSpline() {}


void VelocityPoseSpline::addPoint(const tf2::Vector3 &position, const tf2::Quaternion &orientation) {
  tf2::Transform pose(orientation, position);
  addPoint(pose);
}

void VelocityPoseSpline::addPoint(const geometry_msgs::Pose &pose) {
  tf2::Transform pose_transform;
  tf2::convert(pose, pose_transform);

  addPoint(pose_transform);
}

void VelocityPoseSpline::addPoint(const tf2::Transform &pose) {
  poses_.push_back(pose);
  recalculate_splines();
}

void VelocityPoseSpline::addPoint(double x, double y, double z, double roll, double pitch, double yaw) {
  tf2::Transform pose;
  pose.setOrigin({x, y, z});
  tf2::Quaternion rotation;
  rotation.setRPY(roll, pitch, yaw);
  pose.setRotation(rotation);
  addPoint(pose);
}

void VelocityPoseSpline::addPoint(double x, double y, double z, const geometry_msgs::Quaternion &orientation) {
  tf2::Transform pose;
  pose.setOrigin({x, y, z});
  tf2::Quaternion rotation;
  tf2::convert(orientation, rotation);
  pose.setRotation(rotation);
  addPoint(pose);
}

void VelocityPoseSpline::recalculate_splines() {
  if (poses_.empty()) {
    return;
  }
  times_.clear();

  times_.push_back(0);
  addPoint(poses_.at(0), times_.back(), 0, 0, 0);
  if (offset_ > 0) {
    // duplicate first point at time offset
    times_.at(0) = offset_;
    addPoint(poses_.at(0), times_.back(), 0, 0, 0);
  }
  // get cartesian distances between positions
  for (std::size_t i = 1; i < poses_.size(); ++i) {
    tf2::Transform p1 = poses_.at(i - 1);
    tf2::Transform p2 = poses_.at(i);
    double distance = p1.getOrigin().distance(p2.getOrigin());
    double time = std::abs(distance) / velocity_;
    times_.push_back(times_.back() + time);

    // get velocity per axis
    double vel_x = (p2.getOrigin().x() - p1.getOrigin().x()) / time;
    double vel_y = (p2.getOrigin().y() - p1.getOrigin().y()) / time;
    double vel_z = (p2.getOrigin().z() - p1.getOrigin().z()) / time;

    addPoint(p2, times_.back(), vel_x, vel_y, vel_z);
  }

}

void VelocityPoseSpline::addPoint(const tf2::Transform &pose, double time, double vel_x, double vel_y, double vel_z) {
  x_.addPoint(time, pose.getOrigin().x(), vel_x, 0);
  y_.addPoint(time, pose.getOrigin().y(), vel_y, 0);
  z_.addPoint(time, pose.getOrigin().z(), vel_z, 0);

  double r, p, y;
  tf2::Matrix3x3(pose.getRotation()).getRPY(r, p ,y);

  // angular velocities are currently always 0
  roll_.addPoint(time, r, 0, 0);
  pitch_.addPoint(time, p, 0, 0);
  yaw_.addPoint(time, y, 0, 0);
}

void VelocityPoseSpline::setVelocity(double velocity) {
  velocity_ = velocity;
  recalculate_splines();
}

double VelocityPoseSpline::getVelocity() {
  return velocity_;
}

void VelocityPoseSpline::setTimeOffset(double offset) {
  offset_ = offset;
  recalculate_splines();
}

std::vector<double> VelocityPoseSpline::getTimes() {
  return times_;
}
}
