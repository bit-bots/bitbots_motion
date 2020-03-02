#include "bitbots_splines/pose_spline.h"

namespace bitbots_splines {

tf2::Transform PoseSpline::getTfTransform(double time) {
  tf2::Transform trans;
  trans.setOrigin(getPositionPos(time));
  trans.setRotation(getOrientation(time));
  return trans;
}
geometry_msgs::Pose PoseSpline::getGeometryMsgPose(double time) {
  geometry_msgs::Pose msg;
  msg.position = getGeometryMsgPosition(time);
  msg.orientation = getGeometryMsgOrientation(time);
  return msg;
}

geometry_msgs::Point PoseSpline::getGeometryMsgPosition(double time) {
  geometry_msgs::Point msg;
  tf2::Vector3 tf_vec = getPositionPos(time);
  msg.x = tf_vec.x();
  msg.y = tf_vec.y();
  msg.z = tf_vec.z();
  return msg;
}

geometry_msgs::Quaternion PoseSpline::getGeometryMsgOrientation(double time) {
  geometry_msgs::Quaternion msg;
  tf2::convert(getOrientation(time), msg);
  return msg;
}

tf2::Vector3 PoseSpline::getPositionPos(double time) {
  tf2::Vector3 pos;
  pos[0] = x_.pos(time);
  pos[1] = y_.pos(time);
  pos[2] = z_.pos(time);
  return pos;
}

tf2::Vector3 PoseSpline::getPositionVel(double time) {
  tf2::Vector3 vel;
  vel[0] = x_.vel(time);
  vel[1] = y_.vel(time);
  vel[2] = z_.vel(time);
  return vel;
}
tf2::Vector3 PoseSpline::getPositionAcc(double time) {
  tf2::Vector3 acc;
  acc[0] = x_.acc(time);
  acc[1] = y_.acc(time);
  acc[2] = z_.acc(time);
  return acc;
}

tf2::Vector3 PoseSpline::getEulerAngles(double time) {
  tf2::Vector3 pos;
  pos[0] = roll_.pos(time);
  pos[1] = pitch_.pos(time);
  pos[2] = yaw_.pos(time);
  return pos;
}
tf2::Vector3 PoseSpline::getEulerVel(double time) {
  tf2::Vector3 vel;
  vel[0] = roll_.vel(time);
  vel[1] = pitch_.vel(time);
  vel[2] = yaw_.vel(time);
  return vel;
}
tf2::Vector3 PoseSpline::getEulerAcc(double time) {
  tf2::Vector3 acc;
  acc[0] = roll_.acc(time);
  acc[1] = pitch_.acc(time);
  acc[2] = yaw_.acc(time);
  return acc;
}

tf2::Quaternion PoseSpline::getOrientation(double time) {
  tf2::Quaternion quat;
  tf2::Vector3 rpy = getEulerAngles(time);
  quat.setRPY(rpy[0], rpy[1], rpy[2]);
  quat.normalize();
  return quat;
}

void PoseSpline::addPoint(double time, const tf2::Vector3 &position, const tf2::Quaternion &orientation) {
  tf2::Transform pose(orientation, position);
  addPoint(time, pose);
}

void PoseSpline::addPoint(double time, const geometry_msgs::Pose &pose) {
  tf2::Transform pose_transform;
  tf2::convert(pose, pose_transform);

  addPoint(time, pose_transform);
}

void PoseSpline::addPoint(double time, const tf2::Transform &pose) {
  double r, p, y;
  tf2::Matrix3x3(pose.getRotation()).getRPY(r, p, y);
  addPoint(time, pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z(), r, p ,y);
}

void PoseSpline::addPoint(double time, double x, double y, double z, double roll, double pitch, double yaw) {
  x_.addPoint(time, x);
  y_.addPoint(time, y);
  z_.addPoint(time, z);
  roll_.addPoint(time, roll);
  pitch_.addPoint(time, pitch);
  yaw_.addPoint(time, yaw);
}

void PoseSpline::addPoint(double time, double x, double y, double z, const geometry_msgs::Quaternion &orientation) {
  tf2::Transform pose;
  pose.setOrigin({x, y, z});
  tf2::Quaternion rotation;
  tf2::convert(orientation, rotation);
  pose.setRotation(rotation);
  addPoint(time, pose);
}

SmoothSpline *PoseSpline::x() {
  return &x_;
}

SmoothSpline *PoseSpline::y() {
  return &y_;
}

SmoothSpline *PoseSpline::z() {
  return &z_;
}

SmoothSpline *PoseSpline::roll() {
  return &roll_;
}

SmoothSpline *PoseSpline::pitch() {
  return &pitch_;
}

SmoothSpline *PoseSpline::yaw() {
  return &yaw_;
}

std::string PoseSpline::getDebugString() {
  std::string output;
  output += "x:\n" + x_.getDebugString() + "\n";
  output += "y:\n" + y_.getDebugString() + "\n";
  output += "z:\n" + z_.getDebugString() + "\n";
  output += "roll:\n" + roll_.getDebugString() + "\n";
  output += "pitch:\n" + pitch_.getDebugString() + "\n";
  output += "yaw:\n" + yaw_.getDebugString() + "\n";
  return output;
}

}