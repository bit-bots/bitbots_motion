#ifndef BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_CARTESIAN_VELOCITY_SPLINE_H_
#define BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_CARTESIAN_VELOCITY_SPLINE_H_

#include "pose_spline.h"

namespace bitbots_splines {

/*
 * The VelocityPoseSpline is a wrapper around PoseSpline. The
 * velocity of the movement can be defined instead of the timings
 * which will result in a smoother spline.
 */
class VelocityPoseSpline: public PoseSpline {
 public:
  VelocityPoseSpline();
  /*
   * set the velocity for the whole spline
   * @param velocity velocity in meters per second
   */
  void setVelocity(double velocity);
  double getVelocity();
  /*
   * set an offset before the first spline, for example to execute other movement
   * before the spline starts. This will result in the first spline point
   * being duplicated between times 0 and offset
   * @param offset offset in seconds
   */
  void setTimeOffset(double offset);
  std::vector<double> getTimes();

  void addPoint(const geometry_msgs::Pose &pose);
  void addPoint(const tf2::Vector3 &position, const tf2::Quaternion &orientation);
  void addPoint(double x, double y, double z, double roll, double pitch, double yaw);
  void addPoint(double x, double y, double z, const geometry_msgs::Quaternion &orientation);
  void addPoint(const tf2::Transform &pose);

 private:
  void recalculate_splines();
  void addPoint(const tf2::Transform &pose, double time, double vel_x, double vel_y, double vel_z);

  std::vector<tf2::Transform> poses_;
  std::vector<double> times_;
  double velocity_;
  double offset_ = 0;
};

}

#endif //BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_CARTESIAN_VELOCITY_SPLINE_H_
