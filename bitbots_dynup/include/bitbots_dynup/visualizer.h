#ifndef BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_VISUALIZER_H_
#define BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_VISUALIZER_H_

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <bitbots_splines/smooth_spline.h>
#include <bitbots_splines/spline_container.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <bitbots_splines/abstract_visualizer.h>

namespace bitbots_dynup {

struct VisualizationParams {
  int spline_smoothness;
};

class Visualizer : bitbots_splines::AbstractVisualizer {
 public:

  Visualizer(const std::string &base_topic, rclcpp::Node::SharedPtr node);

  void setParams(VisualizationParams params);

  void displaySplines(bitbots_splines::PoseSpline splines, const std::string &frame);

 private:
  
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr spline_publisher_;
  std::string base_topic_;
  rclcpp::Node::SharedPtr node_;
  const std::string marker_ns_ = "bitbots_dynup";
  VisualizationParams params_;
};
}

#endif //BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_VISUALIZER_H_
