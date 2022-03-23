#include "bitbots_quintic_walk/walk_pywrapper.h"

void PyWalkWrapper::spin_some() {
  rclcpp::spin_some(walk_node_);
}

PyWalkWrapper::PyWalkWrapper(std::string ns, std::vector<py::bytes> parameter_msgs) {
  // create parameters from serialized messages
  std::vector<rclcpp::Parameter> cpp_parameters = {};
  for (auto &parameter_msg: parameter_msgs) {
    cpp_parameters
        .push_back(rclcpp::Parameter::from_parameter_msg(fromPython<rcl_interfaces::msg::Parameter>(parameter_msg)));
  }
  walk_node_ = std::make_shared<bitbots_quintic_walk::WalkNode>(ns, cpp_parameters);
  set_robot_state(0);
  walk_node_->initializeEngine();
}

py::bytes PyWalkWrapper::step(double dt,
                              py::bytes &cmdvel_msg,
                              py::bytes &imu_msg,
                              py::bytes &jointstate_msg,
                              py::bytes &pressure_left,
                              py::bytes &pressure_right) {
  bitbots_msgs::msg::JointCommand result = walk_node_->step(dt,
                                                            std::make_shared<geometry_msgs::msg::Twist>(fromPython<
                                                                geometry_msgs::msg::Twist>(cmdvel_msg)),
                                                            std::make_shared<sensor_msgs::msg::Imu>(fromPython<
                                                                sensor_msgs::msg::Imu>(imu_msg)),
                                                            std::make_shared<sensor_msgs::msg::JointState>(fromPython<
                                                                sensor_msgs::msg::JointState>(jointstate_msg)),
                                                            std::make_shared<bitbots_msgs::msg::FootPressure>(fromPython<
                                                                bitbots_msgs::msg::FootPressure>(pressure_left)),
                                                            std::make_shared<bitbots_msgs::msg::FootPressure>(fromPython<
                                                                bitbots_msgs::msg::FootPressure>(pressure_right)));
  return toPython<bitbots_msgs::msg::JointCommand>(result);
}

py::bytes PyWalkWrapper::step_relative(double dt,
                                       py::bytes &step_msg,
                                       py::bytes &imu_msg,
                                       py::bytes &jointstate_msg,
                                       py::bytes &pressure_left,
                                       py::bytes &pressure_right) {
  bitbots_msgs::msg::JointCommand result = walk_node_->step_relative(dt,
                                                                     std::make_shared<geometry_msgs::msg::Twist>(
                                                                         fromPython<
                                                                             geometry_msgs::msg::Twist>(step_msg)),
                                                                     std::make_shared<sensor_msgs::msg::Imu>(fromPython<
                                                                         sensor_msgs::msg::Imu>(imu_msg)),
                                                                     std::make_shared<sensor_msgs::msg::JointState>(
                                                                         fromPython<
                                                                             sensor_msgs::msg::JointState>(
                                                                             jointstate_msg)),
                                                                     std::make_shared<bitbots_msgs::msg::FootPressure>(
                                                                         fromPython<
                                                                             bitbots_msgs::msg::FootPressure>(
                                                                             pressure_left)),
                                                                     std::make_shared<bitbots_msgs::msg::FootPressure>(
                                                                         fromPython<
                                                                             bitbots_msgs::msg::FootPressure>(
                                                                             pressure_right)));
  return toPython<bitbots_msgs::msg::JointCommand>(result);
}

py::bytes PyWalkWrapper::step_open_loop(double dt, py::bytes &cmdvel_msg) {
  geometry_msgs::msg::PoseArray result = walk_node_->step_open_loop(dt,
                                                                    std::make_shared<geometry_msgs::msg::Twist>(
                                                                        fromPython<geometry_msgs::msg::Twist>(cmdvel_msg)));
  return toPython<geometry_msgs::msg::PoseArray>(result);
}

py::bytes PyWalkWrapper::get_left_foot_pose() {
  geometry_msgs::msg::Pose result = walk_node_->get_left_foot_pose();
  return toPython<geometry_msgs::msg::Pose>(result);
}
py::bytes PyWalkWrapper::get_right_foot_pose() {
  geometry_msgs::msg::Pose result = walk_node_->get_right_foot_pose();
  return toPython<geometry_msgs::msg::Pose>(result);
}

py::bytes PyWalkWrapper::get_odom() {
  nav_msgs::msg::Odometry result = walk_node_->getOdometry();
  return toPython<nav_msgs::msg::Odometry>(result);
}

void PyWalkWrapper::reset() {
  walk_node_->reset();
}

void PyWalkWrapper::special_reset(int state, double phase, py::bytes cmd_vel, bool reset_odometry) {
  bitbots_quintic_walk::WalkState walk_state;
  if (state == 0) {
    walk_state = bitbots_quintic_walk::WalkState::PAUSED;
  } else if (state == 1) {
    walk_state = bitbots_quintic_walk::WalkState::WALKING;
  } else if (state == 2) {
    walk_state = bitbots_quintic_walk::WalkState::IDLE;
  } else if (state == 3) {
    walk_state = bitbots_quintic_walk::WalkState::START_MOVEMENT;
  } else if (state == 4) {
    walk_state = bitbots_quintic_walk::WalkState::STOP_MOVEMENT;
  } else if (state == 5) {
    walk_state = bitbots_quintic_walk::WalkState::START_STEP;
  } else if (state == 6) {
    walk_state = bitbots_quintic_walk::WalkState::STOP_STEP;
  } else if (state == 7) {
    walk_state = bitbots_quintic_walk::WalkState::KICK;
  } else {
    RCLCPP_WARN(walk_node_->get_logger(), "state in special reset not clear");
    return;
  }
  walk_node_->reset(walk_state,
                    phase,
                    std::make_shared<geometry_msgs::msg::Twist>(fromPython<geometry_msgs::msg::Twist>(cmd_vel)),
                    reset_odometry);
}

double PyWalkWrapper::get_phase() {
  return walk_node_->getEngine()->getPhase();
}

double PyWalkWrapper::get_freq() {
  return walk_node_->getEngine()->getFreq();
}

void PyWalkWrapper::set_robot_state(int state) {
  humanoid_league_msgs::msg::RobotControlState state_msg;
  state_msg.state = state;
  walk_node_->robotStateCb(std::make_shared<humanoid_league_msgs::msg::RobotControlState>(state_msg));
}

void PyWalkWrapper::set_parameter(py::bytes parameter_msg) {
  // convert serialized parameter msg to parameter object
  rclcpp::Parameter
      parameter = rclcpp::Parameter::from_parameter_msg(fromPython<rcl_interfaces::msg::Parameter>(parameter_msg));

  // needs to be a vector
  std::vector<rclcpp::Parameter> parameters = {parameter};
  walk_node_->onSetParameters(parameters);
}

void PyWalkWrapper::publish_debug(){
  walk_node_->publish_debug();
}

void PyWalkWrapper::test_memory_leak_from(py::bytes cmdvel_msg_serialized){
    geometry_msgs::msg::Twist cmdvel_msg;
    fromPython<geometry_msgs::msg::Twist>(cmdvel_msg_serialized);
}

void PyWalkWrapper::test_memory_leak_to(){
    geometry_msgs::msg::Twist cmdvel_msg;
    toPython<geometry_msgs::msg::Twist>(cmdvel_msg);
}

template<typename T>
void PyWalkWrapper::test_memory_leak_methods(T &msg){
    // initialize serialized message struct
    rmw_serialized_message_t serialized_message = rmw_get_zero_initialized_serialized_message();
    auto type_support = rosidl_typesupport_cpp::get_message_type_support_handle<T>();
    auto allocator = rcl_get_default_allocator();
    rmw_serialized_message_init(&serialized_message, 0u, &allocator);

    // do the serialization
    rmw_ret_t result = rmw_serialize(&msg, type_support, &serialized_message);
    if (result != RMW_RET_OK) {
      printf("Failed to serialize message!\n");
    }

}

PYBIND11_MODULE(libpy_quintic_walk, m) {
  using namespace bitbots_quintic_walk;

  m.def("initRos", &ros2_python_extension::initRos);

  py::class_<PyWalkWrapper, std::shared_ptr<PyWalkWrapper>>(m, "PyWalkWrapper")
      .def(py::init<std::string, std::vector<py::bytes>>())
      .def("step", &PyWalkWrapper::step)
      .def("step_relative", &PyWalkWrapper::step_relative)
      .def("step_open_loop", &PyWalkWrapper::step_open_loop)
      .def("get_left_foot_pose", &PyWalkWrapper::get_left_foot_pose)
      .def("get_right_foot_pose", &PyWalkWrapper::get_right_foot_pose)
      .def("set_robot_state", &PyWalkWrapper::set_robot_state)
      .def("reset", &PyWalkWrapper::reset)
      .def("special_reset", &PyWalkWrapper::special_reset)
      .def("set_parameter", &PyWalkWrapper::set_parameter)
      .def("get_phase", &PyWalkWrapper::get_phase)
      .def("get_freq", &PyWalkWrapper::get_freq)
      .def("get_odom", &PyWalkWrapper::get_odom)
      .def("spin_some", &PyWalkWrapper::spin_some)
      .def("publish_debug", &PyWalkWrapper::publish_debug)
      .def("test_memory_leak_from", &PyWalkWrapper::test_memory_leak_from)
      .def("test_memory_leak_to", &PyWalkWrapper::test_memory_leak_to)
      .def("test_memory_leak_methods", &PyWalkWrapper::test_memory_leak_methods);
}