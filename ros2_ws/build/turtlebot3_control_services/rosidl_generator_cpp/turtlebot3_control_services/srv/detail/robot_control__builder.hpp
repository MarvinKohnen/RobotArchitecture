// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from turtlebot3_control_services:srv/RobotControl.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_CONTROL_SERVICES__SRV__DETAIL__ROBOT_CONTROL__BUILDER_HPP_
#define TURTLEBOT3_CONTROL_SERVICES__SRV__DETAIL__ROBOT_CONTROL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "turtlebot3_control_services/srv/detail/robot_control__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace turtlebot3_control_services
{

namespace srv
{

namespace builder
{

class Init_RobotControl_Request_priority
{
public:
  explicit Init_RobotControl_Request_priority(::turtlebot3_control_services::srv::RobotControl_Request & msg)
  : msg_(msg)
  {}
  ::turtlebot3_control_services::srv::RobotControl_Request priority(::turtlebot3_control_services::srv::RobotControl_Request::_priority_type arg)
  {
    msg_.priority = std::move(arg);
    return std::move(msg_);
  }

private:
  ::turtlebot3_control_services::srv::RobotControl_Request msg_;
};

class Init_RobotControl_Request_value
{
public:
  explicit Init_RobotControl_Request_value(::turtlebot3_control_services::srv::RobotControl_Request & msg)
  : msg_(msg)
  {}
  Init_RobotControl_Request_priority value(::turtlebot3_control_services::srv::RobotControl_Request::_value_type arg)
  {
    msg_.value = std::move(arg);
    return Init_RobotControl_Request_priority(msg_);
  }

private:
  ::turtlebot3_control_services::srv::RobotControl_Request msg_;
};

class Init_RobotControl_Request_command
{
public:
  Init_RobotControl_Request_command()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotControl_Request_value command(::turtlebot3_control_services::srv::RobotControl_Request::_command_type arg)
  {
    msg_.command = std::move(arg);
    return Init_RobotControl_Request_value(msg_);
  }

private:
  ::turtlebot3_control_services::srv::RobotControl_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::turtlebot3_control_services::srv::RobotControl_Request>()
{
  return turtlebot3_control_services::srv::builder::Init_RobotControl_Request_command();
}

}  // namespace turtlebot3_control_services


namespace turtlebot3_control_services
{

namespace srv
{

namespace builder
{

class Init_RobotControl_Response_message
{
public:
  explicit Init_RobotControl_Response_message(::turtlebot3_control_services::srv::RobotControl_Response & msg)
  : msg_(msg)
  {}
  ::turtlebot3_control_services::srv::RobotControl_Response message(::turtlebot3_control_services::srv::RobotControl_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::turtlebot3_control_services::srv::RobotControl_Response msg_;
};

class Init_RobotControl_Response_success
{
public:
  Init_RobotControl_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotControl_Response_message success(::turtlebot3_control_services::srv::RobotControl_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_RobotControl_Response_message(msg_);
  }

private:
  ::turtlebot3_control_services::srv::RobotControl_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::turtlebot3_control_services::srv::RobotControl_Response>()
{
  return turtlebot3_control_services::srv::builder::Init_RobotControl_Response_success();
}

}  // namespace turtlebot3_control_services

#endif  // TURTLEBOT3_CONTROL_SERVICES__SRV__DETAIL__ROBOT_CONTROL__BUILDER_HPP_
