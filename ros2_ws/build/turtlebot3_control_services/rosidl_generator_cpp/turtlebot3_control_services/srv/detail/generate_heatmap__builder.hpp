// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from turtlebot3_control_services:srv/GenerateHeatmap.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_CONTROL_SERVICES__SRV__DETAIL__GENERATE_HEATMAP__BUILDER_HPP_
#define TURTLEBOT3_CONTROL_SERVICES__SRV__DETAIL__GENERATE_HEATMAP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "turtlebot3_control_services/srv/detail/generate_heatmap__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace turtlebot3_control_services
{

namespace srv
{

namespace builder
{

class Init_GenerateHeatmap_Request_coordinate_set
{
public:
  Init_GenerateHeatmap_Request_coordinate_set()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::turtlebot3_control_services::srv::GenerateHeatmap_Request coordinate_set(::turtlebot3_control_services::srv::GenerateHeatmap_Request::_coordinate_set_type arg)
  {
    msg_.coordinate_set = std::move(arg);
    return std::move(msg_);
  }

private:
  ::turtlebot3_control_services::srv::GenerateHeatmap_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::turtlebot3_control_services::srv::GenerateHeatmap_Request>()
{
  return turtlebot3_control_services::srv::builder::Init_GenerateHeatmap_Request_coordinate_set();
}

}  // namespace turtlebot3_control_services


namespace turtlebot3_control_services
{

namespace srv
{

namespace builder
{

class Init_GenerateHeatmap_Response_success
{
public:
  Init_GenerateHeatmap_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::turtlebot3_control_services::srv::GenerateHeatmap_Response success(::turtlebot3_control_services::srv::GenerateHeatmap_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::turtlebot3_control_services::srv::GenerateHeatmap_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::turtlebot3_control_services::srv::GenerateHeatmap_Response>()
{
  return turtlebot3_control_services::srv::builder::Init_GenerateHeatmap_Response_success();
}

}  // namespace turtlebot3_control_services

#endif  // TURTLEBOT3_CONTROL_SERVICES__SRV__DETAIL__GENERATE_HEATMAP__BUILDER_HPP_
