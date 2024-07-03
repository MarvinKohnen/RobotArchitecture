// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from turtlebot3_control_services:srv/GetLatestMap.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_CONTROL_SERVICES__SRV__DETAIL__GET_LATEST_MAP__BUILDER_HPP_
#define TURTLEBOT3_CONTROL_SERVICES__SRV__DETAIL__GET_LATEST_MAP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "turtlebot3_control_services/srv/detail/get_latest_map__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace turtlebot3_control_services
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::turtlebot3_control_services::srv::GetLatestMap_Request>()
{
  return ::turtlebot3_control_services::srv::GetLatestMap_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace turtlebot3_control_services


namespace turtlebot3_control_services
{

namespace srv
{

namespace builder
{

class Init_GetLatestMap_Response_map_path
{
public:
  Init_GetLatestMap_Response_map_path()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::turtlebot3_control_services::srv::GetLatestMap_Response map_path(::turtlebot3_control_services::srv::GetLatestMap_Response::_map_path_type arg)
  {
    msg_.map_path = std::move(arg);
    return std::move(msg_);
  }

private:
  ::turtlebot3_control_services::srv::GetLatestMap_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::turtlebot3_control_services::srv::GetLatestMap_Response>()
{
  return turtlebot3_control_services::srv::builder::Init_GetLatestMap_Response_map_path();
}

}  // namespace turtlebot3_control_services

#endif  // TURTLEBOT3_CONTROL_SERVICES__SRV__DETAIL__GET_LATEST_MAP__BUILDER_HPP_
