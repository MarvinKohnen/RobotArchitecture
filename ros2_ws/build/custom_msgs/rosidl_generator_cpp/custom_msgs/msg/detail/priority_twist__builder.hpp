// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msgs:msg/PriorityTwist.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__PRIORITY_TWIST__BUILDER_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__PRIORITY_TWIST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msgs/msg/detail/priority_twist__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msgs
{

namespace msg
{

namespace builder
{

class Init_PriorityTwist_priority
{
public:
  explicit Init_PriorityTwist_priority(::custom_msgs::msg::PriorityTwist & msg)
  : msg_(msg)
  {}
  ::custom_msgs::msg::PriorityTwist priority(::custom_msgs::msg::PriorityTwist::_priority_type arg)
  {
    msg_.priority = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msgs::msg::PriorityTwist msg_;
};

class Init_PriorityTwist_command
{
public:
  Init_PriorityTwist_command()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PriorityTwist_priority command(::custom_msgs::msg::PriorityTwist::_command_type arg)
  {
    msg_.command = std::move(arg);
    return Init_PriorityTwist_priority(msg_);
  }

private:
  ::custom_msgs::msg::PriorityTwist msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msgs::msg::PriorityTwist>()
{
  return custom_msgs::msg::builder::Init_PriorityTwist_command();
}

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__PRIORITY_TWIST__BUILDER_HPP_
