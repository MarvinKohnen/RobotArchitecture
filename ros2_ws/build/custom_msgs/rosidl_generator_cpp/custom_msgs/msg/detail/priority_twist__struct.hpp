// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_msgs:msg/PriorityTwist.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__PRIORITY_TWIST__STRUCT_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__PRIORITY_TWIST__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'command'
#include "geometry_msgs/msg/detail/twist__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__custom_msgs__msg__PriorityTwist __attribute__((deprecated))
#else
# define DEPRECATED__custom_msgs__msg__PriorityTwist __declspec(deprecated)
#endif

namespace custom_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PriorityTwist_
{
  using Type = PriorityTwist_<ContainerAllocator>;

  explicit PriorityTwist_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : command(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->priority = 0l;
    }
  }

  explicit PriorityTwist_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : command(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->priority = 0l;
    }
  }

  // field types and members
  using _command_type =
    geometry_msgs::msg::Twist_<ContainerAllocator>;
  _command_type command;
  using _priority_type =
    int32_t;
  _priority_type priority;

  // setters for named parameter idiom
  Type & set__command(
    const geometry_msgs::msg::Twist_<ContainerAllocator> & _arg)
  {
    this->command = _arg;
    return *this;
  }
  Type & set__priority(
    const int32_t & _arg)
  {
    this->priority = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_msgs::msg::PriorityTwist_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_msgs::msg::PriorityTwist_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_msgs::msg::PriorityTwist_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_msgs::msg::PriorityTwist_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::PriorityTwist_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::PriorityTwist_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::PriorityTwist_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::PriorityTwist_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_msgs::msg::PriorityTwist_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_msgs::msg::PriorityTwist_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_msgs__msg__PriorityTwist
    std::shared_ptr<custom_msgs::msg::PriorityTwist_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_msgs__msg__PriorityTwist
    std::shared_ptr<custom_msgs::msg::PriorityTwist_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PriorityTwist_ & other) const
  {
    if (this->command != other.command) {
      return false;
    }
    if (this->priority != other.priority) {
      return false;
    }
    return true;
  }
  bool operator!=(const PriorityTwist_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PriorityTwist_

// alias to use template instance with default allocator
using PriorityTwist =
  custom_msgs::msg::PriorityTwist_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__PRIORITY_TWIST__STRUCT_HPP_
