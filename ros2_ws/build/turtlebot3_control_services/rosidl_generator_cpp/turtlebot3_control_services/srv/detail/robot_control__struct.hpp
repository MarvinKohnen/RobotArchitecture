// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from turtlebot3_control_services:srv/RobotControl.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_CONTROL_SERVICES__SRV__DETAIL__ROBOT_CONTROL__STRUCT_HPP_
#define TURTLEBOT3_CONTROL_SERVICES__SRV__DETAIL__ROBOT_CONTROL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__turtlebot3_control_services__srv__RobotControl_Request __attribute__((deprecated))
#else
# define DEPRECATED__turtlebot3_control_services__srv__RobotControl_Request __declspec(deprecated)
#endif

namespace turtlebot3_control_services
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct RobotControl_Request_
{
  using Type = RobotControl_Request_<ContainerAllocator>;

  explicit RobotControl_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->command = "";
      this->value = 0.0;
      this->priority = 0l;
    }
  }

  explicit RobotControl_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : command(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->command = "";
      this->value = 0.0;
      this->priority = 0l;
    }
  }

  // field types and members
  using _command_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _command_type command;
  using _value_type =
    double;
  _value_type value;
  using _priority_type =
    int32_t;
  _priority_type priority;

  // setters for named parameter idiom
  Type & set__command(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->command = _arg;
    return *this;
  }
  Type & set__value(
    const double & _arg)
  {
    this->value = _arg;
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
    turtlebot3_control_services::srv::RobotControl_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const turtlebot3_control_services::srv::RobotControl_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<turtlebot3_control_services::srv::RobotControl_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<turtlebot3_control_services::srv::RobotControl_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      turtlebot3_control_services::srv::RobotControl_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_control_services::srv::RobotControl_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      turtlebot3_control_services::srv::RobotControl_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_control_services::srv::RobotControl_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<turtlebot3_control_services::srv::RobotControl_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<turtlebot3_control_services::srv::RobotControl_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__turtlebot3_control_services__srv__RobotControl_Request
    std::shared_ptr<turtlebot3_control_services::srv::RobotControl_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__turtlebot3_control_services__srv__RobotControl_Request
    std::shared_ptr<turtlebot3_control_services::srv::RobotControl_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobotControl_Request_ & other) const
  {
    if (this->command != other.command) {
      return false;
    }
    if (this->value != other.value) {
      return false;
    }
    if (this->priority != other.priority) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobotControl_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobotControl_Request_

// alias to use template instance with default allocator
using RobotControl_Request =
  turtlebot3_control_services::srv::RobotControl_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace turtlebot3_control_services


#ifndef _WIN32
# define DEPRECATED__turtlebot3_control_services__srv__RobotControl_Response __attribute__((deprecated))
#else
# define DEPRECATED__turtlebot3_control_services__srv__RobotControl_Response __declspec(deprecated)
#endif

namespace turtlebot3_control_services
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct RobotControl_Response_
{
  using Type = RobotControl_Response_<ContainerAllocator>;

  explicit RobotControl_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit RobotControl_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    turtlebot3_control_services::srv::RobotControl_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const turtlebot3_control_services::srv::RobotControl_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<turtlebot3_control_services::srv::RobotControl_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<turtlebot3_control_services::srv::RobotControl_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      turtlebot3_control_services::srv::RobotControl_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_control_services::srv::RobotControl_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      turtlebot3_control_services::srv::RobotControl_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_control_services::srv::RobotControl_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<turtlebot3_control_services::srv::RobotControl_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<turtlebot3_control_services::srv::RobotControl_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__turtlebot3_control_services__srv__RobotControl_Response
    std::shared_ptr<turtlebot3_control_services::srv::RobotControl_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__turtlebot3_control_services__srv__RobotControl_Response
    std::shared_ptr<turtlebot3_control_services::srv::RobotControl_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobotControl_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobotControl_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobotControl_Response_

// alias to use template instance with default allocator
using RobotControl_Response =
  turtlebot3_control_services::srv::RobotControl_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace turtlebot3_control_services

namespace turtlebot3_control_services
{

namespace srv
{

struct RobotControl
{
  using Request = turtlebot3_control_services::srv::RobotControl_Request;
  using Response = turtlebot3_control_services::srv::RobotControl_Response;
};

}  // namespace srv

}  // namespace turtlebot3_control_services

#endif  // TURTLEBOT3_CONTROL_SERVICES__SRV__DETAIL__ROBOT_CONTROL__STRUCT_HPP_
