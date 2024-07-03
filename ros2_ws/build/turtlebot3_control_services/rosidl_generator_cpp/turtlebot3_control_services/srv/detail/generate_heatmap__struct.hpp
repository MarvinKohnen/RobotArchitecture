// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from turtlebot3_control_services:srv/GenerateHeatmap.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_CONTROL_SERVICES__SRV__DETAIL__GENERATE_HEATMAP__STRUCT_HPP_
#define TURTLEBOT3_CONTROL_SERVICES__SRV__DETAIL__GENERATE_HEATMAP__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__turtlebot3_control_services__srv__GenerateHeatmap_Request __attribute__((deprecated))
#else
# define DEPRECATED__turtlebot3_control_services__srv__GenerateHeatmap_Request __declspec(deprecated)
#endif

namespace turtlebot3_control_services
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GenerateHeatmap_Request_
{
  using Type = GenerateHeatmap_Request_<ContainerAllocator>;

  explicit GenerateHeatmap_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->coordinate_set = 0l;
    }
  }

  explicit GenerateHeatmap_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->coordinate_set = 0l;
    }
  }

  // field types and members
  using _coordinate_set_type =
    int32_t;
  _coordinate_set_type coordinate_set;

  // setters for named parameter idiom
  Type & set__coordinate_set(
    const int32_t & _arg)
  {
    this->coordinate_set = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    turtlebot3_control_services::srv::GenerateHeatmap_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const turtlebot3_control_services::srv::GenerateHeatmap_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<turtlebot3_control_services::srv::GenerateHeatmap_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<turtlebot3_control_services::srv::GenerateHeatmap_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      turtlebot3_control_services::srv::GenerateHeatmap_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_control_services::srv::GenerateHeatmap_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      turtlebot3_control_services::srv::GenerateHeatmap_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_control_services::srv::GenerateHeatmap_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<turtlebot3_control_services::srv::GenerateHeatmap_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<turtlebot3_control_services::srv::GenerateHeatmap_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__turtlebot3_control_services__srv__GenerateHeatmap_Request
    std::shared_ptr<turtlebot3_control_services::srv::GenerateHeatmap_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__turtlebot3_control_services__srv__GenerateHeatmap_Request
    std::shared_ptr<turtlebot3_control_services::srv::GenerateHeatmap_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GenerateHeatmap_Request_ & other) const
  {
    if (this->coordinate_set != other.coordinate_set) {
      return false;
    }
    return true;
  }
  bool operator!=(const GenerateHeatmap_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GenerateHeatmap_Request_

// alias to use template instance with default allocator
using GenerateHeatmap_Request =
  turtlebot3_control_services::srv::GenerateHeatmap_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace turtlebot3_control_services


#ifndef _WIN32
# define DEPRECATED__turtlebot3_control_services__srv__GenerateHeatmap_Response __attribute__((deprecated))
#else
# define DEPRECATED__turtlebot3_control_services__srv__GenerateHeatmap_Response __declspec(deprecated)
#endif

namespace turtlebot3_control_services
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GenerateHeatmap_Response_
{
  using Type = GenerateHeatmap_Response_<ContainerAllocator>;

  explicit GenerateHeatmap_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit GenerateHeatmap_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    turtlebot3_control_services::srv::GenerateHeatmap_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const turtlebot3_control_services::srv::GenerateHeatmap_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<turtlebot3_control_services::srv::GenerateHeatmap_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<turtlebot3_control_services::srv::GenerateHeatmap_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      turtlebot3_control_services::srv::GenerateHeatmap_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_control_services::srv::GenerateHeatmap_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      turtlebot3_control_services::srv::GenerateHeatmap_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_control_services::srv::GenerateHeatmap_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<turtlebot3_control_services::srv::GenerateHeatmap_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<turtlebot3_control_services::srv::GenerateHeatmap_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__turtlebot3_control_services__srv__GenerateHeatmap_Response
    std::shared_ptr<turtlebot3_control_services::srv::GenerateHeatmap_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__turtlebot3_control_services__srv__GenerateHeatmap_Response
    std::shared_ptr<turtlebot3_control_services::srv::GenerateHeatmap_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GenerateHeatmap_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const GenerateHeatmap_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GenerateHeatmap_Response_

// alias to use template instance with default allocator
using GenerateHeatmap_Response =
  turtlebot3_control_services::srv::GenerateHeatmap_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace turtlebot3_control_services

namespace turtlebot3_control_services
{

namespace srv
{

struct GenerateHeatmap
{
  using Request = turtlebot3_control_services::srv::GenerateHeatmap_Request;
  using Response = turtlebot3_control_services::srv::GenerateHeatmap_Response;
};

}  // namespace srv

}  // namespace turtlebot3_control_services

#endif  // TURTLEBOT3_CONTROL_SERVICES__SRV__DETAIL__GENERATE_HEATMAP__STRUCT_HPP_
