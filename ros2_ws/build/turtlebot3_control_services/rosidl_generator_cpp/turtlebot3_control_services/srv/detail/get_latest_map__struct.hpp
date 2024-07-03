// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from turtlebot3_control_services:srv/GetLatestMap.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_CONTROL_SERVICES__SRV__DETAIL__GET_LATEST_MAP__STRUCT_HPP_
#define TURTLEBOT3_CONTROL_SERVICES__SRV__DETAIL__GET_LATEST_MAP__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__turtlebot3_control_services__srv__GetLatestMap_Request __attribute__((deprecated))
#else
# define DEPRECATED__turtlebot3_control_services__srv__GetLatestMap_Request __declspec(deprecated)
#endif

namespace turtlebot3_control_services
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetLatestMap_Request_
{
  using Type = GetLatestMap_Request_<ContainerAllocator>;

  explicit GetLatestMap_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit GetLatestMap_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    turtlebot3_control_services::srv::GetLatestMap_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const turtlebot3_control_services::srv::GetLatestMap_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<turtlebot3_control_services::srv::GetLatestMap_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<turtlebot3_control_services::srv::GetLatestMap_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      turtlebot3_control_services::srv::GetLatestMap_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_control_services::srv::GetLatestMap_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      turtlebot3_control_services::srv::GetLatestMap_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_control_services::srv::GetLatestMap_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<turtlebot3_control_services::srv::GetLatestMap_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<turtlebot3_control_services::srv::GetLatestMap_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__turtlebot3_control_services__srv__GetLatestMap_Request
    std::shared_ptr<turtlebot3_control_services::srv::GetLatestMap_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__turtlebot3_control_services__srv__GetLatestMap_Request
    std::shared_ptr<turtlebot3_control_services::srv::GetLatestMap_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetLatestMap_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetLatestMap_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetLatestMap_Request_

// alias to use template instance with default allocator
using GetLatestMap_Request =
  turtlebot3_control_services::srv::GetLatestMap_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace turtlebot3_control_services


#ifndef _WIN32
# define DEPRECATED__turtlebot3_control_services__srv__GetLatestMap_Response __attribute__((deprecated))
#else
# define DEPRECATED__turtlebot3_control_services__srv__GetLatestMap_Response __declspec(deprecated)
#endif

namespace turtlebot3_control_services
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetLatestMap_Response_
{
  using Type = GetLatestMap_Response_<ContainerAllocator>;

  explicit GetLatestMap_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->map_path = "";
    }
  }

  explicit GetLatestMap_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : map_path(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->map_path = "";
    }
  }

  // field types and members
  using _map_path_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _map_path_type map_path;

  // setters for named parameter idiom
  Type & set__map_path(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->map_path = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    turtlebot3_control_services::srv::GetLatestMap_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const turtlebot3_control_services::srv::GetLatestMap_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<turtlebot3_control_services::srv::GetLatestMap_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<turtlebot3_control_services::srv::GetLatestMap_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      turtlebot3_control_services::srv::GetLatestMap_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_control_services::srv::GetLatestMap_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      turtlebot3_control_services::srv::GetLatestMap_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_control_services::srv::GetLatestMap_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<turtlebot3_control_services::srv::GetLatestMap_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<turtlebot3_control_services::srv::GetLatestMap_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__turtlebot3_control_services__srv__GetLatestMap_Response
    std::shared_ptr<turtlebot3_control_services::srv::GetLatestMap_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__turtlebot3_control_services__srv__GetLatestMap_Response
    std::shared_ptr<turtlebot3_control_services::srv::GetLatestMap_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetLatestMap_Response_ & other) const
  {
    if (this->map_path != other.map_path) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetLatestMap_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetLatestMap_Response_

// alias to use template instance with default allocator
using GetLatestMap_Response =
  turtlebot3_control_services::srv::GetLatestMap_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace turtlebot3_control_services

namespace turtlebot3_control_services
{

namespace srv
{

struct GetLatestMap
{
  using Request = turtlebot3_control_services::srv::GetLatestMap_Request;
  using Response = turtlebot3_control_services::srv::GetLatestMap_Response;
};

}  // namespace srv

}  // namespace turtlebot3_control_services

#endif  // TURTLEBOT3_CONTROL_SERVICES__SRV__DETAIL__GET_LATEST_MAP__STRUCT_HPP_
