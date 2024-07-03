// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from turtlebot3_control_services:srv/GenerateHeatmap.idl
// generated code does not contain a copyright notice
#include "turtlebot3_control_services/srv/detail/generate_heatmap__rosidl_typesupport_fastrtps_cpp.hpp"
#include "turtlebot3_control_services/srv/detail/generate_heatmap__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace turtlebot3_control_services
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_turtlebot3_control_services
cdr_serialize(
  const turtlebot3_control_services::srv::GenerateHeatmap_Request & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: coordinate_set
  cdr << ros_message.coordinate_set;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_turtlebot3_control_services
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  turtlebot3_control_services::srv::GenerateHeatmap_Request & ros_message)
{
  // Member: coordinate_set
  cdr >> ros_message.coordinate_set;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_turtlebot3_control_services
get_serialized_size(
  const turtlebot3_control_services::srv::GenerateHeatmap_Request & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: coordinate_set
  {
    size_t item_size = sizeof(ros_message.coordinate_set);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_turtlebot3_control_services
max_serialized_size_GenerateHeatmap_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: coordinate_set
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = turtlebot3_control_services::srv::GenerateHeatmap_Request;
    is_plain =
      (
      offsetof(DataType, coordinate_set) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _GenerateHeatmap_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const turtlebot3_control_services::srv::GenerateHeatmap_Request *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _GenerateHeatmap_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<turtlebot3_control_services::srv::GenerateHeatmap_Request *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _GenerateHeatmap_Request__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const turtlebot3_control_services::srv::GenerateHeatmap_Request *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _GenerateHeatmap_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_GenerateHeatmap_Request(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _GenerateHeatmap_Request__callbacks = {
  "turtlebot3_control_services::srv",
  "GenerateHeatmap_Request",
  _GenerateHeatmap_Request__cdr_serialize,
  _GenerateHeatmap_Request__cdr_deserialize,
  _GenerateHeatmap_Request__get_serialized_size,
  _GenerateHeatmap_Request__max_serialized_size
};

static rosidl_message_type_support_t _GenerateHeatmap_Request__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_GenerateHeatmap_Request__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace turtlebot3_control_services

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_turtlebot3_control_services
const rosidl_message_type_support_t *
get_message_type_support_handle<turtlebot3_control_services::srv::GenerateHeatmap_Request>()
{
  return &turtlebot3_control_services::srv::typesupport_fastrtps_cpp::_GenerateHeatmap_Request__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, turtlebot3_control_services, srv, GenerateHeatmap_Request)() {
  return &turtlebot3_control_services::srv::typesupport_fastrtps_cpp::_GenerateHeatmap_Request__handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include <limits>
// already included above
// #include <stdexcept>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
// already included above
// #include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace turtlebot3_control_services
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_turtlebot3_control_services
cdr_serialize(
  const turtlebot3_control_services::srv::GenerateHeatmap_Response & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: success
  cdr << (ros_message.success ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_turtlebot3_control_services
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  turtlebot3_control_services::srv::GenerateHeatmap_Response & ros_message)
{
  // Member: success
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.success = tmp ? true : false;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_turtlebot3_control_services
get_serialized_size(
  const turtlebot3_control_services::srv::GenerateHeatmap_Response & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: success
  {
    size_t item_size = sizeof(ros_message.success);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_turtlebot3_control_services
max_serialized_size_GenerateHeatmap_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: success
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = turtlebot3_control_services::srv::GenerateHeatmap_Response;
    is_plain =
      (
      offsetof(DataType, success) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _GenerateHeatmap_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const turtlebot3_control_services::srv::GenerateHeatmap_Response *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _GenerateHeatmap_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<turtlebot3_control_services::srv::GenerateHeatmap_Response *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _GenerateHeatmap_Response__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const turtlebot3_control_services::srv::GenerateHeatmap_Response *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _GenerateHeatmap_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_GenerateHeatmap_Response(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _GenerateHeatmap_Response__callbacks = {
  "turtlebot3_control_services::srv",
  "GenerateHeatmap_Response",
  _GenerateHeatmap_Response__cdr_serialize,
  _GenerateHeatmap_Response__cdr_deserialize,
  _GenerateHeatmap_Response__get_serialized_size,
  _GenerateHeatmap_Response__max_serialized_size
};

static rosidl_message_type_support_t _GenerateHeatmap_Response__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_GenerateHeatmap_Response__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace turtlebot3_control_services

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_turtlebot3_control_services
const rosidl_message_type_support_t *
get_message_type_support_handle<turtlebot3_control_services::srv::GenerateHeatmap_Response>()
{
  return &turtlebot3_control_services::srv::typesupport_fastrtps_cpp::_GenerateHeatmap_Response__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, turtlebot3_control_services, srv, GenerateHeatmap_Response)() {
  return &turtlebot3_control_services::srv::typesupport_fastrtps_cpp::_GenerateHeatmap_Response__handle;
}

#ifdef __cplusplus
}
#endif

#include "rmw/error_handling.h"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/service_type_support_decl.hpp"

namespace turtlebot3_control_services
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

static service_type_support_callbacks_t _GenerateHeatmap__callbacks = {
  "turtlebot3_control_services::srv",
  "GenerateHeatmap",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, turtlebot3_control_services, srv, GenerateHeatmap_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, turtlebot3_control_services, srv, GenerateHeatmap_Response)(),
};

static rosidl_service_type_support_t _GenerateHeatmap__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_GenerateHeatmap__callbacks,
  get_service_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace turtlebot3_control_services

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_turtlebot3_control_services
const rosidl_service_type_support_t *
get_service_type_support_handle<turtlebot3_control_services::srv::GenerateHeatmap>()
{
  return &turtlebot3_control_services::srv::typesupport_fastrtps_cpp::_GenerateHeatmap__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, turtlebot3_control_services, srv, GenerateHeatmap)() {
  return &turtlebot3_control_services::srv::typesupport_fastrtps_cpp::_GenerateHeatmap__handle;
}

#ifdef __cplusplus
}
#endif
