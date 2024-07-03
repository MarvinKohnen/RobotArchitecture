// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from turtlebot3_control_services:srv/GetLatestMap.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_CONTROL_SERVICES__SRV__DETAIL__GET_LATEST_MAP__STRUCT_H_
#define TURTLEBOT3_CONTROL_SERVICES__SRV__DETAIL__GET_LATEST_MAP__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetLatestMap in the package turtlebot3_control_services.
typedef struct turtlebot3_control_services__srv__GetLatestMap_Request
{
  uint8_t structure_needs_at_least_one_member;
} turtlebot3_control_services__srv__GetLatestMap_Request;

// Struct for a sequence of turtlebot3_control_services__srv__GetLatestMap_Request.
typedef struct turtlebot3_control_services__srv__GetLatestMap_Request__Sequence
{
  turtlebot3_control_services__srv__GetLatestMap_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} turtlebot3_control_services__srv__GetLatestMap_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'map_path'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GetLatestMap in the package turtlebot3_control_services.
typedef struct turtlebot3_control_services__srv__GetLatestMap_Response
{
  rosidl_runtime_c__String map_path;
} turtlebot3_control_services__srv__GetLatestMap_Response;

// Struct for a sequence of turtlebot3_control_services__srv__GetLatestMap_Response.
typedef struct turtlebot3_control_services__srv__GetLatestMap_Response__Sequence
{
  turtlebot3_control_services__srv__GetLatestMap_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} turtlebot3_control_services__srv__GetLatestMap_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TURTLEBOT3_CONTROL_SERVICES__SRV__DETAIL__GET_LATEST_MAP__STRUCT_H_
