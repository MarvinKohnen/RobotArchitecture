// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from turtlebot3_control_services:srv/RobotControl.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_CONTROL_SERVICES__SRV__DETAIL__ROBOT_CONTROL__STRUCT_H_
#define TURTLEBOT3_CONTROL_SERVICES__SRV__DETAIL__ROBOT_CONTROL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'command'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/RobotControl in the package turtlebot3_control_services.
typedef struct turtlebot3_control_services__srv__RobotControl_Request
{
  /// e.g., "move_forward", "turn_left", "turn_right", "full_stop"
  rosidl_runtime_c__String command;
  /// speed for move and turn, not used for full_stop
  double value;
  /// Priority of the command
  int32_t priority;
} turtlebot3_control_services__srv__RobotControl_Request;

// Struct for a sequence of turtlebot3_control_services__srv__RobotControl_Request.
typedef struct turtlebot3_control_services__srv__RobotControl_Request__Sequence
{
  turtlebot3_control_services__srv__RobotControl_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} turtlebot3_control_services__srv__RobotControl_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/RobotControl in the package turtlebot3_control_services.
typedef struct turtlebot3_control_services__srv__RobotControl_Response
{
  /// True if the command was executed
  bool success;
  /// Additional message, e.g., "Command executed."
  rosidl_runtime_c__String message;
} turtlebot3_control_services__srv__RobotControl_Response;

// Struct for a sequence of turtlebot3_control_services__srv__RobotControl_Response.
typedef struct turtlebot3_control_services__srv__RobotControl_Response__Sequence
{
  turtlebot3_control_services__srv__RobotControl_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} turtlebot3_control_services__srv__RobotControl_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TURTLEBOT3_CONTROL_SERVICES__SRV__DETAIL__ROBOT_CONTROL__STRUCT_H_
