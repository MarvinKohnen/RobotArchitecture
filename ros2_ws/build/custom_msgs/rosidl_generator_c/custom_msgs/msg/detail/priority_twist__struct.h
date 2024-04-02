// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_msgs:msg/PriorityTwist.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__PRIORITY_TWIST__STRUCT_H_
#define CUSTOM_MSGS__MSG__DETAIL__PRIORITY_TWIST__STRUCT_H_

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
#include "geometry_msgs/msg/detail/twist__struct.h"

/// Struct defined in msg/PriorityTwist in the package custom_msgs.
typedef struct custom_msgs__msg__PriorityTwist
{
  geometry_msgs__msg__Twist command;
  int32_t priority;
} custom_msgs__msg__PriorityTwist;

// Struct for a sequence of custom_msgs__msg__PriorityTwist.
typedef struct custom_msgs__msg__PriorityTwist__Sequence
{
  custom_msgs__msg__PriorityTwist * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_msgs__msg__PriorityTwist__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MSGS__MSG__DETAIL__PRIORITY_TWIST__STRUCT_H_
