// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from turtlebot3_control_services:srv/GenerateHeatmap.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_CONTROL_SERVICES__SRV__DETAIL__GENERATE_HEATMAP__STRUCT_H_
#define TURTLEBOT3_CONTROL_SERVICES__SRV__DETAIL__GENERATE_HEATMAP__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GenerateHeatmap in the package turtlebot3_control_services.
typedef struct turtlebot3_control_services__srv__GenerateHeatmap_Request
{
  /// The index of the coordinate set
  int32_t coordinate_set;
} turtlebot3_control_services__srv__GenerateHeatmap_Request;

// Struct for a sequence of turtlebot3_control_services__srv__GenerateHeatmap_Request.
typedef struct turtlebot3_control_services__srv__GenerateHeatmap_Request__Sequence
{
  turtlebot3_control_services__srv__GenerateHeatmap_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} turtlebot3_control_services__srv__GenerateHeatmap_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/GenerateHeatmap in the package turtlebot3_control_services.
typedef struct turtlebot3_control_services__srv__GenerateHeatmap_Response
{
  /// True if the heatmap was generated successfully
  bool success;
} turtlebot3_control_services__srv__GenerateHeatmap_Response;

// Struct for a sequence of turtlebot3_control_services__srv__GenerateHeatmap_Response.
typedef struct turtlebot3_control_services__srv__GenerateHeatmap_Response__Sequence
{
  turtlebot3_control_services__srv__GenerateHeatmap_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} turtlebot3_control_services__srv__GenerateHeatmap_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TURTLEBOT3_CONTROL_SERVICES__SRV__DETAIL__GENERATE_HEATMAP__STRUCT_H_
