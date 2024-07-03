// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from turtlebot3_control_services:srv/GenerateHeatmap.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_CONTROL_SERVICES__SRV__DETAIL__GENERATE_HEATMAP__TRAITS_HPP_
#define TURTLEBOT3_CONTROL_SERVICES__SRV__DETAIL__GENERATE_HEATMAP__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "turtlebot3_control_services/srv/detail/generate_heatmap__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace turtlebot3_control_services
{

namespace srv
{

inline void to_flow_style_yaml(
  const GenerateHeatmap_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: coordinate_set
  {
    out << "coordinate_set: ";
    rosidl_generator_traits::value_to_yaml(msg.coordinate_set, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GenerateHeatmap_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: coordinate_set
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "coordinate_set: ";
    rosidl_generator_traits::value_to_yaml(msg.coordinate_set, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GenerateHeatmap_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace turtlebot3_control_services

namespace rosidl_generator_traits
{

[[deprecated("use turtlebot3_control_services::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const turtlebot3_control_services::srv::GenerateHeatmap_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  turtlebot3_control_services::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use turtlebot3_control_services::srv::to_yaml() instead")]]
inline std::string to_yaml(const turtlebot3_control_services::srv::GenerateHeatmap_Request & msg)
{
  return turtlebot3_control_services::srv::to_yaml(msg);
}

template<>
inline const char * data_type<turtlebot3_control_services::srv::GenerateHeatmap_Request>()
{
  return "turtlebot3_control_services::srv::GenerateHeatmap_Request";
}

template<>
inline const char * name<turtlebot3_control_services::srv::GenerateHeatmap_Request>()
{
  return "turtlebot3_control_services/srv/GenerateHeatmap_Request";
}

template<>
struct has_fixed_size<turtlebot3_control_services::srv::GenerateHeatmap_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<turtlebot3_control_services::srv::GenerateHeatmap_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<turtlebot3_control_services::srv::GenerateHeatmap_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace turtlebot3_control_services
{

namespace srv
{

inline void to_flow_style_yaml(
  const GenerateHeatmap_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GenerateHeatmap_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GenerateHeatmap_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace turtlebot3_control_services

namespace rosidl_generator_traits
{

[[deprecated("use turtlebot3_control_services::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const turtlebot3_control_services::srv::GenerateHeatmap_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  turtlebot3_control_services::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use turtlebot3_control_services::srv::to_yaml() instead")]]
inline std::string to_yaml(const turtlebot3_control_services::srv::GenerateHeatmap_Response & msg)
{
  return turtlebot3_control_services::srv::to_yaml(msg);
}

template<>
inline const char * data_type<turtlebot3_control_services::srv::GenerateHeatmap_Response>()
{
  return "turtlebot3_control_services::srv::GenerateHeatmap_Response";
}

template<>
inline const char * name<turtlebot3_control_services::srv::GenerateHeatmap_Response>()
{
  return "turtlebot3_control_services/srv/GenerateHeatmap_Response";
}

template<>
struct has_fixed_size<turtlebot3_control_services::srv::GenerateHeatmap_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<turtlebot3_control_services::srv::GenerateHeatmap_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<turtlebot3_control_services::srv::GenerateHeatmap_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<turtlebot3_control_services::srv::GenerateHeatmap>()
{
  return "turtlebot3_control_services::srv::GenerateHeatmap";
}

template<>
inline const char * name<turtlebot3_control_services::srv::GenerateHeatmap>()
{
  return "turtlebot3_control_services/srv/GenerateHeatmap";
}

template<>
struct has_fixed_size<turtlebot3_control_services::srv::GenerateHeatmap>
  : std::integral_constant<
    bool,
    has_fixed_size<turtlebot3_control_services::srv::GenerateHeatmap_Request>::value &&
    has_fixed_size<turtlebot3_control_services::srv::GenerateHeatmap_Response>::value
  >
{
};

template<>
struct has_bounded_size<turtlebot3_control_services::srv::GenerateHeatmap>
  : std::integral_constant<
    bool,
    has_bounded_size<turtlebot3_control_services::srv::GenerateHeatmap_Request>::value &&
    has_bounded_size<turtlebot3_control_services::srv::GenerateHeatmap_Response>::value
  >
{
};

template<>
struct is_service<turtlebot3_control_services::srv::GenerateHeatmap>
  : std::true_type
{
};

template<>
struct is_service_request<turtlebot3_control_services::srv::GenerateHeatmap_Request>
  : std::true_type
{
};

template<>
struct is_service_response<turtlebot3_control_services::srv::GenerateHeatmap_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // TURTLEBOT3_CONTROL_SERVICES__SRV__DETAIL__GENERATE_HEATMAP__TRAITS_HPP_
