// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from sound_interfaces:msg/SoundEvent.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "sound_interfaces/msg/sound_event.hpp"


#ifndef SOUND_INTERFACES__MSG__DETAIL__SOUND_EVENT__TRAITS_HPP_
#define SOUND_INTERFACES__MSG__DETAIL__SOUND_EVENT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "sound_interfaces/msg/detail/sound_event__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace sound_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const SoundEvent & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: angle
  {
    out << "angle: ";
    rosidl_generator_traits::value_to_yaml(msg.angle, out);
    out << ", ";
  }

  // member: confidence
  {
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
    out << ", ";
  }

  // member: amplitude
  {
    out << "amplitude: ";
    rosidl_generator_traits::value_to_yaml(msg.amplitude, out);
    out << ", ";
  }

  // member: is_active
  {
    out << "is_active: ";
    rosidl_generator_traits::value_to_yaml(msg.is_active, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SoundEvent & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angle: ";
    rosidl_generator_traits::value_to_yaml(msg.angle, out);
    out << "\n";
  }

  // member: confidence
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
    out << "\n";
  }

  // member: amplitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "amplitude: ";
    rosidl_generator_traits::value_to_yaml(msg.amplitude, out);
    out << "\n";
  }

  // member: is_active
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_active: ";
    rosidl_generator_traits::value_to_yaml(msg.is_active, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SoundEvent & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace sound_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use sound_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const sound_interfaces::msg::SoundEvent & msg,
  std::ostream & out, size_t indentation = 0)
{
  sound_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use sound_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const sound_interfaces::msg::SoundEvent & msg)
{
  return sound_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<sound_interfaces::msg::SoundEvent>()
{
  return "sound_interfaces::msg::SoundEvent";
}

template<>
inline const char * name<sound_interfaces::msg::SoundEvent>()
{
  return "sound_interfaces/msg/SoundEvent";
}

template<>
struct has_fixed_size<sound_interfaces::msg::SoundEvent>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<sound_interfaces::msg::SoundEvent>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<sound_interfaces::msg::SoundEvent>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SOUND_INTERFACES__MSG__DETAIL__SOUND_EVENT__TRAITS_HPP_
