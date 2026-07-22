// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from sound_interfaces:msg/SoundEvent.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "sound_interfaces/msg/sound_event.hpp"


#ifndef SOUND_INTERFACES__MSG__DETAIL__SOUND_EVENT__BUILDER_HPP_
#define SOUND_INTERFACES__MSG__DETAIL__SOUND_EVENT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "sound_interfaces/msg/detail/sound_event__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace sound_interfaces
{

namespace msg
{

namespace builder
{

class Init_SoundEvent_is_active
{
public:
  explicit Init_SoundEvent_is_active(::sound_interfaces::msg::SoundEvent & msg)
  : msg_(msg)
  {}
  ::sound_interfaces::msg::SoundEvent is_active(::sound_interfaces::msg::SoundEvent::_is_active_type arg)
  {
    msg_.is_active = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sound_interfaces::msg::SoundEvent msg_;
};

class Init_SoundEvent_amplitude
{
public:
  explicit Init_SoundEvent_amplitude(::sound_interfaces::msg::SoundEvent & msg)
  : msg_(msg)
  {}
  Init_SoundEvent_is_active amplitude(::sound_interfaces::msg::SoundEvent::_amplitude_type arg)
  {
    msg_.amplitude = std::move(arg);
    return Init_SoundEvent_is_active(msg_);
  }

private:
  ::sound_interfaces::msg::SoundEvent msg_;
};

class Init_SoundEvent_confidence
{
public:
  explicit Init_SoundEvent_confidence(::sound_interfaces::msg::SoundEvent & msg)
  : msg_(msg)
  {}
  Init_SoundEvent_amplitude confidence(::sound_interfaces::msg::SoundEvent::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_SoundEvent_amplitude(msg_);
  }

private:
  ::sound_interfaces::msg::SoundEvent msg_;
};

class Init_SoundEvent_angle
{
public:
  explicit Init_SoundEvent_angle(::sound_interfaces::msg::SoundEvent & msg)
  : msg_(msg)
  {}
  Init_SoundEvent_confidence angle(::sound_interfaces::msg::SoundEvent::_angle_type arg)
  {
    msg_.angle = std::move(arg);
    return Init_SoundEvent_confidence(msg_);
  }

private:
  ::sound_interfaces::msg::SoundEvent msg_;
};

class Init_SoundEvent_header
{
public:
  Init_SoundEvent_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SoundEvent_angle header(::sound_interfaces::msg::SoundEvent::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_SoundEvent_angle(msg_);
  }

private:
  ::sound_interfaces::msg::SoundEvent msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::sound_interfaces::msg::SoundEvent>()
{
  return sound_interfaces::msg::builder::Init_SoundEvent_header();
}

}  // namespace sound_interfaces

#endif  // SOUND_INTERFACES__MSG__DETAIL__SOUND_EVENT__BUILDER_HPP_
