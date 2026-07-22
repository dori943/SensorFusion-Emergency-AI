// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sound_interfaces:msg/SoundEvent.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "sound_interfaces/msg/sound_event.hpp"


#ifndef SOUND_INTERFACES__MSG__DETAIL__SOUND_EVENT__STRUCT_HPP_
#define SOUND_INTERFACES__MSG__DETAIL__SOUND_EVENT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__sound_interfaces__msg__SoundEvent __attribute__((deprecated))
#else
# define DEPRECATED__sound_interfaces__msg__SoundEvent __declspec(deprecated)
#endif

namespace sound_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SoundEvent_
{
  using Type = SoundEvent_<ContainerAllocator>;

  explicit SoundEvent_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->angle = 0.0f;
      this->confidence = 0.0f;
      this->amplitude = 0.0f;
      this->is_active = false;
    }
  }

  explicit SoundEvent_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->angle = 0.0f;
      this->confidence = 0.0f;
      this->amplitude = 0.0f;
      this->is_active = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _angle_type =
    float;
  _angle_type angle;
  using _confidence_type =
    float;
  _confidence_type confidence;
  using _amplitude_type =
    float;
  _amplitude_type amplitude;
  using _is_active_type =
    bool;
  _is_active_type is_active;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__angle(
    const float & _arg)
  {
    this->angle = _arg;
    return *this;
  }
  Type & set__confidence(
    const float & _arg)
  {
    this->confidence = _arg;
    return *this;
  }
  Type & set__amplitude(
    const float & _arg)
  {
    this->amplitude = _arg;
    return *this;
  }
  Type & set__is_active(
    const bool & _arg)
  {
    this->is_active = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sound_interfaces::msg::SoundEvent_<ContainerAllocator> *;
  using ConstRawPtr =
    const sound_interfaces::msg::SoundEvent_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sound_interfaces::msg::SoundEvent_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sound_interfaces::msg::SoundEvent_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sound_interfaces::msg::SoundEvent_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sound_interfaces::msg::SoundEvent_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sound_interfaces::msg::SoundEvent_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sound_interfaces::msg::SoundEvent_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sound_interfaces::msg::SoundEvent_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sound_interfaces::msg::SoundEvent_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sound_interfaces__msg__SoundEvent
    std::shared_ptr<sound_interfaces::msg::SoundEvent_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sound_interfaces__msg__SoundEvent
    std::shared_ptr<sound_interfaces::msg::SoundEvent_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SoundEvent_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->angle != other.angle) {
      return false;
    }
    if (this->confidence != other.confidence) {
      return false;
    }
    if (this->amplitude != other.amplitude) {
      return false;
    }
    if (this->is_active != other.is_active) {
      return false;
    }
    return true;
  }
  bool operator!=(const SoundEvent_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SoundEvent_

// alias to use template instance with default allocator
using SoundEvent =
  sound_interfaces::msg::SoundEvent_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sound_interfaces

#endif  // SOUND_INTERFACES__MSG__DETAIL__SOUND_EVENT__STRUCT_HPP_
