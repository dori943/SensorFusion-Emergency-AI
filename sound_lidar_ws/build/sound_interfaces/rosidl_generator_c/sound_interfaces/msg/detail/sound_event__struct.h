// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sound_interfaces:msg/SoundEvent.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "sound_interfaces/msg/sound_event.h"


#ifndef SOUND_INTERFACES__MSG__DETAIL__SOUND_EVENT__STRUCT_H_
#define SOUND_INTERFACES__MSG__DETAIL__SOUND_EVENT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/SoundEvent in the package sound_interfaces.
/**
  * sound_interfaces/msg/SoundEvent.msg
 */
typedef struct sound_interfaces__msg__SoundEvent
{
  /// 타임스탬프 포함
  std_msgs__msg__Header header;
  /// 소리 방향 (도, -180 ~ 180)
  float angle;
  /// 감지 신뢰도 (0.0 ~ 1.0)
  float confidence;
  /// 소리 크기 (dB)
  float amplitude;
  /// 소리 감지 여부
  bool is_active;
} sound_interfaces__msg__SoundEvent;

// Struct for a sequence of sound_interfaces__msg__SoundEvent.
typedef struct sound_interfaces__msg__SoundEvent__Sequence
{
  sound_interfaces__msg__SoundEvent * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sound_interfaces__msg__SoundEvent__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SOUND_INTERFACES__MSG__DETAIL__SOUND_EVENT__STRUCT_H_
