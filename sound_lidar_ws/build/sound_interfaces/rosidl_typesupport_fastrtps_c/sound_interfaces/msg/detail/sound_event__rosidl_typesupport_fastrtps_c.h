// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from sound_interfaces:msg/SoundEvent.idl
// generated code does not contain a copyright notice
#ifndef SOUND_INTERFACES__MSG__DETAIL__SOUND_EVENT__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define SOUND_INTERFACES__MSG__DETAIL__SOUND_EVENT__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "sound_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "sound_interfaces/msg/detail/sound_event__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sound_interfaces
bool cdr_serialize_sound_interfaces__msg__SoundEvent(
  const sound_interfaces__msg__SoundEvent * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sound_interfaces
bool cdr_deserialize_sound_interfaces__msg__SoundEvent(
  eprosima::fastcdr::Cdr &,
  sound_interfaces__msg__SoundEvent * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sound_interfaces
size_t get_serialized_size_sound_interfaces__msg__SoundEvent(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sound_interfaces
size_t max_serialized_size_sound_interfaces__msg__SoundEvent(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sound_interfaces
bool cdr_serialize_key_sound_interfaces__msg__SoundEvent(
  const sound_interfaces__msg__SoundEvent * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sound_interfaces
size_t get_serialized_size_key_sound_interfaces__msg__SoundEvent(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sound_interfaces
size_t max_serialized_size_key_sound_interfaces__msg__SoundEvent(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sound_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, sound_interfaces, msg, SoundEvent)();

#ifdef __cplusplus
}
#endif

#endif  // SOUND_INTERFACES__MSG__DETAIL__SOUND_EVENT__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
