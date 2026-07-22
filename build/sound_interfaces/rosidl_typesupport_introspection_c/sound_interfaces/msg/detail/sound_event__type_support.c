// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from sound_interfaces:msg/SoundEvent.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "sound_interfaces/msg/detail/sound_event__rosidl_typesupport_introspection_c.h"
#include "sound_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "sound_interfaces/msg/detail/sound_event__functions.h"
#include "sound_interfaces/msg/detail/sound_event__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void sound_interfaces__msg__SoundEvent__rosidl_typesupport_introspection_c__SoundEvent_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  sound_interfaces__msg__SoundEvent__init(message_memory);
}

void sound_interfaces__msg__SoundEvent__rosidl_typesupport_introspection_c__SoundEvent_fini_function(void * message_memory)
{
  sound_interfaces__msg__SoundEvent__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember sound_interfaces__msg__SoundEvent__rosidl_typesupport_introspection_c__SoundEvent_message_member_array[5] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sound_interfaces__msg__SoundEvent, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "angle",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sound_interfaces__msg__SoundEvent, angle),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "confidence",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sound_interfaces__msg__SoundEvent, confidence),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "amplitude",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sound_interfaces__msg__SoundEvent, amplitude),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "is_active",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sound_interfaces__msg__SoundEvent, is_active),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers sound_interfaces__msg__SoundEvent__rosidl_typesupport_introspection_c__SoundEvent_message_members = {
  "sound_interfaces__msg",  // message namespace
  "SoundEvent",  // message name
  5,  // number of fields
  sizeof(sound_interfaces__msg__SoundEvent),
  false,  // has_any_key_member_
  sound_interfaces__msg__SoundEvent__rosidl_typesupport_introspection_c__SoundEvent_message_member_array,  // message members
  sound_interfaces__msg__SoundEvent__rosidl_typesupport_introspection_c__SoundEvent_init_function,  // function to initialize message memory (memory has to be allocated)
  sound_interfaces__msg__SoundEvent__rosidl_typesupport_introspection_c__SoundEvent_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t sound_interfaces__msg__SoundEvent__rosidl_typesupport_introspection_c__SoundEvent_message_type_support_handle = {
  0,
  &sound_interfaces__msg__SoundEvent__rosidl_typesupport_introspection_c__SoundEvent_message_members,
  get_message_typesupport_handle_function,
  &sound_interfaces__msg__SoundEvent__get_type_hash,
  &sound_interfaces__msg__SoundEvent__get_type_description,
  &sound_interfaces__msg__SoundEvent__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_sound_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sound_interfaces, msg, SoundEvent)() {
  sound_interfaces__msg__SoundEvent__rosidl_typesupport_introspection_c__SoundEvent_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!sound_interfaces__msg__SoundEvent__rosidl_typesupport_introspection_c__SoundEvent_message_type_support_handle.typesupport_identifier) {
    sound_interfaces__msg__SoundEvent__rosidl_typesupport_introspection_c__SoundEvent_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &sound_interfaces__msg__SoundEvent__rosidl_typesupport_introspection_c__SoundEvent_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
