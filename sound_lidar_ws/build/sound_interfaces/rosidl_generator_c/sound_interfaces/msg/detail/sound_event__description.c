// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from sound_interfaces:msg/SoundEvent.idl
// generated code does not contain a copyright notice

#include "sound_interfaces/msg/detail/sound_event__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_sound_interfaces
const rosidl_type_hash_t *
sound_interfaces__msg__SoundEvent__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xc5, 0x67, 0x93, 0x39, 0xb3, 0x8b, 0x76, 0x4e,
      0x8d, 0x79, 0x01, 0x6b, 0xf7, 0x2c, 0x93, 0x81,
      0x6a, 0xfe, 0x36, 0xa4, 0x73, 0x21, 0x15, 0x55,
      0xf0, 0x98, 0x17, 0x8f, 0x47, 0x62, 0x53, 0x52,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "std_msgs/msg/detail/header__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char sound_interfaces__msg__SoundEvent__TYPE_NAME[] = "sound_interfaces/msg/SoundEvent";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char sound_interfaces__msg__SoundEvent__FIELD_NAME__header[] = "header";
static char sound_interfaces__msg__SoundEvent__FIELD_NAME__angle[] = "angle";
static char sound_interfaces__msg__SoundEvent__FIELD_NAME__confidence[] = "confidence";
static char sound_interfaces__msg__SoundEvent__FIELD_NAME__amplitude[] = "amplitude";
static char sound_interfaces__msg__SoundEvent__FIELD_NAME__is_active[] = "is_active";

static rosidl_runtime_c__type_description__Field sound_interfaces__msg__SoundEvent__FIELDS[] = {
  {
    {sound_interfaces__msg__SoundEvent__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {sound_interfaces__msg__SoundEvent__FIELD_NAME__angle, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {sound_interfaces__msg__SoundEvent__FIELD_NAME__confidence, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {sound_interfaces__msg__SoundEvent__FIELD_NAME__amplitude, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {sound_interfaces__msg__SoundEvent__FIELD_NAME__is_active, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription sound_interfaces__msg__SoundEvent__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
sound_interfaces__msg__SoundEvent__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {sound_interfaces__msg__SoundEvent__TYPE_NAME, 31, 31},
      {sound_interfaces__msg__SoundEvent__FIELDS, 5, 5},
    },
    {sound_interfaces__msg__SoundEvent__REFERENCED_TYPE_DESCRIPTIONS, 2, 2},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# sound_interfaces/msg/SoundEvent.msg\n"
  "std_msgs/Header header     # \\xed\\x83\\x80\\xec\\x9e\\x84\\xec\\x8a\\xa4\\xed\\x83\\xac\\xed\\x94\\x84 \\xed\\x8f\\xac\\xed\\x95\\xa8\n"
  "float32 angle              # \\xec\\x86\\x8c\\xeb\\xa6\\xac \\xeb\\xb0\\xa9\\xed\\x96\\xa5 (\\xeb\\x8f\\x84, -180 ~ 180)\n"
  "float32 confidence         # \\xea\\xb0\\x90\\xec\\xa7\\x80 \\xec\\x8b\\xa0\\xeb\\xa2\\xb0\\xeb\\x8f\\x84 (0.0 ~ 1.0)\n"
  "float32 amplitude          # \\xec\\x86\\x8c\\xeb\\xa6\\xac \\xed\\x81\\xac\\xea\\xb8\\xb0 (dB)\n"
  "bool    is_active          # \\xec\\x86\\x8c\\xeb\\xa6\\xac \\xea\\xb0\\x90\\xec\\xa7\\x80 \\xec\\x97\\xac\\xeb\\xb6\\x80";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
sound_interfaces__msg__SoundEvent__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {sound_interfaces__msg__SoundEvent__TYPE_NAME, 31, 31},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 253, 253},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
sound_interfaces__msg__SoundEvent__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[3];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 3, 3};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *sound_interfaces__msg__SoundEvent__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
