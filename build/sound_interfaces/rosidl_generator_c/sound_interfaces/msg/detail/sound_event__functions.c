// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from sound_interfaces:msg/SoundEvent.idl
// generated code does not contain a copyright notice
#include "sound_interfaces/msg/detail/sound_event__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
sound_interfaces__msg__SoundEvent__init(sound_interfaces__msg__SoundEvent * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    sound_interfaces__msg__SoundEvent__fini(msg);
    return false;
  }
  // angle
  // confidence
  // amplitude
  // is_active
  return true;
}

void
sound_interfaces__msg__SoundEvent__fini(sound_interfaces__msg__SoundEvent * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // angle
  // confidence
  // amplitude
  // is_active
}

bool
sound_interfaces__msg__SoundEvent__are_equal(const sound_interfaces__msg__SoundEvent * lhs, const sound_interfaces__msg__SoundEvent * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // angle
  if (lhs->angle != rhs->angle) {
    return false;
  }
  // confidence
  if (lhs->confidence != rhs->confidence) {
    return false;
  }
  // amplitude
  if (lhs->amplitude != rhs->amplitude) {
    return false;
  }
  // is_active
  if (lhs->is_active != rhs->is_active) {
    return false;
  }
  return true;
}

bool
sound_interfaces__msg__SoundEvent__copy(
  const sound_interfaces__msg__SoundEvent * input,
  sound_interfaces__msg__SoundEvent * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // angle
  output->angle = input->angle;
  // confidence
  output->confidence = input->confidence;
  // amplitude
  output->amplitude = input->amplitude;
  // is_active
  output->is_active = input->is_active;
  return true;
}

sound_interfaces__msg__SoundEvent *
sound_interfaces__msg__SoundEvent__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sound_interfaces__msg__SoundEvent * msg = (sound_interfaces__msg__SoundEvent *)allocator.allocate(sizeof(sound_interfaces__msg__SoundEvent), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(sound_interfaces__msg__SoundEvent));
  bool success = sound_interfaces__msg__SoundEvent__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
sound_interfaces__msg__SoundEvent__destroy(sound_interfaces__msg__SoundEvent * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    sound_interfaces__msg__SoundEvent__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
sound_interfaces__msg__SoundEvent__Sequence__init(sound_interfaces__msg__SoundEvent__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sound_interfaces__msg__SoundEvent * data = NULL;

  if (size) {
    data = (sound_interfaces__msg__SoundEvent *)allocator.zero_allocate(size, sizeof(sound_interfaces__msg__SoundEvent), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = sound_interfaces__msg__SoundEvent__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        sound_interfaces__msg__SoundEvent__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
sound_interfaces__msg__SoundEvent__Sequence__fini(sound_interfaces__msg__SoundEvent__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      sound_interfaces__msg__SoundEvent__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

sound_interfaces__msg__SoundEvent__Sequence *
sound_interfaces__msg__SoundEvent__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sound_interfaces__msg__SoundEvent__Sequence * array = (sound_interfaces__msg__SoundEvent__Sequence *)allocator.allocate(sizeof(sound_interfaces__msg__SoundEvent__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = sound_interfaces__msg__SoundEvent__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
sound_interfaces__msg__SoundEvent__Sequence__destroy(sound_interfaces__msg__SoundEvent__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    sound_interfaces__msg__SoundEvent__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
sound_interfaces__msg__SoundEvent__Sequence__are_equal(const sound_interfaces__msg__SoundEvent__Sequence * lhs, const sound_interfaces__msg__SoundEvent__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!sound_interfaces__msg__SoundEvent__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
sound_interfaces__msg__SoundEvent__Sequence__copy(
  const sound_interfaces__msg__SoundEvent__Sequence * input,
  sound_interfaces__msg__SoundEvent__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(sound_interfaces__msg__SoundEvent);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    sound_interfaces__msg__SoundEvent * data =
      (sound_interfaces__msg__SoundEvent *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!sound_interfaces__msg__SoundEvent__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          sound_interfaces__msg__SoundEvent__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!sound_interfaces__msg__SoundEvent__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
