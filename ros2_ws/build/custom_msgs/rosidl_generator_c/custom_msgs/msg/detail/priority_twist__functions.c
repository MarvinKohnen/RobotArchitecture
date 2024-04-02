// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from custom_msgs:msg/PriorityTwist.idl
// generated code does not contain a copyright notice
#include "custom_msgs/msg/detail/priority_twist__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `command`
#include "geometry_msgs/msg/detail/twist__functions.h"

bool
custom_msgs__msg__PriorityTwist__init(custom_msgs__msg__PriorityTwist * msg)
{
  if (!msg) {
    return false;
  }
  // command
  if (!geometry_msgs__msg__Twist__init(&msg->command)) {
    custom_msgs__msg__PriorityTwist__fini(msg);
    return false;
  }
  // priority
  return true;
}

void
custom_msgs__msg__PriorityTwist__fini(custom_msgs__msg__PriorityTwist * msg)
{
  if (!msg) {
    return;
  }
  // command
  geometry_msgs__msg__Twist__fini(&msg->command);
  // priority
}

bool
custom_msgs__msg__PriorityTwist__are_equal(const custom_msgs__msg__PriorityTwist * lhs, const custom_msgs__msg__PriorityTwist * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // command
  if (!geometry_msgs__msg__Twist__are_equal(
      &(lhs->command), &(rhs->command)))
  {
    return false;
  }
  // priority
  if (lhs->priority != rhs->priority) {
    return false;
  }
  return true;
}

bool
custom_msgs__msg__PriorityTwist__copy(
  const custom_msgs__msg__PriorityTwist * input,
  custom_msgs__msg__PriorityTwist * output)
{
  if (!input || !output) {
    return false;
  }
  // command
  if (!geometry_msgs__msg__Twist__copy(
      &(input->command), &(output->command)))
  {
    return false;
  }
  // priority
  output->priority = input->priority;
  return true;
}

custom_msgs__msg__PriorityTwist *
custom_msgs__msg__PriorityTwist__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_msgs__msg__PriorityTwist * msg = (custom_msgs__msg__PriorityTwist *)allocator.allocate(sizeof(custom_msgs__msg__PriorityTwist), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(custom_msgs__msg__PriorityTwist));
  bool success = custom_msgs__msg__PriorityTwist__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
custom_msgs__msg__PriorityTwist__destroy(custom_msgs__msg__PriorityTwist * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    custom_msgs__msg__PriorityTwist__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
custom_msgs__msg__PriorityTwist__Sequence__init(custom_msgs__msg__PriorityTwist__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_msgs__msg__PriorityTwist * data = NULL;

  if (size) {
    data = (custom_msgs__msg__PriorityTwist *)allocator.zero_allocate(size, sizeof(custom_msgs__msg__PriorityTwist), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = custom_msgs__msg__PriorityTwist__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        custom_msgs__msg__PriorityTwist__fini(&data[i - 1]);
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
custom_msgs__msg__PriorityTwist__Sequence__fini(custom_msgs__msg__PriorityTwist__Sequence * array)
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
      custom_msgs__msg__PriorityTwist__fini(&array->data[i]);
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

custom_msgs__msg__PriorityTwist__Sequence *
custom_msgs__msg__PriorityTwist__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_msgs__msg__PriorityTwist__Sequence * array = (custom_msgs__msg__PriorityTwist__Sequence *)allocator.allocate(sizeof(custom_msgs__msg__PriorityTwist__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = custom_msgs__msg__PriorityTwist__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
custom_msgs__msg__PriorityTwist__Sequence__destroy(custom_msgs__msg__PriorityTwist__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    custom_msgs__msg__PriorityTwist__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
custom_msgs__msg__PriorityTwist__Sequence__are_equal(const custom_msgs__msg__PriorityTwist__Sequence * lhs, const custom_msgs__msg__PriorityTwist__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!custom_msgs__msg__PriorityTwist__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
custom_msgs__msg__PriorityTwist__Sequence__copy(
  const custom_msgs__msg__PriorityTwist__Sequence * input,
  custom_msgs__msg__PriorityTwist__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(custom_msgs__msg__PriorityTwist);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    custom_msgs__msg__PriorityTwist * data =
      (custom_msgs__msg__PriorityTwist *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!custom_msgs__msg__PriorityTwist__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          custom_msgs__msg__PriorityTwist__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!custom_msgs__msg__PriorityTwist__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
