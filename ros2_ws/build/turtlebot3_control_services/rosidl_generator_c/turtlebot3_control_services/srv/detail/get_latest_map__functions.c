// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from turtlebot3_control_services:srv/GetLatestMap.idl
// generated code does not contain a copyright notice
#include "turtlebot3_control_services/srv/detail/get_latest_map__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
turtlebot3_control_services__srv__GetLatestMap_Request__init(turtlebot3_control_services__srv__GetLatestMap_Request * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
turtlebot3_control_services__srv__GetLatestMap_Request__fini(turtlebot3_control_services__srv__GetLatestMap_Request * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
turtlebot3_control_services__srv__GetLatestMap_Request__are_equal(const turtlebot3_control_services__srv__GetLatestMap_Request * lhs, const turtlebot3_control_services__srv__GetLatestMap_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
turtlebot3_control_services__srv__GetLatestMap_Request__copy(
  const turtlebot3_control_services__srv__GetLatestMap_Request * input,
  turtlebot3_control_services__srv__GetLatestMap_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

turtlebot3_control_services__srv__GetLatestMap_Request *
turtlebot3_control_services__srv__GetLatestMap_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  turtlebot3_control_services__srv__GetLatestMap_Request * msg = (turtlebot3_control_services__srv__GetLatestMap_Request *)allocator.allocate(sizeof(turtlebot3_control_services__srv__GetLatestMap_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(turtlebot3_control_services__srv__GetLatestMap_Request));
  bool success = turtlebot3_control_services__srv__GetLatestMap_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
turtlebot3_control_services__srv__GetLatestMap_Request__destroy(turtlebot3_control_services__srv__GetLatestMap_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    turtlebot3_control_services__srv__GetLatestMap_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
turtlebot3_control_services__srv__GetLatestMap_Request__Sequence__init(turtlebot3_control_services__srv__GetLatestMap_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  turtlebot3_control_services__srv__GetLatestMap_Request * data = NULL;

  if (size) {
    data = (turtlebot3_control_services__srv__GetLatestMap_Request *)allocator.zero_allocate(size, sizeof(turtlebot3_control_services__srv__GetLatestMap_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = turtlebot3_control_services__srv__GetLatestMap_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        turtlebot3_control_services__srv__GetLatestMap_Request__fini(&data[i - 1]);
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
turtlebot3_control_services__srv__GetLatestMap_Request__Sequence__fini(turtlebot3_control_services__srv__GetLatestMap_Request__Sequence * array)
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
      turtlebot3_control_services__srv__GetLatestMap_Request__fini(&array->data[i]);
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

turtlebot3_control_services__srv__GetLatestMap_Request__Sequence *
turtlebot3_control_services__srv__GetLatestMap_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  turtlebot3_control_services__srv__GetLatestMap_Request__Sequence * array = (turtlebot3_control_services__srv__GetLatestMap_Request__Sequence *)allocator.allocate(sizeof(turtlebot3_control_services__srv__GetLatestMap_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = turtlebot3_control_services__srv__GetLatestMap_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
turtlebot3_control_services__srv__GetLatestMap_Request__Sequence__destroy(turtlebot3_control_services__srv__GetLatestMap_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    turtlebot3_control_services__srv__GetLatestMap_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
turtlebot3_control_services__srv__GetLatestMap_Request__Sequence__are_equal(const turtlebot3_control_services__srv__GetLatestMap_Request__Sequence * lhs, const turtlebot3_control_services__srv__GetLatestMap_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!turtlebot3_control_services__srv__GetLatestMap_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
turtlebot3_control_services__srv__GetLatestMap_Request__Sequence__copy(
  const turtlebot3_control_services__srv__GetLatestMap_Request__Sequence * input,
  turtlebot3_control_services__srv__GetLatestMap_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(turtlebot3_control_services__srv__GetLatestMap_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    turtlebot3_control_services__srv__GetLatestMap_Request * data =
      (turtlebot3_control_services__srv__GetLatestMap_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!turtlebot3_control_services__srv__GetLatestMap_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          turtlebot3_control_services__srv__GetLatestMap_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!turtlebot3_control_services__srv__GetLatestMap_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `map_path`
#include "rosidl_runtime_c/string_functions.h"

bool
turtlebot3_control_services__srv__GetLatestMap_Response__init(turtlebot3_control_services__srv__GetLatestMap_Response * msg)
{
  if (!msg) {
    return false;
  }
  // map_path
  if (!rosidl_runtime_c__String__init(&msg->map_path)) {
    turtlebot3_control_services__srv__GetLatestMap_Response__fini(msg);
    return false;
  }
  return true;
}

void
turtlebot3_control_services__srv__GetLatestMap_Response__fini(turtlebot3_control_services__srv__GetLatestMap_Response * msg)
{
  if (!msg) {
    return;
  }
  // map_path
  rosidl_runtime_c__String__fini(&msg->map_path);
}

bool
turtlebot3_control_services__srv__GetLatestMap_Response__are_equal(const turtlebot3_control_services__srv__GetLatestMap_Response * lhs, const turtlebot3_control_services__srv__GetLatestMap_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // map_path
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->map_path), &(rhs->map_path)))
  {
    return false;
  }
  return true;
}

bool
turtlebot3_control_services__srv__GetLatestMap_Response__copy(
  const turtlebot3_control_services__srv__GetLatestMap_Response * input,
  turtlebot3_control_services__srv__GetLatestMap_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // map_path
  if (!rosidl_runtime_c__String__copy(
      &(input->map_path), &(output->map_path)))
  {
    return false;
  }
  return true;
}

turtlebot3_control_services__srv__GetLatestMap_Response *
turtlebot3_control_services__srv__GetLatestMap_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  turtlebot3_control_services__srv__GetLatestMap_Response * msg = (turtlebot3_control_services__srv__GetLatestMap_Response *)allocator.allocate(sizeof(turtlebot3_control_services__srv__GetLatestMap_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(turtlebot3_control_services__srv__GetLatestMap_Response));
  bool success = turtlebot3_control_services__srv__GetLatestMap_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
turtlebot3_control_services__srv__GetLatestMap_Response__destroy(turtlebot3_control_services__srv__GetLatestMap_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    turtlebot3_control_services__srv__GetLatestMap_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
turtlebot3_control_services__srv__GetLatestMap_Response__Sequence__init(turtlebot3_control_services__srv__GetLatestMap_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  turtlebot3_control_services__srv__GetLatestMap_Response * data = NULL;

  if (size) {
    data = (turtlebot3_control_services__srv__GetLatestMap_Response *)allocator.zero_allocate(size, sizeof(turtlebot3_control_services__srv__GetLatestMap_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = turtlebot3_control_services__srv__GetLatestMap_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        turtlebot3_control_services__srv__GetLatestMap_Response__fini(&data[i - 1]);
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
turtlebot3_control_services__srv__GetLatestMap_Response__Sequence__fini(turtlebot3_control_services__srv__GetLatestMap_Response__Sequence * array)
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
      turtlebot3_control_services__srv__GetLatestMap_Response__fini(&array->data[i]);
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

turtlebot3_control_services__srv__GetLatestMap_Response__Sequence *
turtlebot3_control_services__srv__GetLatestMap_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  turtlebot3_control_services__srv__GetLatestMap_Response__Sequence * array = (turtlebot3_control_services__srv__GetLatestMap_Response__Sequence *)allocator.allocate(sizeof(turtlebot3_control_services__srv__GetLatestMap_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = turtlebot3_control_services__srv__GetLatestMap_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
turtlebot3_control_services__srv__GetLatestMap_Response__Sequence__destroy(turtlebot3_control_services__srv__GetLatestMap_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    turtlebot3_control_services__srv__GetLatestMap_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
turtlebot3_control_services__srv__GetLatestMap_Response__Sequence__are_equal(const turtlebot3_control_services__srv__GetLatestMap_Response__Sequence * lhs, const turtlebot3_control_services__srv__GetLatestMap_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!turtlebot3_control_services__srv__GetLatestMap_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
turtlebot3_control_services__srv__GetLatestMap_Response__Sequence__copy(
  const turtlebot3_control_services__srv__GetLatestMap_Response__Sequence * input,
  turtlebot3_control_services__srv__GetLatestMap_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(turtlebot3_control_services__srv__GetLatestMap_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    turtlebot3_control_services__srv__GetLatestMap_Response * data =
      (turtlebot3_control_services__srv__GetLatestMap_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!turtlebot3_control_services__srv__GetLatestMap_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          turtlebot3_control_services__srv__GetLatestMap_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!turtlebot3_control_services__srv__GetLatestMap_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
