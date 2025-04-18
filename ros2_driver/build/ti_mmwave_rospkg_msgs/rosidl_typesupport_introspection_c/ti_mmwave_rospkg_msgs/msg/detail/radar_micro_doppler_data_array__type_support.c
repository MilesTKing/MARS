// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ti_mmwave_rospkg_msgs:msg/RadarMicroDopplerDataArray.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ti_mmwave_rospkg_msgs/msg/detail/radar_micro_doppler_data_array__rosidl_typesupport_introspection_c.h"
#include "ti_mmwave_rospkg_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ti_mmwave_rospkg_msgs/msg/detail/radar_micro_doppler_data_array__functions.h"
#include "ti_mmwave_rospkg_msgs/msg/detail/radar_micro_doppler_data_array__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `track`
#include "ti_mmwave_rospkg_msgs/msg/radar_micro_doppler_data_contents.h"
// Member `track`
#include "ti_mmwave_rospkg_msgs/msg/detail/radar_micro_doppler_data_contents__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataArray__rosidl_typesupport_introspection_c__RadarMicroDopplerDataArray_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataArray__init(message_memory);
}

void ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataArray__rosidl_typesupport_introspection_c__RadarMicroDopplerDataArray_fini_function(void * message_memory)
{
  ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataArray__fini(message_memory);
}

size_t ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataArray__rosidl_typesupport_introspection_c__size_function__RadarMicroDopplerDataArray__track(
  const void * untyped_member)
{
  const ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataContents__Sequence * member =
    (const ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataContents__Sequence *)(untyped_member);
  return member->size;
}

const void * ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataArray__rosidl_typesupport_introspection_c__get_const_function__RadarMicroDopplerDataArray__track(
  const void * untyped_member, size_t index)
{
  const ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataContents__Sequence * member =
    (const ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataContents__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataArray__rosidl_typesupport_introspection_c__get_function__RadarMicroDopplerDataArray__track(
  void * untyped_member, size_t index)
{
  ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataContents__Sequence * member =
    (ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataContents__Sequence *)(untyped_member);
  return &member->data[index];
}

void ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataArray__rosidl_typesupport_introspection_c__fetch_function__RadarMicroDopplerDataArray__track(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataContents * item =
    ((const ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataContents *)
    ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataArray__rosidl_typesupport_introspection_c__get_const_function__RadarMicroDopplerDataArray__track(untyped_member, index));
  ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataContents * value =
    (ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataContents *)(untyped_value);
  *value = *item;
}

void ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataArray__rosidl_typesupport_introspection_c__assign_function__RadarMicroDopplerDataArray__track(
  void * untyped_member, size_t index, const void * untyped_value)
{
  ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataContents * item =
    ((ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataContents *)
    ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataArray__rosidl_typesupport_introspection_c__get_function__RadarMicroDopplerDataArray__track(untyped_member, index));
  const ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataContents * value =
    (const ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataContents *)(untyped_value);
  *item = *value;
}

bool ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataArray__rosidl_typesupport_introspection_c__resize_function__RadarMicroDopplerDataArray__track(
  void * untyped_member, size_t size)
{
  ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataContents__Sequence * member =
    (ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataContents__Sequence *)(untyped_member);
  ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataContents__Sequence__fini(member);
  return ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataContents__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataArray__rosidl_typesupport_introspection_c__RadarMicroDopplerDataArray_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataArray, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "num_tracks",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataArray, num_tracks),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "track",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataArray, track),  // bytes offset in struct
    NULL,  // default value
    ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataArray__rosidl_typesupport_introspection_c__size_function__RadarMicroDopplerDataArray__track,  // size() function pointer
    ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataArray__rosidl_typesupport_introspection_c__get_const_function__RadarMicroDopplerDataArray__track,  // get_const(index) function pointer
    ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataArray__rosidl_typesupport_introspection_c__get_function__RadarMicroDopplerDataArray__track,  // get(index) function pointer
    ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataArray__rosidl_typesupport_introspection_c__fetch_function__RadarMicroDopplerDataArray__track,  // fetch(index, &value) function pointer
    ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataArray__rosidl_typesupport_introspection_c__assign_function__RadarMicroDopplerDataArray__track,  // assign(index, value) function pointer
    ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataArray__rosidl_typesupport_introspection_c__resize_function__RadarMicroDopplerDataArray__track  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataArray__rosidl_typesupport_introspection_c__RadarMicroDopplerDataArray_message_members = {
  "ti_mmwave_rospkg_msgs__msg",  // message namespace
  "RadarMicroDopplerDataArray",  // message name
  3,  // number of fields
  sizeof(ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataArray),
  ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataArray__rosidl_typesupport_introspection_c__RadarMicroDopplerDataArray_message_member_array,  // message members
  ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataArray__rosidl_typesupport_introspection_c__RadarMicroDopplerDataArray_init_function,  // function to initialize message memory (memory has to be allocated)
  ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataArray__rosidl_typesupport_introspection_c__RadarMicroDopplerDataArray_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataArray__rosidl_typesupport_introspection_c__RadarMicroDopplerDataArray_message_type_support_handle = {
  0,
  &ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataArray__rosidl_typesupport_introspection_c__RadarMicroDopplerDataArray_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ti_mmwave_rospkg_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ti_mmwave_rospkg_msgs, msg, RadarMicroDopplerDataArray)() {
  ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataArray__rosidl_typesupport_introspection_c__RadarMicroDopplerDataArray_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataArray__rosidl_typesupport_introspection_c__RadarMicroDopplerDataArray_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ti_mmwave_rospkg_msgs, msg, RadarMicroDopplerDataContents)();
  if (!ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataArray__rosidl_typesupport_introspection_c__RadarMicroDopplerDataArray_message_type_support_handle.typesupport_identifier) {
    ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataArray__rosidl_typesupport_introspection_c__RadarMicroDopplerDataArray_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataArray__rosidl_typesupport_introspection_c__RadarMicroDopplerDataArray_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
