// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ti_mmwave_rospkg_msgs:msg/RadarPointTrackID.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ti_mmwave_rospkg_msgs/msg/detail/radar_point_track_id__rosidl_typesupport_introspection_c.h"
#include "ti_mmwave_rospkg_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ti_mmwave_rospkg_msgs/msg/detail/radar_point_track_id__functions.h"
#include "ti_mmwave_rospkg_msgs/msg/detail/radar_point_track_id__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ti_mmwave_rospkg_msgs__msg__RadarPointTrackID__rosidl_typesupport_introspection_c__RadarPointTrackID_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ti_mmwave_rospkg_msgs__msg__RadarPointTrackID__init(message_memory);
}

void ti_mmwave_rospkg_msgs__msg__RadarPointTrackID__rosidl_typesupport_introspection_c__RadarPointTrackID_fini_function(void * message_memory)
{
  ti_mmwave_rospkg_msgs__msg__RadarPointTrackID__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember ti_mmwave_rospkg_msgs__msg__RadarPointTrackID__rosidl_typesupport_introspection_c__RadarPointTrackID_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ti_mmwave_rospkg_msgs__msg__RadarPointTrackID, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "tid",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ti_mmwave_rospkg_msgs__msg__RadarPointTrackID, tid),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ti_mmwave_rospkg_msgs__msg__RadarPointTrackID__rosidl_typesupport_introspection_c__RadarPointTrackID_message_members = {
  "ti_mmwave_rospkg_msgs__msg",  // message namespace
  "RadarPointTrackID",  // message name
  2,  // number of fields
  sizeof(ti_mmwave_rospkg_msgs__msg__RadarPointTrackID),
  ti_mmwave_rospkg_msgs__msg__RadarPointTrackID__rosidl_typesupport_introspection_c__RadarPointTrackID_message_member_array,  // message members
  ti_mmwave_rospkg_msgs__msg__RadarPointTrackID__rosidl_typesupport_introspection_c__RadarPointTrackID_init_function,  // function to initialize message memory (memory has to be allocated)
  ti_mmwave_rospkg_msgs__msg__RadarPointTrackID__rosidl_typesupport_introspection_c__RadarPointTrackID_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ti_mmwave_rospkg_msgs__msg__RadarPointTrackID__rosidl_typesupport_introspection_c__RadarPointTrackID_message_type_support_handle = {
  0,
  &ti_mmwave_rospkg_msgs__msg__RadarPointTrackID__rosidl_typesupport_introspection_c__RadarPointTrackID_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ti_mmwave_rospkg_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ti_mmwave_rospkg_msgs, msg, RadarPointTrackID)() {
  ti_mmwave_rospkg_msgs__msg__RadarPointTrackID__rosidl_typesupport_introspection_c__RadarPointTrackID_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!ti_mmwave_rospkg_msgs__msg__RadarPointTrackID__rosidl_typesupport_introspection_c__RadarPointTrackID_message_type_support_handle.typesupport_identifier) {
    ti_mmwave_rospkg_msgs__msg__RadarPointTrackID__rosidl_typesupport_introspection_c__RadarPointTrackID_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ti_mmwave_rospkg_msgs__msg__RadarPointTrackID__rosidl_typesupport_introspection_c__RadarPointTrackID_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
