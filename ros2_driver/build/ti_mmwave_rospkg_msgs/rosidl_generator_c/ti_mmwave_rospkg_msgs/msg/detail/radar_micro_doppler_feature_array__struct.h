// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ti_mmwave_rospkg_msgs:msg/RadarMicroDopplerFeatureArray.idl
// generated code does not contain a copyright notice

#ifndef TI_MMWAVE_ROSPKG_MSGS__MSG__DETAIL__RADAR_MICRO_DOPPLER_FEATURE_ARRAY__STRUCT_H_
#define TI_MMWAVE_ROSPKG_MSGS__MSG__DETAIL__RADAR_MICRO_DOPPLER_FEATURE_ARRAY__STRUCT_H_

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
// Member 'track'
#include "ti_mmwave_rospkg_msgs/msg/detail/radar_micro_doppler_feature_contents__struct.h"

/// Struct defined in msg/RadarMicroDopplerFeatureArray in the package ti_mmwave_rospkg_msgs.
typedef struct ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureArray
{
  std_msgs__msg__Header header;
  uint32_t num_tracks;
  ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents__Sequence track;
} ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureArray;

// Struct for a sequence of ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureArray.
typedef struct ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureArray__Sequence
{
  ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TI_MMWAVE_ROSPKG_MSGS__MSG__DETAIL__RADAR_MICRO_DOPPLER_FEATURE_ARRAY__STRUCT_H_
