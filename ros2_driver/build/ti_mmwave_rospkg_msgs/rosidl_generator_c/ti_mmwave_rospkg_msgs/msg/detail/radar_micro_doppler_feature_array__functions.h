// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from ti_mmwave_rospkg_msgs:msg/RadarMicroDopplerFeatureArray.idl
// generated code does not contain a copyright notice

#ifndef TI_MMWAVE_ROSPKG_MSGS__MSG__DETAIL__RADAR_MICRO_DOPPLER_FEATURE_ARRAY__FUNCTIONS_H_
#define TI_MMWAVE_ROSPKG_MSGS__MSG__DETAIL__RADAR_MICRO_DOPPLER_FEATURE_ARRAY__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "ti_mmwave_rospkg_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "ti_mmwave_rospkg_msgs/msg/detail/radar_micro_doppler_feature_array__struct.h"

/// Initialize msg/RadarMicroDopplerFeatureArray message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureArray
 * )) before or use
 * ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureArray__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ti_mmwave_rospkg_msgs
bool
ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureArray__init(ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureArray * msg);

/// Finalize msg/RadarMicroDopplerFeatureArray message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ti_mmwave_rospkg_msgs
void
ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureArray__fini(ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureArray * msg);

/// Create msg/RadarMicroDopplerFeatureArray message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureArray__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ti_mmwave_rospkg_msgs
ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureArray *
ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureArray__create();

/// Destroy msg/RadarMicroDopplerFeatureArray message.
/**
 * It calls
 * ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureArray__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ti_mmwave_rospkg_msgs
void
ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureArray__destroy(ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureArray * msg);

/// Check for msg/RadarMicroDopplerFeatureArray message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ti_mmwave_rospkg_msgs
bool
ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureArray__are_equal(const ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureArray * lhs, const ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureArray * rhs);

/// Copy a msg/RadarMicroDopplerFeatureArray message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ti_mmwave_rospkg_msgs
bool
ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureArray__copy(
  const ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureArray * input,
  ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureArray * output);

/// Initialize array of msg/RadarMicroDopplerFeatureArray messages.
/**
 * It allocates the memory for the number of elements and calls
 * ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureArray__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ti_mmwave_rospkg_msgs
bool
ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureArray__Sequence__init(ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureArray__Sequence * array, size_t size);

/// Finalize array of msg/RadarMicroDopplerFeatureArray messages.
/**
 * It calls
 * ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureArray__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ti_mmwave_rospkg_msgs
void
ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureArray__Sequence__fini(ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureArray__Sequence * array);

/// Create array of msg/RadarMicroDopplerFeatureArray messages.
/**
 * It allocates the memory for the array and calls
 * ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureArray__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ti_mmwave_rospkg_msgs
ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureArray__Sequence *
ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureArray__Sequence__create(size_t size);

/// Destroy array of msg/RadarMicroDopplerFeatureArray messages.
/**
 * It calls
 * ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureArray__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ti_mmwave_rospkg_msgs
void
ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureArray__Sequence__destroy(ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureArray__Sequence * array);

/// Check for msg/RadarMicroDopplerFeatureArray message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ti_mmwave_rospkg_msgs
bool
ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureArray__Sequence__are_equal(const ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureArray__Sequence * lhs, const ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureArray__Sequence * rhs);

/// Copy an array of msg/RadarMicroDopplerFeatureArray messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ti_mmwave_rospkg_msgs
bool
ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureArray__Sequence__copy(
  const ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureArray__Sequence * input,
  ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureArray__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // TI_MMWAVE_ROSPKG_MSGS__MSG__DETAIL__RADAR_MICRO_DOPPLER_FEATURE_ARRAY__FUNCTIONS_H_
