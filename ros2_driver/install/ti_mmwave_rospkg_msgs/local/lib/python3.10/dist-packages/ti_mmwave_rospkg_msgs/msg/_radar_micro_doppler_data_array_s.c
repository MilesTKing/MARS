// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from ti_mmwave_rospkg_msgs:msg/RadarMicroDopplerDataArray.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "ti_mmwave_rospkg_msgs/msg/detail/radar_micro_doppler_data_array__struct.h"
#include "ti_mmwave_rospkg_msgs/msg/detail/radar_micro_doppler_data_array__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

// Nested array functions includes
#include "ti_mmwave_rospkg_msgs/msg/detail/radar_micro_doppler_data_contents__functions.h"
// end nested array functions include
ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);
bool ti_mmwave_rospkg_msgs__msg__radar_micro_doppler_data_contents__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * ti_mmwave_rospkg_msgs__msg__radar_micro_doppler_data_contents__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool ti_mmwave_rospkg_msgs__msg__radar_micro_doppler_data_array__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[85];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("ti_mmwave_rospkg_msgs.msg._radar_micro_doppler_data_array.RadarMicroDopplerDataArray", full_classname_dest, 84) == 0);
  }
  ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataArray * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // num_tracks
    PyObject * field = PyObject_GetAttrString(_pymsg, "num_tracks");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->num_tracks = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // track
    PyObject * field = PyObject_GetAttrString(_pymsg, "track");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'track'");
    if (!seq_field) {
      Py_DECREF(field);
      return false;
    }
    Py_ssize_t size = PySequence_Size(field);
    if (-1 == size) {
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    if (!ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataContents__Sequence__init(&(ros_message->track), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataContents__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataContents * dest = ros_message->track.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!ti_mmwave_rospkg_msgs__msg__radar_micro_doppler_data_contents__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * ti_mmwave_rospkg_msgs__msg__radar_micro_doppler_data_array__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of RadarMicroDopplerDataArray */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("ti_mmwave_rospkg_msgs.msg._radar_micro_doppler_data_array");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "RadarMicroDopplerDataArray");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataArray * ros_message = (ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataArray *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // num_tracks
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->num_tracks);
    {
      int rc = PyObject_SetAttrString(_pymessage, "num_tracks", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // track
    PyObject * field = NULL;
    size_t size = ros_message->track.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataContents * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->track.data[i]);
      PyObject * pyitem = ti_mmwave_rospkg_msgs__msg__radar_micro_doppler_data_contents__convert_to_py(item);
      if (!pyitem) {
        Py_DECREF(field);
        return NULL;
      }
      int rc = PyList_SetItem(field, i, pyitem);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "track", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
