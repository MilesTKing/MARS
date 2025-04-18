// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ti_mmwave_rospkg_msgs:msg/RadarClassifier.idl
// generated code does not contain a copyright notice

#ifndef TI_MMWAVE_ROSPKG_MSGS__MSG__DETAIL__RADAR_CLASSIFIER__BUILDER_HPP_
#define TI_MMWAVE_ROSPKG_MSGS__MSG__DETAIL__RADAR_CLASSIFIER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ti_mmwave_rospkg_msgs/msg/detail/radar_classifier__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ti_mmwave_rospkg_msgs
{

namespace msg
{

namespace builder
{

class Init_RadarClassifier_value
{
public:
  explicit Init_RadarClassifier_value(::ti_mmwave_rospkg_msgs::msg::RadarClassifier & msg)
  : msg_(msg)
  {}
  ::ti_mmwave_rospkg_msgs::msg::RadarClassifier value(::ti_mmwave_rospkg_msgs::msg::RadarClassifier::_value_type arg)
  {
    msg_.value = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ti_mmwave_rospkg_msgs::msg::RadarClassifier msg_;
};

class Init_RadarClassifier_track_id
{
public:
  Init_RadarClassifier_track_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RadarClassifier_value track_id(::ti_mmwave_rospkg_msgs::msg::RadarClassifier::_track_id_type arg)
  {
    msg_.track_id = std::move(arg);
    return Init_RadarClassifier_value(msg_);
  }

private:
  ::ti_mmwave_rospkg_msgs::msg::RadarClassifier msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ti_mmwave_rospkg_msgs::msg::RadarClassifier>()
{
  return ti_mmwave_rospkg_msgs::msg::builder::Init_RadarClassifier_track_id();
}

}  // namespace ti_mmwave_rospkg_msgs

#endif  // TI_MMWAVE_ROSPKG_MSGS__MSG__DETAIL__RADAR_CLASSIFIER__BUILDER_HPP_
