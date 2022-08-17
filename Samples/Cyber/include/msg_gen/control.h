// Generated by gencpp from file msg_gen/control.msg
// DO NOT EDIT!


#ifndef MSG_GEN_MESSAGE_CONTROL_H
#define MSG_GEN_MESSAGE_CONTROL_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <msg_gen/EThrottleMode.h>
#include <msg_gen/EBrakeMode.h>
#include <msg_gen/ESteeringMode.h>
#include <msg_gen/EGearMode.h>

namespace msg_gen
{
template <class ContainerAllocator>
struct control_
{
  typedef control_<ContainerAllocator> Type;

  control_()
    : throttleMode()
    , throttle(0.0)
    , brakeMode()
    , brake(0.0)
    , steeringMode()
    , steering(0.0)
    , handbrake(false)
    , isManualGear(false)
    , gear()
    , clutch(0.0)
    , throttle_input_data()
    , brake_input_data()
    , steering_input_data()  {
    }
  control_(const ContainerAllocator& _alloc)
    : throttleMode(_alloc)
    , throttle(0.0)
    , brakeMode(_alloc)
    , brake(0.0)
    , steeringMode(_alloc)
    , steering(0.0)
    , handbrake(false)
    , isManualGear(false)
    , gear(_alloc)
    , clutch(0.0)
    , throttle_input_data(_alloc)
    , brake_input_data(_alloc)
    , steering_input_data(_alloc)  {
  (void)_alloc;
    }



   typedef  ::msg_gen::EThrottleMode_<ContainerAllocator>  _throttleMode_type;
  _throttleMode_type throttleMode;

   typedef double _throttle_type;
  _throttle_type throttle;

   typedef  ::msg_gen::EBrakeMode_<ContainerAllocator>  _brakeMode_type;
  _brakeMode_type brakeMode;

   typedef double _brake_type;
  _brake_type brake;

   typedef  ::msg_gen::ESteeringMode_<ContainerAllocator>  _steeringMode_type;
  _steeringMode_type steeringMode;

   typedef double _steering_type;
  _steering_type steering;

   typedef uint8_t _handbrake_type;
  _handbrake_type handbrake;

   typedef uint8_t _isManualGear_type;
  _isManualGear_type isManualGear;

   typedef  ::msg_gen::EGearMode_<ContainerAllocator>  _gear_type;
  _gear_type gear;

   typedef double _clutch_type;
  _clutch_type clutch;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _throttle_input_data_type;
  _throttle_input_data_type throttle_input_data;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _brake_input_data_type;
  _brake_input_data_type brake_input_data;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _steering_input_data_type;
  _steering_input_data_type steering_input_data;





  typedef boost::shared_ptr< ::msg_gen::control_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::msg_gen::control_<ContainerAllocator> const> ConstPtr;

}; // struct control_

typedef ::msg_gen::control_<std::allocator<void> > control;

typedef boost::shared_ptr< ::msg_gen::control > controlPtr;
typedef boost::shared_ptr< ::msg_gen::control const> controlConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::msg_gen::control_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::msg_gen::control_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::msg_gen::control_<ContainerAllocator1> & lhs, const ::msg_gen::control_<ContainerAllocator2> & rhs)
{
  return lhs.throttleMode == rhs.throttleMode &&
    lhs.throttle == rhs.throttle &&
    lhs.brakeMode == rhs.brakeMode &&
    lhs.brake == rhs.brake &&
    lhs.steeringMode == rhs.steeringMode &&
    lhs.steering == rhs.steering &&
    lhs.handbrake == rhs.handbrake &&
    lhs.isManualGear == rhs.isManualGear &&
    lhs.gear == rhs.gear &&
    lhs.clutch == rhs.clutch &&
    lhs.throttle_input_data == rhs.throttle_input_data &&
    lhs.brake_input_data == rhs.brake_input_data &&
    lhs.steering_input_data == rhs.steering_input_data;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::msg_gen::control_<ContainerAllocator1> & lhs, const ::msg_gen::control_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace msg_gen

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::msg_gen::control_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::msg_gen::control_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::msg_gen::control_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::msg_gen::control_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::msg_gen::control_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::msg_gen::control_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::msg_gen::control_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8b2abfae631de46b09ce8107f0acb9b3";
  }

  static const char* value(const ::msg_gen::control_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8b2abfae631de46bULL;
  static const uint64_t static_value2 = 0x09ce8107f0acb9b3ULL;
};

template<class ContainerAllocator>
struct DataType< ::msg_gen::control_<ContainerAllocator> >
{
  static const char* value()
  {
    return "msg_gen/control";
  }

  static const char* value(const ::msg_gen::control_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::msg_gen::control_<ContainerAllocator> >
{
  static const char* value()
  {
    return "EThrottleMode throttleMode\n"
"float64 throttle\n"
"EBrakeMode brakeMode\n"
"float64 brake\n"
"ESteeringMode steeringMode\n"
"float64 steering\n"
"bool handbrake\n"
"bool isManualGear\n"
"EGearMode  gear\n"
"float64 clutch\n"
"float64[] throttle_input_data\n"
"float64[] brake_input_data\n"
"float64[] steering_input_data\n"
"\n"
"================================================================================\n"
"MSG: msg_gen/EThrottleMode\n"
"uint8 EThrottleMode_Percent = 0\n"
"uint8 EThrottleMode_Torque = 1\n"
"uint8 EThrottleMode_Speed = 2\n"
"uint8 EThrottleMode_Accel = 3\n"
"uint8 EThrottleMode_EngineAV = 4\n"
"uint8 EThrottleMode_WheelTorque = 5\n"
"uint32 ThrottleMode\n"
"================================================================================\n"
"MSG: msg_gen/EBrakeMode\n"
"uint8 EBrakeMode_Percent = 0\n"
"uint8 EBrakeMode_MasterCylinderPressure = 1\n"
"uint8 EBrakeMode_PedalForce = 2\n"
"uint8 EBrakeMode_WheelCylinderPressure = 3\n"
"uint8 EBrakeMode_WheelTorque = 4\n"
"uint32 BrakeMode\n"
"================================================================================\n"
"MSG: msg_gen/ESteeringMode\n"
"uint8 ESteeringMode_Percent = 0\n"
"uint8 ESteeringMode_SteeringWheelAngle = 1\n"
"uint8 ESteeringMode_Torque = 2\n"
"uint8 ESteeringMode_AngularSpeed = 3\n"
"uint8 ESteeringMode_WheelAngle = 4\n"
"uint8 ESteeringMode_WheelAnglarSpeed = 5\n"
"uint64 SteeringMode\n"
"================================================================================\n"
"MSG: msg_gen/EGearMode\n"
"uint8 EGearMode_Neutral = 0\n"
"uint8 EGearMode_Drive = 1\n"
"uint8 EGearMode_Reverse = 2\n"
"uint8 EGearMode_Parking = 3\n"
"uint8 EGearManualMode_1 = 4\n"
"uint8 EGearManualMode_2 = 5\n"
"uint8 EGearManualMode_3 = 6\n"
"uint8 EGearManualMode_4 = 7\n"
"uint8 EGearManualMode_5 = 8\n"
"uint8 EGearManualMode_6 = 9\n"
"uint8 EGearManualMode_7 = 10\n"
"uint8 EGearManualMode_8 = 11\n"
"uint32 GearMode\n"
;
  }

  static const char* value(const ::msg_gen::control_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::msg_gen::control_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.throttleMode);
      stream.next(m.throttle);
      stream.next(m.brakeMode);
      stream.next(m.brake);
      stream.next(m.steeringMode);
      stream.next(m.steering);
      stream.next(m.handbrake);
      stream.next(m.isManualGear);
      stream.next(m.gear);
      stream.next(m.clutch);
      stream.next(m.throttle_input_data);
      stream.next(m.brake_input_data);
      stream.next(m.steering_input_data);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct control_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::msg_gen::control_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::msg_gen::control_<ContainerAllocator>& v)
  {
    s << indent << "throttleMode: ";
    s << std::endl;
    Printer< ::msg_gen::EThrottleMode_<ContainerAllocator> >::stream(s, indent + "  ", v.throttleMode);
    s << indent << "throttle: ";
    Printer<double>::stream(s, indent + "  ", v.throttle);
    s << indent << "brakeMode: ";
    s << std::endl;
    Printer< ::msg_gen::EBrakeMode_<ContainerAllocator> >::stream(s, indent + "  ", v.brakeMode);
    s << indent << "brake: ";
    Printer<double>::stream(s, indent + "  ", v.brake);
    s << indent << "steeringMode: ";
    s << std::endl;
    Printer< ::msg_gen::ESteeringMode_<ContainerAllocator> >::stream(s, indent + "  ", v.steeringMode);
    s << indent << "steering: ";
    Printer<double>::stream(s, indent + "  ", v.steering);
    s << indent << "handbrake: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.handbrake);
    s << indent << "isManualGear: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.isManualGear);
    s << indent << "gear: ";
    s << std::endl;
    Printer< ::msg_gen::EGearMode_<ContainerAllocator> >::stream(s, indent + "  ", v.gear);
    s << indent << "clutch: ";
    Printer<double>::stream(s, indent + "  ", v.clutch);
    s << indent << "throttle_input_data[]" << std::endl;
    for (size_t i = 0; i < v.throttle_input_data.size(); ++i)
    {
      s << indent << "  throttle_input_data[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.throttle_input_data[i]);
    }
    s << indent << "brake_input_data[]" << std::endl;
    for (size_t i = 0; i < v.brake_input_data.size(); ++i)
    {
      s << indent << "  brake_input_data[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.brake_input_data[i]);
    }
    s << indent << "steering_input_data[]" << std::endl;
    for (size_t i = 0; i < v.steering_input_data.size(); ++i)
    {
      s << indent << "  steering_input_data[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.steering_input_data[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // MSG_GEN_MESSAGE_CONTROL_H
