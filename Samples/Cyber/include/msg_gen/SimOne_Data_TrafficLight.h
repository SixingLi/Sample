// Generated by gencpp from file msg_gen/SimOne_Data_TrafficLight.msg
// DO NOT EDIT!


#ifndef MSG_GEN_MESSAGE_SIMONE_DATA_TRAFFICLIGHT_H
#define MSG_GEN_MESSAGE_SIMONE_DATA_TRAFFICLIGHT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <msg_gen/SimOne_TrafficLight_Status.h>

namespace msg_gen
{
template <class ContainerAllocator>
struct SimOne_Data_TrafficLight_
{
  typedef SimOne_Data_TrafficLight_<ContainerAllocator> Type;

  SimOne_Data_TrafficLight_()
    : index(0)
    , opendriveLightId(0)
    , countDown(0)
    , status()  {
    }
  SimOne_Data_TrafficLight_(const ContainerAllocator& _alloc)
    : index(0)
    , opendriveLightId(0)
    , countDown(0)
    , status(_alloc)  {
  (void)_alloc;
    }



   typedef int32_t _index_type;
  _index_type index;

   typedef int32_t _opendriveLightId_type;
  _opendriveLightId_type opendriveLightId;

   typedef int32_t _countDown_type;
  _countDown_type countDown;

   typedef  ::msg_gen::SimOne_TrafficLight_Status_<ContainerAllocator>  _status_type;
  _status_type status;





  typedef boost::shared_ptr< ::msg_gen::SimOne_Data_TrafficLight_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::msg_gen::SimOne_Data_TrafficLight_<ContainerAllocator> const> ConstPtr;

}; // struct SimOne_Data_TrafficLight_

typedef ::msg_gen::SimOne_Data_TrafficLight_<std::allocator<void> > SimOne_Data_TrafficLight;

typedef boost::shared_ptr< ::msg_gen::SimOne_Data_TrafficLight > SimOne_Data_TrafficLightPtr;
typedef boost::shared_ptr< ::msg_gen::SimOne_Data_TrafficLight const> SimOne_Data_TrafficLightConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::msg_gen::SimOne_Data_TrafficLight_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::msg_gen::SimOne_Data_TrafficLight_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::msg_gen::SimOne_Data_TrafficLight_<ContainerAllocator1> & lhs, const ::msg_gen::SimOne_Data_TrafficLight_<ContainerAllocator2> & rhs)
{
  return lhs.index == rhs.index &&
    lhs.opendriveLightId == rhs.opendriveLightId &&
    lhs.countDown == rhs.countDown &&
    lhs.status == rhs.status;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::msg_gen::SimOne_Data_TrafficLight_<ContainerAllocator1> & lhs, const ::msg_gen::SimOne_Data_TrafficLight_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace msg_gen

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::msg_gen::SimOne_Data_TrafficLight_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::msg_gen::SimOne_Data_TrafficLight_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::msg_gen::SimOne_Data_TrafficLight_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::msg_gen::SimOne_Data_TrafficLight_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::msg_gen::SimOne_Data_TrafficLight_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::msg_gen::SimOne_Data_TrafficLight_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::msg_gen::SimOne_Data_TrafficLight_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a21e9a02f972d791144379651d7bfaae";
  }

  static const char* value(const ::msg_gen::SimOne_Data_TrafficLight_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa21e9a02f972d791ULL;
  static const uint64_t static_value2 = 0x144379651d7bfaaeULL;
};

template<class ContainerAllocator>
struct DataType< ::msg_gen::SimOne_Data_TrafficLight_<ContainerAllocator> >
{
  static const char* value()
  {
    return "msg_gen/SimOne_Data_TrafficLight";
  }

  static const char* value(const ::msg_gen::SimOne_Data_TrafficLight_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::msg_gen::SimOne_Data_TrafficLight_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 index\n"
"int32 opendriveLightId\n"
"int32 countDown\n"
"SimOne_TrafficLight_Status status\n"
"================================================================================\n"
"MSG: msg_gen/SimOne_TrafficLight_Status\n"
"uint8 ESimOne_TrafficLight_Status_Invalid = 0\n"
"uint8 ESimOne_TrafficLight_Status_Red = 1\n"
"uint8 ESimOne_TrafficLight_Status_Green = 2\n"
"uint8 ESimOne_TrafficLight_Status_Yellow = 3\n"
"uint8 ESimOne_TrafficLight_Status_RedBlink = 4\n"
"uint8 ESimOne_TrafficLight_Status_GreenBlink = 5\n"
"uint8 ESimOne_TrafficLight_Status_YellowBlink = 6\n"
"uint8 ESimOne_TrafficLight_Status_Black = 7\n"
"uint32 SimOne_TrafficLight_Status\n"
;
  }

  static const char* value(const ::msg_gen::SimOne_Data_TrafficLight_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::msg_gen::SimOne_Data_TrafficLight_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.index);
      stream.next(m.opendriveLightId);
      stream.next(m.countDown);
      stream.next(m.status);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SimOne_Data_TrafficLight_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::msg_gen::SimOne_Data_TrafficLight_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::msg_gen::SimOne_Data_TrafficLight_<ContainerAllocator>& v)
  {
    s << indent << "index: ";
    Printer<int32_t>::stream(s, indent + "  ", v.index);
    s << indent << "opendriveLightId: ";
    Printer<int32_t>::stream(s, indent + "  ", v.opendriveLightId);
    s << indent << "countDown: ";
    Printer<int32_t>::stream(s, indent + "  ", v.countDown);
    s << indent << "status: ";
    s << std::endl;
    Printer< ::msg_gen::SimOne_TrafficLight_Status_<ContainerAllocator> >::stream(s, indent + "  ", v.status);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MSG_GEN_MESSAGE_SIMONE_DATA_TRAFFICLIGHT_H