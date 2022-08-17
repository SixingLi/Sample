// Generated by gencpp from file msg_gen/ESimone_Vehicle_EventInfo_Type.msg
// DO NOT EDIT!


#ifndef MSG_GEN_MESSAGE_ESIMONE_VEHICLE_EVENTINFO_TYPE_H
#define MSG_GEN_MESSAGE_ESIMONE_VEHICLE_EVENTINFO_TYPE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace msg_gen
{
template <class ContainerAllocator>
struct ESimone_Vehicle_EventInfo_Type_
{
  typedef ESimone_Vehicle_EventInfo_Type_<ContainerAllocator> Type;

  ESimone_Vehicle_EventInfo_Type_()
    : SimOne_VehicleEventInfo_Type(0)  {
    }
  ESimone_Vehicle_EventInfo_Type_(const ContainerAllocator& _alloc)
    : SimOne_VehicleEventInfo_Type(0)  {
  (void)_alloc;
    }



   typedef uint32_t _SimOne_VehicleEventInfo_Type_type;
  _SimOne_VehicleEventInfo_Type_type SimOne_VehicleEventInfo_Type;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(ESimOne_VehicleEventInfo_Forward_Collision_Warning)
  #undef ESimOne_VehicleEventInfo_Forward_Collision_Warning
#endif
#if defined(_WIN32) && defined(ESimOne_VehicleEventInfo_Backward_Collision_Warning)
  #undef ESimOne_VehicleEventInfo_Backward_Collision_Warning
#endif
#if defined(_WIN32) && defined(ESimOne_VehicleEventInfo_Left_Turn_Decision)
  #undef ESimOne_VehicleEventInfo_Left_Turn_Decision
#endif
#if defined(_WIN32) && defined(ESimOne_VehicleEventInfo_Left_Turn_Warning)
  #undef ESimOne_VehicleEventInfo_Left_Turn_Warning
#endif
#if defined(_WIN32) && defined(ESimOne_VehicleEventInfo_Right_Turn_Decision)
  #undef ESimOne_VehicleEventInfo_Right_Turn_Decision
#endif
#if defined(_WIN32) && defined(ESimOne_VehicleEventInfo_Right_Turn_Warning)
  #undef ESimOne_VehicleEventInfo_Right_Turn_Warning
#endif
#if defined(_WIN32) && defined(ESimOne_VehicleEventInfo_Forward_Straight_Decision)
  #undef ESimOne_VehicleEventInfo_Forward_Straight_Decision
#endif
#if defined(_WIN32) && defined(ESimOne_VehicleEventInfo_Forward_Straight_Warning)
  #undef ESimOne_VehicleEventInfo_Forward_Straight_Warning
#endif
#if defined(_WIN32) && defined(ESimOne_VehicleEventInfo_Over_Speed_Warning)
  #undef ESimOne_VehicleEventInfo_Over_Speed_Warning
#endif
#if defined(_WIN32) && defined(ESimOne_VehicleEventInfo_Lane_Change_Decision)
  #undef ESimOne_VehicleEventInfo_Lane_Change_Decision
#endif
#if defined(_WIN32) && defined(ESimOne_VehicleEventInfo_Lane_Change_Warning)
  #undef ESimOne_VehicleEventInfo_Lane_Change_Warning
#endif
#if defined(_WIN32) && defined(ESimOne_VehicleEventInfo_Overtake_Decision)
  #undef ESimOne_VehicleEventInfo_Overtake_Decision
#endif
#if defined(_WIN32) && defined(ESimOne_VehicleEventInfo_Emergency_Braking_Decision)
  #undef ESimOne_VehicleEventInfo_Emergency_Braking_Decision
#endif
#if defined(_WIN32) && defined(ESimOne_VehicleEventInfo_Accelerate_Decision)
  #undef ESimOne_VehicleEventInfo_Accelerate_Decision
#endif

  enum {
    ESimOne_VehicleEventInfo_Forward_Collision_Warning = 0u,
    ESimOne_VehicleEventInfo_Backward_Collision_Warning = 1u,
    ESimOne_VehicleEventInfo_Left_Turn_Decision = 2u,
    ESimOne_VehicleEventInfo_Left_Turn_Warning = 3u,
    ESimOne_VehicleEventInfo_Right_Turn_Decision = 4u,
    ESimOne_VehicleEventInfo_Right_Turn_Warning = 5u,
    ESimOne_VehicleEventInfo_Forward_Straight_Decision = 6u,
    ESimOne_VehicleEventInfo_Forward_Straight_Warning = 7u,
    ESimOne_VehicleEventInfo_Over_Speed_Warning = 8u,
    ESimOne_VehicleEventInfo_Lane_Change_Decision = 9u,
    ESimOne_VehicleEventInfo_Lane_Change_Warning = 10u,
    ESimOne_VehicleEventInfo_Overtake_Decision = 11u,
    ESimOne_VehicleEventInfo_Emergency_Braking_Decision = 12u,
    ESimOne_VehicleEventInfo_Accelerate_Decision = 13u,
  };


  typedef boost::shared_ptr< ::msg_gen::ESimone_Vehicle_EventInfo_Type_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::msg_gen::ESimone_Vehicle_EventInfo_Type_<ContainerAllocator> const> ConstPtr;

}; // struct ESimone_Vehicle_EventInfo_Type_

typedef ::msg_gen::ESimone_Vehicle_EventInfo_Type_<std::allocator<void> > ESimone_Vehicle_EventInfo_Type;

typedef boost::shared_ptr< ::msg_gen::ESimone_Vehicle_EventInfo_Type > ESimone_Vehicle_EventInfo_TypePtr;
typedef boost::shared_ptr< ::msg_gen::ESimone_Vehicle_EventInfo_Type const> ESimone_Vehicle_EventInfo_TypeConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::msg_gen::ESimone_Vehicle_EventInfo_Type_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::msg_gen::ESimone_Vehicle_EventInfo_Type_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::msg_gen::ESimone_Vehicle_EventInfo_Type_<ContainerAllocator1> & lhs, const ::msg_gen::ESimone_Vehicle_EventInfo_Type_<ContainerAllocator2> & rhs)
{
  return lhs.SimOne_VehicleEventInfo_Type == rhs.SimOne_VehicleEventInfo_Type;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::msg_gen::ESimone_Vehicle_EventInfo_Type_<ContainerAllocator1> & lhs, const ::msg_gen::ESimone_Vehicle_EventInfo_Type_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace msg_gen

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::msg_gen::ESimone_Vehicle_EventInfo_Type_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::msg_gen::ESimone_Vehicle_EventInfo_Type_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::msg_gen::ESimone_Vehicle_EventInfo_Type_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::msg_gen::ESimone_Vehicle_EventInfo_Type_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::msg_gen::ESimone_Vehicle_EventInfo_Type_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::msg_gen::ESimone_Vehicle_EventInfo_Type_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::msg_gen::ESimone_Vehicle_EventInfo_Type_<ContainerAllocator> >
{
  static const char* value()
  {
    return "de5c6dd2acd804246a5121f0c3a2a7e2";
  }

  static const char* value(const ::msg_gen::ESimone_Vehicle_EventInfo_Type_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xde5c6dd2acd80424ULL;
  static const uint64_t static_value2 = 0x6a5121f0c3a2a7e2ULL;
};

template<class ContainerAllocator>
struct DataType< ::msg_gen::ESimone_Vehicle_EventInfo_Type_<ContainerAllocator> >
{
  static const char* value()
  {
    return "msg_gen/ESimone_Vehicle_EventInfo_Type";
  }

  static const char* value(const ::msg_gen::ESimone_Vehicle_EventInfo_Type_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::msg_gen::ESimone_Vehicle_EventInfo_Type_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 ESimOne_VehicleEventInfo_Forward_Collision_Warning = 0\n"
"uint8 ESimOne_VehicleEventInfo_Backward_Collision_Warning = 1\n"
"uint8 ESimOne_VehicleEventInfo_Left_Turn_Decision = 2\n"
"uint8 ESimOne_VehicleEventInfo_Left_Turn_Warning = 3\n"
"uint8 ESimOne_VehicleEventInfo_Right_Turn_Decision = 4\n"
"uint8 ESimOne_VehicleEventInfo_Right_Turn_Warning = 5\n"
"uint8 ESimOne_VehicleEventInfo_Forward_Straight_Decision = 6\n"
"uint8 ESimOne_VehicleEventInfo_Forward_Straight_Warning = 7\n"
"uint8 ESimOne_VehicleEventInfo_Over_Speed_Warning = 8\n"
"uint8 ESimOne_VehicleEventInfo_Lane_Change_Decision = 9\n"
"uint8 ESimOne_VehicleEventInfo_Lane_Change_Warning = 10\n"
"uint8 ESimOne_VehicleEventInfo_Overtake_Decision = 11\n"
"uint8 ESimOne_VehicleEventInfo_Emergency_Braking_Decision = 12\n"
"uint8 ESimOne_VehicleEventInfo_Accelerate_Decision = 13\n"
"uint32 SimOne_VehicleEventInfo_Type\n"
"\n"
;
  }

  static const char* value(const ::msg_gen::ESimone_Vehicle_EventInfo_Type_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::msg_gen::ESimone_Vehicle_EventInfo_Type_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.SimOne_VehicleEventInfo_Type);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ESimone_Vehicle_EventInfo_Type_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::msg_gen::ESimone_Vehicle_EventInfo_Type_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::msg_gen::ESimone_Vehicle_EventInfo_Type_<ContainerAllocator>& v)
  {
    s << indent << "SimOne_VehicleEventInfo_Type: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.SimOne_VehicleEventInfo_Type);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MSG_GEN_MESSAGE_ESIMONE_VEHICLE_EVENTINFO_TYPE_H
