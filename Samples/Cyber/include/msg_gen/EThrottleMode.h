// Generated by gencpp from file msg_gen/EThrottleMode.msg
// DO NOT EDIT!


#ifndef MSG_GEN_MESSAGE_ETHROTTLEMODE_H
#define MSG_GEN_MESSAGE_ETHROTTLEMODE_H


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
struct EThrottleMode_
{
  typedef EThrottleMode_<ContainerAllocator> Type;

  EThrottleMode_()
    : ThrottleMode(0)  {
    }
  EThrottleMode_(const ContainerAllocator& _alloc)
    : ThrottleMode(0)  {
  (void)_alloc;
    }



   typedef uint32_t _ThrottleMode_type;
  _ThrottleMode_type ThrottleMode;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(EThrottleMode_Percent)
  #undef EThrottleMode_Percent
#endif
#if defined(_WIN32) && defined(EThrottleMode_Torque)
  #undef EThrottleMode_Torque
#endif
#if defined(_WIN32) && defined(EThrottleMode_Speed)
  #undef EThrottleMode_Speed
#endif
#if defined(_WIN32) && defined(EThrottleMode_Accel)
  #undef EThrottleMode_Accel
#endif
#if defined(_WIN32) && defined(EThrottleMode_EngineAV)
  #undef EThrottleMode_EngineAV
#endif
#if defined(_WIN32) && defined(EThrottleMode_WheelTorque)
  #undef EThrottleMode_WheelTorque
#endif

  enum {
    EThrottleMode_Percent = 0u,
    EThrottleMode_Torque = 1u,
    EThrottleMode_Speed = 2u,
    EThrottleMode_Accel = 3u,
    EThrottleMode_EngineAV = 4u,
    EThrottleMode_WheelTorque = 5u,
  };


  typedef boost::shared_ptr< ::msg_gen::EThrottleMode_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::msg_gen::EThrottleMode_<ContainerAllocator> const> ConstPtr;

}; // struct EThrottleMode_

typedef ::msg_gen::EThrottleMode_<std::allocator<void> > EThrottleMode;

typedef boost::shared_ptr< ::msg_gen::EThrottleMode > EThrottleModePtr;
typedef boost::shared_ptr< ::msg_gen::EThrottleMode const> EThrottleModeConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::msg_gen::EThrottleMode_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::msg_gen::EThrottleMode_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::msg_gen::EThrottleMode_<ContainerAllocator1> & lhs, const ::msg_gen::EThrottleMode_<ContainerAllocator2> & rhs)
{
  return lhs.ThrottleMode == rhs.ThrottleMode;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::msg_gen::EThrottleMode_<ContainerAllocator1> & lhs, const ::msg_gen::EThrottleMode_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace msg_gen

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::msg_gen::EThrottleMode_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::msg_gen::EThrottleMode_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::msg_gen::EThrottleMode_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::msg_gen::EThrottleMode_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::msg_gen::EThrottleMode_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::msg_gen::EThrottleMode_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::msg_gen::EThrottleMode_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5b76285d5fb9a925c43a6df81a882007";
  }

  static const char* value(const ::msg_gen::EThrottleMode_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5b76285d5fb9a925ULL;
  static const uint64_t static_value2 = 0xc43a6df81a882007ULL;
};

template<class ContainerAllocator>
struct DataType< ::msg_gen::EThrottleMode_<ContainerAllocator> >
{
  static const char* value()
  {
    return "msg_gen/EThrottleMode";
  }

  static const char* value(const ::msg_gen::EThrottleMode_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::msg_gen::EThrottleMode_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 EThrottleMode_Percent = 0\n"
"uint8 EThrottleMode_Torque = 1\n"
"uint8 EThrottleMode_Speed = 2\n"
"uint8 EThrottleMode_Accel = 3\n"
"uint8 EThrottleMode_EngineAV = 4\n"
"uint8 EThrottleMode_WheelTorque = 5\n"
"uint32 ThrottleMode\n"
;
  }

  static const char* value(const ::msg_gen::EThrottleMode_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::msg_gen::EThrottleMode_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.ThrottleMode);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct EThrottleMode_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::msg_gen::EThrottleMode_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::msg_gen::EThrottleMode_<ContainerAllocator>& v)
  {
    s << indent << "ThrottleMode: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.ThrottleMode);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MSG_GEN_MESSAGE_ETHROTTLEMODE_H
