// Generated by gencpp from file msg_gen/ESimOneData_BoundaryType.msg
// DO NOT EDIT!


#ifndef MSG_GEN_MESSAGE_ESIMONEDATA_BOUNDARYTYPE_H
#define MSG_GEN_MESSAGE_ESIMONEDATA_BOUNDARYTYPE_H


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
struct ESimOneData_BoundaryType_
{
  typedef ESimOneData_BoundaryType_<ContainerAllocator> Type;

  ESimOneData_BoundaryType_()
    : BoundaryType(0)  {
    }
  ESimOneData_BoundaryType_(const ContainerAllocator& _alloc)
    : BoundaryType(0)  {
  (void)_alloc;
    }



   typedef uint32_t _BoundaryType_type;
  _BoundaryType_type BoundaryType;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(BoundaryType_none)
  #undef BoundaryType_none
#endif
#if defined(_WIN32) && defined(BoundaryType_solid)
  #undef BoundaryType_solid
#endif
#if defined(_WIN32) && defined(BoundaryType_broken)
  #undef BoundaryType_broken
#endif
#if defined(_WIN32) && defined(BoundaryType_solid_solid)
  #undef BoundaryType_solid_solid
#endif
#if defined(_WIN32) && defined(BoundaryType_solid_broken)
  #undef BoundaryType_solid_broken
#endif
#if defined(_WIN32) && defined(BoundaryType_broken_solid)
  #undef BoundaryType_broken_solid
#endif
#if defined(_WIN32) && defined(BoundaryType_broken_broken)
  #undef BoundaryType_broken_broken
#endif
#if defined(_WIN32) && defined(BoundaryType_botts_dots)
  #undef BoundaryType_botts_dots
#endif
#if defined(_WIN32) && defined(BoundaryType_grass)
  #undef BoundaryType_grass
#endif
#if defined(_WIN32) && defined(BoundaryType_curb)
  #undef BoundaryType_curb
#endif

  enum {
    BoundaryType_none = 0u,
    BoundaryType_solid = 1u,
    BoundaryType_broken = 2u,
    BoundaryType_solid_solid = 3u,
    BoundaryType_solid_broken = 4u,
    BoundaryType_broken_solid = 5u,
    BoundaryType_broken_broken = 6u,
    BoundaryType_botts_dots = 7u,
    BoundaryType_grass = 8u,
    BoundaryType_curb = 9u,
  };


  typedef boost::shared_ptr< ::msg_gen::ESimOneData_BoundaryType_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::msg_gen::ESimOneData_BoundaryType_<ContainerAllocator> const> ConstPtr;

}; // struct ESimOneData_BoundaryType_

typedef ::msg_gen::ESimOneData_BoundaryType_<std::allocator<void> > ESimOneData_BoundaryType;

typedef boost::shared_ptr< ::msg_gen::ESimOneData_BoundaryType > ESimOneData_BoundaryTypePtr;
typedef boost::shared_ptr< ::msg_gen::ESimOneData_BoundaryType const> ESimOneData_BoundaryTypeConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::msg_gen::ESimOneData_BoundaryType_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::msg_gen::ESimOneData_BoundaryType_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::msg_gen::ESimOneData_BoundaryType_<ContainerAllocator1> & lhs, const ::msg_gen::ESimOneData_BoundaryType_<ContainerAllocator2> & rhs)
{
  return lhs.BoundaryType == rhs.BoundaryType;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::msg_gen::ESimOneData_BoundaryType_<ContainerAllocator1> & lhs, const ::msg_gen::ESimOneData_BoundaryType_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace msg_gen

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::msg_gen::ESimOneData_BoundaryType_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::msg_gen::ESimOneData_BoundaryType_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::msg_gen::ESimOneData_BoundaryType_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::msg_gen::ESimOneData_BoundaryType_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::msg_gen::ESimOneData_BoundaryType_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::msg_gen::ESimOneData_BoundaryType_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::msg_gen::ESimOneData_BoundaryType_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7a1e96431f64425ee5cd2e6ff1e3ed43";
  }

  static const char* value(const ::msg_gen::ESimOneData_BoundaryType_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7a1e96431f64425eULL;
  static const uint64_t static_value2 = 0xe5cd2e6ff1e3ed43ULL;
};

template<class ContainerAllocator>
struct DataType< ::msg_gen::ESimOneData_BoundaryType_<ContainerAllocator> >
{
  static const char* value()
  {
    return "msg_gen/ESimOneData_BoundaryType";
  }

  static const char* value(const ::msg_gen::ESimOneData_BoundaryType_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::msg_gen::ESimOneData_BoundaryType_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 BoundaryType_none = 0\n"
"uint8 BoundaryType_solid = 1\n"
"uint8 BoundaryType_broken = 2\n"
"uint8 BoundaryType_solid_solid = 3\n"
"uint8 BoundaryType_solid_broken = 4\n"
"uint8 BoundaryType_broken_solid = 5\n"
"uint8 BoundaryType_broken_broken = 6\n"
"uint8 BoundaryType_botts_dots = 7\n"
"uint8 BoundaryType_grass = 8\n"
"uint8 BoundaryType_curb = 9\n"
"uint32 BoundaryType\n"
;
  }

  static const char* value(const ::msg_gen::ESimOneData_BoundaryType_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::msg_gen::ESimOneData_BoundaryType_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.BoundaryType);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ESimOneData_BoundaryType_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::msg_gen::ESimOneData_BoundaryType_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::msg_gen::ESimOneData_BoundaryType_<ContainerAllocator>& v)
  {
    s << indent << "BoundaryType: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.BoundaryType);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MSG_GEN_MESSAGE_ESIMONEDATA_BOUNDARYTYPE_H
