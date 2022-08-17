// Generated by gencpp from file msg_gen/SimOneData_Vec3f.msg
// DO NOT EDIT!


#ifndef MSG_GEN_MESSAGE_SIMONEDATA_VEC3F_H
#define MSG_GEN_MESSAGE_SIMONEDATA_VEC3F_H


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
struct SimOneData_Vec3f_
{
  typedef SimOneData_Vec3f_<ContainerAllocator> Type;

  SimOneData_Vec3f_()
    : x(0.0)
    , y(0.0)
    , z(0.0)  {
    }
  SimOneData_Vec3f_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)
    , z(0.0)  {
  (void)_alloc;
    }



   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;

   typedef float _z_type;
  _z_type z;





  typedef boost::shared_ptr< ::msg_gen::SimOneData_Vec3f_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::msg_gen::SimOneData_Vec3f_<ContainerAllocator> const> ConstPtr;

}; // struct SimOneData_Vec3f_

typedef ::msg_gen::SimOneData_Vec3f_<std::allocator<void> > SimOneData_Vec3f;

typedef boost::shared_ptr< ::msg_gen::SimOneData_Vec3f > SimOneData_Vec3fPtr;
typedef boost::shared_ptr< ::msg_gen::SimOneData_Vec3f const> SimOneData_Vec3fConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::msg_gen::SimOneData_Vec3f_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::msg_gen::SimOneData_Vec3f_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::msg_gen::SimOneData_Vec3f_<ContainerAllocator1> & lhs, const ::msg_gen::SimOneData_Vec3f_<ContainerAllocator2> & rhs)
{
  return lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.z == rhs.z;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::msg_gen::SimOneData_Vec3f_<ContainerAllocator1> & lhs, const ::msg_gen::SimOneData_Vec3f_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace msg_gen

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::msg_gen::SimOneData_Vec3f_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::msg_gen::SimOneData_Vec3f_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::msg_gen::SimOneData_Vec3f_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::msg_gen::SimOneData_Vec3f_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::msg_gen::SimOneData_Vec3f_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::msg_gen::SimOneData_Vec3f_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::msg_gen::SimOneData_Vec3f_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cc153912f1453b708d221682bc23d9ac";
  }

  static const char* value(const ::msg_gen::SimOneData_Vec3f_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xcc153912f1453b70ULL;
  static const uint64_t static_value2 = 0x8d221682bc23d9acULL;
};

template<class ContainerAllocator>
struct DataType< ::msg_gen::SimOneData_Vec3f_<ContainerAllocator> >
{
  static const char* value()
  {
    return "msg_gen/SimOneData_Vec3f";
  }

  static const char* value(const ::msg_gen::SimOneData_Vec3f_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::msg_gen::SimOneData_Vec3f_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 x\n"
"float32 y\n"
"float32 z\n"
;
  }

  static const char* value(const ::msg_gen::SimOneData_Vec3f_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::msg_gen::SimOneData_Vec3f_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SimOneData_Vec3f_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::msg_gen::SimOneData_Vec3f_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::msg_gen::SimOneData_Vec3f_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<float>::stream(s, indent + "  ", v.z);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MSG_GEN_MESSAGE_SIMONEDATA_VEC3F_H
