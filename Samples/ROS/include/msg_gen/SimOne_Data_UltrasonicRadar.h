// Generated by gencpp from file msg_gen/SimOne_Data_UltrasonicRadar.msg
// DO NOT EDIT!


#ifndef MSG_GEN_MESSAGE_SIMONE_DATA_ULTRASONICRADAR_H
#define MSG_GEN_MESSAGE_SIMONE_DATA_ULTRASONICRADAR_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <msg_gen/SimOne_Data_UltrasonicRadarDetection_Entry.h>

namespace msg_gen
{
template <class ContainerAllocator>
struct SimOne_Data_UltrasonicRadar_
{
  typedef SimOne_Data_UltrasonicRadar_<ContainerAllocator> Type;

  SimOne_Data_UltrasonicRadar_()
    : id(0)
    , obstacleNum(0)
    , obstacledetections()  {
    }
  SimOne_Data_UltrasonicRadar_(const ContainerAllocator& _alloc)
    : id(0)
    , obstacleNum(0)
    , obstacledetections(_alloc)  {
  (void)_alloc;
    }



   typedef int32_t _id_type;
  _id_type id;

   typedef int32_t _obstacleNum_type;
  _obstacleNum_type obstacleNum;

   typedef std::vector< ::msg_gen::SimOne_Data_UltrasonicRadarDetection_Entry_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::msg_gen::SimOne_Data_UltrasonicRadarDetection_Entry_<ContainerAllocator> >::other >  _obstacledetections_type;
  _obstacledetections_type obstacledetections;





  typedef boost::shared_ptr< ::msg_gen::SimOne_Data_UltrasonicRadar_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::msg_gen::SimOne_Data_UltrasonicRadar_<ContainerAllocator> const> ConstPtr;

}; // struct SimOne_Data_UltrasonicRadar_

typedef ::msg_gen::SimOne_Data_UltrasonicRadar_<std::allocator<void> > SimOne_Data_UltrasonicRadar;

typedef boost::shared_ptr< ::msg_gen::SimOne_Data_UltrasonicRadar > SimOne_Data_UltrasonicRadarPtr;
typedef boost::shared_ptr< ::msg_gen::SimOne_Data_UltrasonicRadar const> SimOne_Data_UltrasonicRadarConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::msg_gen::SimOne_Data_UltrasonicRadar_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::msg_gen::SimOne_Data_UltrasonicRadar_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::msg_gen::SimOne_Data_UltrasonicRadar_<ContainerAllocator1> & lhs, const ::msg_gen::SimOne_Data_UltrasonicRadar_<ContainerAllocator2> & rhs)
{
  return lhs.id == rhs.id &&
    lhs.obstacleNum == rhs.obstacleNum &&
    lhs.obstacledetections == rhs.obstacledetections;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::msg_gen::SimOne_Data_UltrasonicRadar_<ContainerAllocator1> & lhs, const ::msg_gen::SimOne_Data_UltrasonicRadar_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace msg_gen

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::msg_gen::SimOne_Data_UltrasonicRadar_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::msg_gen::SimOne_Data_UltrasonicRadar_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::msg_gen::SimOne_Data_UltrasonicRadar_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::msg_gen::SimOne_Data_UltrasonicRadar_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::msg_gen::SimOne_Data_UltrasonicRadar_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::msg_gen::SimOne_Data_UltrasonicRadar_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::msg_gen::SimOne_Data_UltrasonicRadar_<ContainerAllocator> >
{
  static const char* value()
  {
    return "68f8a894aaf87f17686c6108cf74440e";
  }

  static const char* value(const ::msg_gen::SimOne_Data_UltrasonicRadar_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x68f8a894aaf87f17ULL;
  static const uint64_t static_value2 = 0x686c6108cf74440eULL;
};

template<class ContainerAllocator>
struct DataType< ::msg_gen::SimOne_Data_UltrasonicRadar_<ContainerAllocator> >
{
  static const char* value()
  {
    return "msg_gen/SimOne_Data_UltrasonicRadar";
  }

  static const char* value(const ::msg_gen::SimOne_Data_UltrasonicRadar_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::msg_gen::SimOne_Data_UltrasonicRadar_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 id\n"
"int32 obstacleNum\n"
"SimOne_Data_UltrasonicRadarDetection_Entry[] obstacledetections\n"
"================================================================================\n"
"MSG: msg_gen/SimOne_Data_UltrasonicRadarDetection_Entry\n"
"float64 obstacleRanges\n"
"float64 x\n"
"float64 y\n"
;
  }

  static const char* value(const ::msg_gen::SimOne_Data_UltrasonicRadar_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::msg_gen::SimOne_Data_UltrasonicRadar_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id);
      stream.next(m.obstacleNum);
      stream.next(m.obstacledetections);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SimOne_Data_UltrasonicRadar_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::msg_gen::SimOne_Data_UltrasonicRadar_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::msg_gen::SimOne_Data_UltrasonicRadar_<ContainerAllocator>& v)
  {
    s << indent << "id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.id);
    s << indent << "obstacleNum: ";
    Printer<int32_t>::stream(s, indent + "  ", v.obstacleNum);
    s << indent << "obstacledetections[]" << std::endl;
    for (size_t i = 0; i < v.obstacledetections.size(); ++i)
    {
      s << indent << "  obstacledetections[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::msg_gen::SimOne_Data_UltrasonicRadarDetection_Entry_<ContainerAllocator> >::stream(s, indent + "    ", v.obstacledetections[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // MSG_GEN_MESSAGE_SIMONE_DATA_ULTRASONICRADAR_H