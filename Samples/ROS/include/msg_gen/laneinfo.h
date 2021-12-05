// Generated by gencpp from file msg_gen/laneinfo.msg
// DO NOT EDIT!


#ifndef MSG_GEN_MESSAGE_LANEINFO_H
#define MSG_GEN_MESSAGE_LANEINFO_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <msg_gen/ESimOneLaneType.h>
#include <msg_gen/SimOne_Data_LaneLineInfo.h>
#include <msg_gen/SimOne_Data_LaneLineInfo.h>
#include <msg_gen/SimOne_Data_LaneLineInfo.h>
#include <msg_gen/SimOne_Data_LaneLineInfo.h>
#include <msg_gen/SimOne_Data_LaneLineInfo.h>

namespace msg_gen
{
template <class ContainerAllocator>
struct laneinfo_
{
  typedef laneinfo_<ContainerAllocator> Type;

  laneinfo_()
    : id(0)
    , laneType()
    , laneLeftID(0)
    , laneRightID(0)
    , lanePredecessorID()
    , laneSuccessorID()
    , l_Line()
    , c_Line()
    , r_Line()
    , ll_Line()
    , rr_Line()  {
    }
  laneinfo_(const ContainerAllocator& _alloc)
    : id(0)
    , laneType(_alloc)
    , laneLeftID(0)
    , laneRightID(0)
    , lanePredecessorID(_alloc)
    , laneSuccessorID(_alloc)
    , l_Line(_alloc)
    , c_Line(_alloc)
    , r_Line(_alloc)
    , ll_Line(_alloc)
    , rr_Line(_alloc)  {
  (void)_alloc;
    }



   typedef int32_t _id_type;
  _id_type id;

   typedef  ::msg_gen::ESimOneLaneType_<ContainerAllocator>  _laneType_type;
  _laneType_type laneType;

   typedef int32_t _laneLeftID_type;
  _laneLeftID_type laneLeftID;

   typedef int32_t _laneRightID_type;
  _laneRightID_type laneRightID;

   typedef std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  _lanePredecessorID_type;
  _lanePredecessorID_type lanePredecessorID;

   typedef std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  _laneSuccessorID_type;
  _laneSuccessorID_type laneSuccessorID;

   typedef  ::msg_gen::SimOne_Data_LaneLineInfo_<ContainerAllocator>  _l_Line_type;
  _l_Line_type l_Line;

   typedef  ::msg_gen::SimOne_Data_LaneLineInfo_<ContainerAllocator>  _c_Line_type;
  _c_Line_type c_Line;

   typedef  ::msg_gen::SimOne_Data_LaneLineInfo_<ContainerAllocator>  _r_Line_type;
  _r_Line_type r_Line;

   typedef  ::msg_gen::SimOne_Data_LaneLineInfo_<ContainerAllocator>  _ll_Line_type;
  _ll_Line_type ll_Line;

   typedef  ::msg_gen::SimOne_Data_LaneLineInfo_<ContainerAllocator>  _rr_Line_type;
  _rr_Line_type rr_Line;





  typedef boost::shared_ptr< ::msg_gen::laneinfo_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::msg_gen::laneinfo_<ContainerAllocator> const> ConstPtr;

}; // struct laneinfo_

typedef ::msg_gen::laneinfo_<std::allocator<void> > laneinfo;

typedef boost::shared_ptr< ::msg_gen::laneinfo > laneinfoPtr;
typedef boost::shared_ptr< ::msg_gen::laneinfo const> laneinfoConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::msg_gen::laneinfo_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::msg_gen::laneinfo_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::msg_gen::laneinfo_<ContainerAllocator1> & lhs, const ::msg_gen::laneinfo_<ContainerAllocator2> & rhs)
{
  return lhs.id == rhs.id &&
    lhs.laneType == rhs.laneType &&
    lhs.laneLeftID == rhs.laneLeftID &&
    lhs.laneRightID == rhs.laneRightID &&
    lhs.lanePredecessorID == rhs.lanePredecessorID &&
    lhs.laneSuccessorID == rhs.laneSuccessorID &&
    lhs.l_Line == rhs.l_Line &&
    lhs.c_Line == rhs.c_Line &&
    lhs.r_Line == rhs.r_Line &&
    lhs.ll_Line == rhs.ll_Line &&
    lhs.rr_Line == rhs.rr_Line;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::msg_gen::laneinfo_<ContainerAllocator1> & lhs, const ::msg_gen::laneinfo_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace msg_gen

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::msg_gen::laneinfo_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::msg_gen::laneinfo_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::msg_gen::laneinfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::msg_gen::laneinfo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::msg_gen::laneinfo_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::msg_gen::laneinfo_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::msg_gen::laneinfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "85e7d109f09e16a90afebfc1f9b07bfc";
  }

  static const char* value(const ::msg_gen::laneinfo_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x85e7d109f09e16a9ULL;
  static const uint64_t static_value2 = 0x0afebfc1f9b07bfcULL;
};

template<class ContainerAllocator>
struct DataType< ::msg_gen::laneinfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "msg_gen/laneinfo";
  }

  static const char* value(const ::msg_gen::laneinfo_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::msg_gen::laneinfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 id\n"
"ESimOneLaneType laneType\n"
"int32 laneLeftID\n"
"int32 laneRightID\n"
"int32[] lanePredecessorID\n"
"int32[] laneSuccessorID\n"
"SimOne_Data_LaneLineInfo l_Line\n"
"SimOne_Data_LaneLineInfo c_Line\n"
"SimOne_Data_LaneLineInfo r_Line\n"
"SimOne_Data_LaneLineInfo ll_Line\n"
"SimOne_Data_LaneLineInfo rr_Line\n"
"================================================================================\n"
"MSG: msg_gen/ESimOneLaneType\n"
"uint8 LaneType_none = 0\n"
"uint8 LaneType_driving = 1\n"
"uint8 LaneType_stop = 2\n"
"uint8 LaneType_shoulder = 3\n"
"uint8 LaneType_biking = 4\n"
"uint8 LaneType_sidewalk = 5\n"
"uint8 LaneType_border = 6\n"
"uint8 LaneType_restricted = 7\n"
"uint8 LaneType_parking = 8\n"
"uint8 LaneType_bidirectional = 9\n"
"uint8 LaneType_median = 10\n"
"uint8 LaneType_special1 = 11\n"
"uint8 LaneType_special2 = 12\n"
"uint8 LaneType_special3 = 13\n"
"uint8 LaneType_roadWorks = 14\n"
"uint8 LaneType_tram = 15\n"
"uint8 LaneType_rail = 16\n"
"uint8 LaneType_entry = 17\n"
"uint8 LaneType_exit = 18\n"
"uint8 LaneType_offRamp = 19\n"
"uint8 LaneType_onRamp = 20\n"
"uint8 LaneType_mwyEntry = 21\n"
"uint8 LaneType_mwyExit = 22\n"
"uint32 LaneType\n"
"================================================================================\n"
"MSG: msg_gen/SimOne_Data_LaneLineInfo\n"
"int32 lineID\n"
"ESimOneData_BoundaryType lineType\n"
"ESimOneData_BoundaryColor lineColor\n"
"float64 linewidth\n"
"SimOneData_Vec3f[] linePoints\n"
"ESimOneData_LineCurveParameter linecurveParameter\n"
"================================================================================\n"
"MSG: msg_gen/ESimOneData_BoundaryType\n"
"uint8 BoundaryType_none = 0\n"
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
"================================================================================\n"
"MSG: msg_gen/ESimOneData_BoundaryColor\n"
"uint8 BoundaryColor_standard = 0\n"
"uint8 BoundaryColor_blue = 1\n"
"uint8 BoundaryColor_green = 2\n"
"uint8 BoundaryColor_red = 3\n"
"uint8 BoundaryColor_white = 4\n"
"uint8 BoundaryColor_yellow = 5\n"
"uint32 BoundaryColor\n"
"================================================================================\n"
"MSG: msg_gen/SimOneData_Vec3f\n"
"float32 x\n"
"float32 y\n"
"float32 z\n"
"================================================================================\n"
"MSG: msg_gen/ESimOneData_LineCurveParameter\n"
"float64 C0\n"
"float64 C1\n"
"float64 C2\n"
"float64 C3\n"
"SimOneData_Vec3f firstPoints\n"
"SimOneData_Vec3f endPoints\n"
"float64 length\n"
;
  }

  static const char* value(const ::msg_gen::laneinfo_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::msg_gen::laneinfo_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id);
      stream.next(m.laneType);
      stream.next(m.laneLeftID);
      stream.next(m.laneRightID);
      stream.next(m.lanePredecessorID);
      stream.next(m.laneSuccessorID);
      stream.next(m.l_Line);
      stream.next(m.c_Line);
      stream.next(m.r_Line);
      stream.next(m.ll_Line);
      stream.next(m.rr_Line);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct laneinfo_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::msg_gen::laneinfo_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::msg_gen::laneinfo_<ContainerAllocator>& v)
  {
    s << indent << "id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.id);
    s << indent << "laneType: ";
    s << std::endl;
    Printer< ::msg_gen::ESimOneLaneType_<ContainerAllocator> >::stream(s, indent + "  ", v.laneType);
    s << indent << "laneLeftID: ";
    Printer<int32_t>::stream(s, indent + "  ", v.laneLeftID);
    s << indent << "laneRightID: ";
    Printer<int32_t>::stream(s, indent + "  ", v.laneRightID);
    s << indent << "lanePredecessorID[]" << std::endl;
    for (size_t i = 0; i < v.lanePredecessorID.size(); ++i)
    {
      s << indent << "  lanePredecessorID[" << i << "]: ";
      Printer<int32_t>::stream(s, indent + "  ", v.lanePredecessorID[i]);
    }
    s << indent << "laneSuccessorID[]" << std::endl;
    for (size_t i = 0; i < v.laneSuccessorID.size(); ++i)
    {
      s << indent << "  laneSuccessorID[" << i << "]: ";
      Printer<int32_t>::stream(s, indent + "  ", v.laneSuccessorID[i]);
    }
    s << indent << "l_Line: ";
    s << std::endl;
    Printer< ::msg_gen::SimOne_Data_LaneLineInfo_<ContainerAllocator> >::stream(s, indent + "  ", v.l_Line);
    s << indent << "c_Line: ";
    s << std::endl;
    Printer< ::msg_gen::SimOne_Data_LaneLineInfo_<ContainerAllocator> >::stream(s, indent + "  ", v.c_Line);
    s << indent << "r_Line: ";
    s << std::endl;
    Printer< ::msg_gen::SimOne_Data_LaneLineInfo_<ContainerAllocator> >::stream(s, indent + "  ", v.r_Line);
    s << indent << "ll_Line: ";
    s << std::endl;
    Printer< ::msg_gen::SimOne_Data_LaneLineInfo_<ContainerAllocator> >::stream(s, indent + "  ", v.ll_Line);
    s << indent << "rr_Line: ";
    s << std::endl;
    Printer< ::msg_gen::SimOne_Data_LaneLineInfo_<ContainerAllocator> >::stream(s, indent + "  ", v.rr_Line);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MSG_GEN_MESSAGE_LANEINFO_H