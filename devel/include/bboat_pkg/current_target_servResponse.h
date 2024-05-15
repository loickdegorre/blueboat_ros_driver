// Generated by gencpp from file bboat_pkg/current_target_servResponse.msg
// DO NOT EDIT!


#ifndef BBOAT_PKG_MESSAGE_CURRENT_TARGET_SERVRESPONSE_H
#define BBOAT_PKG_MESSAGE_CURRENT_TARGET_SERVRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Point.h>

namespace bboat_pkg
{
template <class ContainerAllocator>
struct current_target_servResponse_
{
  typedef current_target_servResponse_<ContainerAllocator> Type;

  current_target_servResponse_()
    : target()  {
    }
  current_target_servResponse_(const ContainerAllocator& _alloc)
    : target(_alloc)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _target_type;
  _target_type target;





  typedef boost::shared_ptr< ::bboat_pkg::current_target_servResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::bboat_pkg::current_target_servResponse_<ContainerAllocator> const> ConstPtr;

}; // struct current_target_servResponse_

typedef ::bboat_pkg::current_target_servResponse_<std::allocator<void> > current_target_servResponse;

typedef boost::shared_ptr< ::bboat_pkg::current_target_servResponse > current_target_servResponsePtr;
typedef boost::shared_ptr< ::bboat_pkg::current_target_servResponse const> current_target_servResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::bboat_pkg::current_target_servResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::bboat_pkg::current_target_servResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::bboat_pkg::current_target_servResponse_<ContainerAllocator1> & lhs, const ::bboat_pkg::current_target_servResponse_<ContainerAllocator2> & rhs)
{
  return lhs.target == rhs.target;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::bboat_pkg::current_target_servResponse_<ContainerAllocator1> & lhs, const ::bboat_pkg::current_target_servResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace bboat_pkg

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::bboat_pkg::current_target_servResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::bboat_pkg::current_target_servResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::bboat_pkg::current_target_servResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::bboat_pkg::current_target_servResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bboat_pkg::current_target_servResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bboat_pkg::current_target_servResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::bboat_pkg::current_target_servResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7bff3d00c7def8278f440fadd57c0653";
  }

  static const char* value(const ::bboat_pkg::current_target_servResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7bff3d00c7def827ULL;
  static const uint64_t static_value2 = 0x8f440fadd57c0653ULL;
};

template<class ContainerAllocator>
struct DataType< ::bboat_pkg::current_target_servResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bboat_pkg/current_target_servResponse";
  }

  static const char* value(const ::bboat_pkg::current_target_servResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::bboat_pkg::current_target_servResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "geometry_msgs/Point target\n"
"\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::bboat_pkg::current_target_servResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::bboat_pkg::current_target_servResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.target);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct current_target_servResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::bboat_pkg::current_target_servResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::bboat_pkg::current_target_servResponse_<ContainerAllocator>& v)
  {
    s << indent << "target: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.target);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BBOAT_PKG_MESSAGE_CURRENT_TARGET_SERVRESPONSE_H
