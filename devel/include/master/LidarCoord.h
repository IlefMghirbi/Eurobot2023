// Generated by gencpp from file master/LidarCoord.msg
// DO NOT EDIT!


#ifndef MASTER_MESSAGE_LIDARCOORD_H
#define MASTER_MESSAGE_LIDARCOORD_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace master
{
template <class ContainerAllocator>
struct LidarCoord_
{
  typedef LidarCoord_<ContainerAllocator> Type;

  LidarCoord_()
    : x(0.0)
    , y(0.0)
    , theta(0.0)  {
    }
  LidarCoord_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)
    , theta(0.0)  {
  (void)_alloc;
    }



   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;

   typedef float _theta_type;
  _theta_type theta;





  typedef boost::shared_ptr< ::master::LidarCoord_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::master::LidarCoord_<ContainerAllocator> const> ConstPtr;

}; // struct LidarCoord_

typedef ::master::LidarCoord_<std::allocator<void> > LidarCoord;

typedef boost::shared_ptr< ::master::LidarCoord > LidarCoordPtr;
typedef boost::shared_ptr< ::master::LidarCoord const> LidarCoordConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::master::LidarCoord_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::master::LidarCoord_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::master::LidarCoord_<ContainerAllocator1> & lhs, const ::master::LidarCoord_<ContainerAllocator2> & rhs)
{
  return lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.theta == rhs.theta;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::master::LidarCoord_<ContainerAllocator1> & lhs, const ::master::LidarCoord_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace master

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::master::LidarCoord_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::master::LidarCoord_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::master::LidarCoord_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::master::LidarCoord_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::master::LidarCoord_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::master::LidarCoord_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::master::LidarCoord_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a130bc60ee6513855dc62ea83fcc5b20";
  }

  static const char* value(const ::master::LidarCoord_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa130bc60ee651385ULL;
  static const uint64_t static_value2 = 0x5dc62ea83fcc5b20ULL;
};

template<class ContainerAllocator>
struct DataType< ::master::LidarCoord_<ContainerAllocator> >
{
  static const char* value()
  {
    return "master/LidarCoord";
  }

  static const char* value(const ::master::LidarCoord_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::master::LidarCoord_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 x\n"
"float32 y\n"
"float32 theta\n"
;
  }

  static const char* value(const ::master::LidarCoord_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::master::LidarCoord_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.theta);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct LidarCoord_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::master::LidarCoord_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::master::LidarCoord_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
    s << indent << "theta: ";
    Printer<float>::stream(s, indent + "  ", v.theta);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MASTER_MESSAGE_LIDARCOORD_H
