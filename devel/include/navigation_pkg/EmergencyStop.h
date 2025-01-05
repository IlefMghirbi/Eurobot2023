// Generated by gencpp from file navigation_pkg/EmergencyStop.msg
// DO NOT EDIT!


#ifndef NAVIGATION_PKG_MESSAGE_EMERGENCYSTOP_H
#define NAVIGATION_PKG_MESSAGE_EMERGENCYSTOP_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace navigation_pkg
{
template <class ContainerAllocator>
struct EmergencyStop_
{
  typedef EmergencyStop_<ContainerAllocator> Type;

  EmergencyStop_()
    : signal()  {
    }
  EmergencyStop_(const ContainerAllocator& _alloc)
    : signal(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _signal_type;
  _signal_type signal;





  typedef boost::shared_ptr< ::navigation_pkg::EmergencyStop_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::navigation_pkg::EmergencyStop_<ContainerAllocator> const> ConstPtr;

}; // struct EmergencyStop_

typedef ::navigation_pkg::EmergencyStop_<std::allocator<void> > EmergencyStop;

typedef boost::shared_ptr< ::navigation_pkg::EmergencyStop > EmergencyStopPtr;
typedef boost::shared_ptr< ::navigation_pkg::EmergencyStop const> EmergencyStopConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::navigation_pkg::EmergencyStop_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::navigation_pkg::EmergencyStop_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::navigation_pkg::EmergencyStop_<ContainerAllocator1> & lhs, const ::navigation_pkg::EmergencyStop_<ContainerAllocator2> & rhs)
{
  return lhs.signal == rhs.signal;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::navigation_pkg::EmergencyStop_<ContainerAllocator1> & lhs, const ::navigation_pkg::EmergencyStop_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace navigation_pkg

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::navigation_pkg::EmergencyStop_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::navigation_pkg::EmergencyStop_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::navigation_pkg::EmergencyStop_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::navigation_pkg::EmergencyStop_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::navigation_pkg::EmergencyStop_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::navigation_pkg::EmergencyStop_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::navigation_pkg::EmergencyStop_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9591b8ace81feee36c93130ad3fe6a19";
  }

  static const char* value(const ::navigation_pkg::EmergencyStop_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9591b8ace81feee3ULL;
  static const uint64_t static_value2 = 0x6c93130ad3fe6a19ULL;
};

template<class ContainerAllocator>
struct DataType< ::navigation_pkg::EmergencyStop_<ContainerAllocator> >
{
  static const char* value()
  {
    return "navigation_pkg/EmergencyStop";
  }

  static const char* value(const ::navigation_pkg::EmergencyStop_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::navigation_pkg::EmergencyStop_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string signal\n"
;
  }

  static const char* value(const ::navigation_pkg::EmergencyStop_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::navigation_pkg::EmergencyStop_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.signal);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct EmergencyStop_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::navigation_pkg::EmergencyStop_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::navigation_pkg::EmergencyStop_<ContainerAllocator>& v)
  {
    s << indent << "signal: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.signal);
  }
};

} // namespace message_operations
} // namespace ros

#endif // NAVIGATION_PKG_MESSAGE_EMERGENCYSTOP_H
