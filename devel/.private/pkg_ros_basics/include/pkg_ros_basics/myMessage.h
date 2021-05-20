// Generated by gencpp from file pkg_ros_basics/myMessage.msg
// DO NOT EDIT!


#ifndef PKG_ROS_BASICS_MESSAGE_MYMESSAGE_H
#define PKG_ROS_BASICS_MESSAGE_MYMESSAGE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace pkg_ros_basics
{
template <class ContainerAllocator>
struct myMessage_
{
  typedef myMessage_<ContainerAllocator> Type;

  myMessage_()
    : id(0)
    , name()
    , temperature(0.0)
    , humidity(0.0)  {
    }
  myMessage_(const ContainerAllocator& _alloc)
    : id(0)
    , name(_alloc)
    , temperature(0.0)
    , humidity(0.0)  {
  (void)_alloc;
    }



   typedef int32_t _id_type;
  _id_type id;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _name_type;
  _name_type name;

   typedef float _temperature_type;
  _temperature_type temperature;

   typedef float _humidity_type;
  _humidity_type humidity;





  typedef boost::shared_ptr< ::pkg_ros_basics::myMessage_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pkg_ros_basics::myMessage_<ContainerAllocator> const> ConstPtr;

}; // struct myMessage_

typedef ::pkg_ros_basics::myMessage_<std::allocator<void> > myMessage;

typedef boost::shared_ptr< ::pkg_ros_basics::myMessage > myMessagePtr;
typedef boost::shared_ptr< ::pkg_ros_basics::myMessage const> myMessageConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pkg_ros_basics::myMessage_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pkg_ros_basics::myMessage_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::pkg_ros_basics::myMessage_<ContainerAllocator1> & lhs, const ::pkg_ros_basics::myMessage_<ContainerAllocator2> & rhs)
{
  return lhs.id == rhs.id &&
    lhs.name == rhs.name &&
    lhs.temperature == rhs.temperature &&
    lhs.humidity == rhs.humidity;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::pkg_ros_basics::myMessage_<ContainerAllocator1> & lhs, const ::pkg_ros_basics::myMessage_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace pkg_ros_basics

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::pkg_ros_basics::myMessage_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pkg_ros_basics::myMessage_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pkg_ros_basics::myMessage_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pkg_ros_basics::myMessage_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pkg_ros_basics::myMessage_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pkg_ros_basics::myMessage_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pkg_ros_basics::myMessage_<ContainerAllocator> >
{
  static const char* value()
  {
    return "16767b4b63fd3551b69c6c06672a0bf6";
  }

  static const char* value(const ::pkg_ros_basics::myMessage_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x16767b4b63fd3551ULL;
  static const uint64_t static_value2 = 0xb69c6c06672a0bf6ULL;
};

template<class ContainerAllocator>
struct DataType< ::pkg_ros_basics::myMessage_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pkg_ros_basics/myMessage";
  }

  static const char* value(const ::pkg_ros_basics::myMessage_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pkg_ros_basics::myMessage_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 id\n"
"string name\n"
"float32 temperature\n"
"float32 humidity\n"
;
  }

  static const char* value(const ::pkg_ros_basics::myMessage_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pkg_ros_basics::myMessage_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id);
      stream.next(m.name);
      stream.next(m.temperature);
      stream.next(m.humidity);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct myMessage_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pkg_ros_basics::myMessage_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pkg_ros_basics::myMessage_<ContainerAllocator>& v)
  {
    s << indent << "id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.id);
    s << indent << "name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.name);
    s << indent << "temperature: ";
    Printer<float>::stream(s, indent + "  ", v.temperature);
    s << indent << "humidity: ";
    Printer<float>::stream(s, indent + "  ", v.humidity);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PKG_ROS_BASICS_MESSAGE_MYMESSAGE_H