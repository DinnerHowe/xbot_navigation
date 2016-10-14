// Generated by gencpp from file xbot_msgs/WheelDropEvent.msg
// DO NOT EDIT!


#ifndef XBOT_MSGS_MESSAGE_WHEELDROPEVENT_H
#define XBOT_MSGS_MESSAGE_WHEELDROPEVENT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace xbot_msgs
{
template <class ContainerAllocator>
struct WheelDropEvent_
{
  typedef WheelDropEvent_<ContainerAllocator> Type;

  WheelDropEvent_()
    : wheel(0)
    , state(0)  {
    }
  WheelDropEvent_(const ContainerAllocator& _alloc)
    : wheel(0)
    , state(0)  {
  (void)_alloc;
    }



   typedef uint8_t _wheel_type;
  _wheel_type wheel;

   typedef uint8_t _state_type;
  _state_type state;


    enum { LEFT = 0u };
     enum { RIGHT = 1u };
     enum { RAISED = 0u };
     enum { DROPPED = 1u };
 

  typedef boost::shared_ptr< ::xbot_msgs::WheelDropEvent_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::xbot_msgs::WheelDropEvent_<ContainerAllocator> const> ConstPtr;

}; // struct WheelDropEvent_

typedef ::xbot_msgs::WheelDropEvent_<std::allocator<void> > WheelDropEvent;

typedef boost::shared_ptr< ::xbot_msgs::WheelDropEvent > WheelDropEventPtr;
typedef boost::shared_ptr< ::xbot_msgs::WheelDropEvent const> WheelDropEventConstPtr;

// constants requiring out of line definition

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::xbot_msgs::WheelDropEvent_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::xbot_msgs::WheelDropEvent_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace xbot_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'xbot_msgs': ['/home/howe/Xbot/src/xbot/xbot_msgs/msg', '/home/howe/Xbot/devel/share/xbot_msgs/msg'], 'actionlib_msgs': ['/opt/ros/indigo/share/actionlib_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::xbot_msgs::WheelDropEvent_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::xbot_msgs::WheelDropEvent_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::xbot_msgs::WheelDropEvent_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::xbot_msgs::WheelDropEvent_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::xbot_msgs::WheelDropEvent_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::xbot_msgs::WheelDropEvent_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::xbot_msgs::WheelDropEvent_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e102837d89384d67669a0df86b63f33b";
  }

  static const char* value(const ::xbot_msgs::WheelDropEvent_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe102837d89384d67ULL;
  static const uint64_t static_value2 = 0x669a0df86b63f33bULL;
};

template<class ContainerAllocator>
struct DataType< ::xbot_msgs::WheelDropEvent_<ContainerAllocator> >
{
  static const char* value()
  {
    return "xbot_msgs/WheelDropEvent";
  }

  static const char* value(const ::xbot_msgs::WheelDropEvent_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::xbot_msgs::WheelDropEvent_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Provides a wheel drop event.\n\
# This message is generated whenever one of the wheels is dropped (robot fell\n\
# or was raised) or raised (normal condition).\n\
# Note that, despite wheel_drop field on SensorState messages, state field is\n\
# not a bitmask, but the new state of a single sensor.\n\
\n\
# wheel\n\
uint8 LEFT  = 0\n\
uint8 RIGHT = 1\n\
\n\
# state\n\
uint8 RAISED  = 0\n\
uint8 DROPPED = 1\n\
\n\
uint8 wheel\n\
uint8 state\n\
";
  }

  static const char* value(const ::xbot_msgs::WheelDropEvent_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::xbot_msgs::WheelDropEvent_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.wheel);
      stream.next(m.state);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct WheelDropEvent_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::xbot_msgs::WheelDropEvent_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::xbot_msgs::WheelDropEvent_<ContainerAllocator>& v)
  {
    s << indent << "wheel: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.wheel);
    s << indent << "state: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.state);
  }
};

} // namespace message_operations
} // namespace ros

#endif // XBOT_MSGS_MESSAGE_WHEELDROPEVENT_H
