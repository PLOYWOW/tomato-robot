// Generated by gencpp from file tomato_detection/switch_camRequest.msg
// DO NOT EDIT!


#ifndef TOMATO_DETECTION_MESSAGE_SWITCH_CAMREQUEST_H
#define TOMATO_DETECTION_MESSAGE_SWITCH_CAMREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace tomato_detection
{
template <class ContainerAllocator>
struct switch_camRequest_
{
  typedef switch_camRequest_<ContainerAllocator> Type;

  switch_camRequest_()
    : req_status(false)  {
    }
  switch_camRequest_(const ContainerAllocator& _alloc)
    : req_status(false)  {
  (void)_alloc;
    }



   typedef uint8_t _req_status_type;
  _req_status_type req_status;





  typedef boost::shared_ptr< ::tomato_detection::switch_camRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::tomato_detection::switch_camRequest_<ContainerAllocator> const> ConstPtr;

}; // struct switch_camRequest_

typedef ::tomato_detection::switch_camRequest_<std::allocator<void> > switch_camRequest;

typedef boost::shared_ptr< ::tomato_detection::switch_camRequest > switch_camRequestPtr;
typedef boost::shared_ptr< ::tomato_detection::switch_camRequest const> switch_camRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::tomato_detection::switch_camRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::tomato_detection::switch_camRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::tomato_detection::switch_camRequest_<ContainerAllocator1> & lhs, const ::tomato_detection::switch_camRequest_<ContainerAllocator2> & rhs)
{
  return lhs.req_status == rhs.req_status;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::tomato_detection::switch_camRequest_<ContainerAllocator1> & lhs, const ::tomato_detection::switch_camRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace tomato_detection

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::tomato_detection::switch_camRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::tomato_detection::switch_camRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::tomato_detection::switch_camRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::tomato_detection::switch_camRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tomato_detection::switch_camRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tomato_detection::switch_camRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::tomato_detection::switch_camRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "87a8ff803de57df7fedeb6b9a9d38f8d";
  }

  static const char* value(const ::tomato_detection::switch_camRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x87a8ff803de57df7ULL;
  static const uint64_t static_value2 = 0xfedeb6b9a9d38f8dULL;
};

template<class ContainerAllocator>
struct DataType< ::tomato_detection::switch_camRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "tomato_detection/switch_camRequest";
  }

  static const char* value(const ::tomato_detection::switch_camRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::tomato_detection::switch_camRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool req_status\n"
;
  }

  static const char* value(const ::tomato_detection::switch_camRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::tomato_detection::switch_camRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.req_status);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct switch_camRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::tomato_detection::switch_camRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::tomato_detection::switch_camRequest_<ContainerAllocator>& v)
  {
    s << indent << "req_status: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.req_status);
  }
};

} // namespace message_operations
} // namespace ros

#endif // TOMATO_DETECTION_MESSAGE_SWITCH_CAMREQUEST_H