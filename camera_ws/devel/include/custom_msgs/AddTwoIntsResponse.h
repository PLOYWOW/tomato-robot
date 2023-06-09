// Generated by gencpp from file custom_msgs/AddTwoIntsResponse.msg
// DO NOT EDIT!


#ifndef CUSTOM_MSGS_MESSAGE_ADDTWOINTSRESPONSE_H
#define CUSTOM_MSGS_MESSAGE_ADDTWOINTSRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace custom_msgs
{
template <class ContainerAllocator>
struct AddTwoIntsResponse_
{
  typedef AddTwoIntsResponse_<ContainerAllocator> Type;

  AddTwoIntsResponse_()
    : Sum(0)  {
    }
  AddTwoIntsResponse_(const ContainerAllocator& _alloc)
    : Sum(0)  {
  (void)_alloc;
    }



   typedef int64_t _Sum_type;
  _Sum_type Sum;





  typedef boost::shared_ptr< ::custom_msgs::AddTwoIntsResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::custom_msgs::AddTwoIntsResponse_<ContainerAllocator> const> ConstPtr;

}; // struct AddTwoIntsResponse_

typedef ::custom_msgs::AddTwoIntsResponse_<std::allocator<void> > AddTwoIntsResponse;

typedef boost::shared_ptr< ::custom_msgs::AddTwoIntsResponse > AddTwoIntsResponsePtr;
typedef boost::shared_ptr< ::custom_msgs::AddTwoIntsResponse const> AddTwoIntsResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::custom_msgs::AddTwoIntsResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::custom_msgs::AddTwoIntsResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::custom_msgs::AddTwoIntsResponse_<ContainerAllocator1> & lhs, const ::custom_msgs::AddTwoIntsResponse_<ContainerAllocator2> & rhs)
{
  return lhs.Sum == rhs.Sum;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::custom_msgs::AddTwoIntsResponse_<ContainerAllocator1> & lhs, const ::custom_msgs::AddTwoIntsResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace custom_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::custom_msgs::AddTwoIntsResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::custom_msgs::AddTwoIntsResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::custom_msgs::AddTwoIntsResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::custom_msgs::AddTwoIntsResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::custom_msgs::AddTwoIntsResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::custom_msgs::AddTwoIntsResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::custom_msgs::AddTwoIntsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "055d509888d173367b1b9f94491341fd";
  }

  static const char* value(const ::custom_msgs::AddTwoIntsResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x055d509888d17336ULL;
  static const uint64_t static_value2 = 0x7b1b9f94491341fdULL;
};

template<class ContainerAllocator>
struct DataType< ::custom_msgs::AddTwoIntsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "custom_msgs/AddTwoIntsResponse";
  }

  static const char* value(const ::custom_msgs::AddTwoIntsResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::custom_msgs::AddTwoIntsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int64 Sum\n"
;
  }

  static const char* value(const ::custom_msgs::AddTwoIntsResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::custom_msgs::AddTwoIntsResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.Sum);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct AddTwoIntsResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::custom_msgs::AddTwoIntsResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::custom_msgs::AddTwoIntsResponse_<ContainerAllocator>& v)
  {
    s << indent << "Sum: ";
    Printer<int64_t>::stream(s, indent + "  ", v.Sum);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CUSTOM_MSGS_MESSAGE_ADDTWOINTSRESPONSE_H
