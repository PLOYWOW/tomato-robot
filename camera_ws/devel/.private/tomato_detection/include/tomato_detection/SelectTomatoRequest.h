// Generated by gencpp from file tomato_detection/SelectTomatoRequest.msg
// DO NOT EDIT!


#ifndef TOMATO_DETECTION_MESSAGE_SELECTTOMATOREQUEST_H
#define TOMATO_DETECTION_MESSAGE_SELECTTOMATOREQUEST_H


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
struct SelectTomatoRequest_
{
  typedef SelectTomatoRequest_<ContainerAllocator> Type;

  SelectTomatoRequest_()
    : gripperPos()  {
    }
  SelectTomatoRequest_(const ContainerAllocator& _alloc)
    : gripperPos(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> _gripperPos_type;
  _gripperPos_type gripperPos;





  typedef boost::shared_ptr< ::tomato_detection::SelectTomatoRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::tomato_detection::SelectTomatoRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SelectTomatoRequest_

typedef ::tomato_detection::SelectTomatoRequest_<std::allocator<void> > SelectTomatoRequest;

typedef boost::shared_ptr< ::tomato_detection::SelectTomatoRequest > SelectTomatoRequestPtr;
typedef boost::shared_ptr< ::tomato_detection::SelectTomatoRequest const> SelectTomatoRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::tomato_detection::SelectTomatoRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::tomato_detection::SelectTomatoRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::tomato_detection::SelectTomatoRequest_<ContainerAllocator1> & lhs, const ::tomato_detection::SelectTomatoRequest_<ContainerAllocator2> & rhs)
{
  return lhs.gripperPos == rhs.gripperPos;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::tomato_detection::SelectTomatoRequest_<ContainerAllocator1> & lhs, const ::tomato_detection::SelectTomatoRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace tomato_detection

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::tomato_detection::SelectTomatoRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::tomato_detection::SelectTomatoRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::tomato_detection::SelectTomatoRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::tomato_detection::SelectTomatoRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tomato_detection::SelectTomatoRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tomato_detection::SelectTomatoRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::tomato_detection::SelectTomatoRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5b4a1966605e01c4d22aa4278d4a6590";
  }

  static const char* value(const ::tomato_detection::SelectTomatoRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5b4a1966605e01c4ULL;
  static const uint64_t static_value2 = 0xd22aa4278d4a6590ULL;
};

template<class ContainerAllocator>
struct DataType< ::tomato_detection::SelectTomatoRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "tomato_detection/SelectTomatoRequest";
  }

  static const char* value(const ::tomato_detection::SelectTomatoRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::tomato_detection::SelectTomatoRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32[] gripperPos\n"
;
  }

  static const char* value(const ::tomato_detection::SelectTomatoRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::tomato_detection::SelectTomatoRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.gripperPos);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SelectTomatoRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::tomato_detection::SelectTomatoRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::tomato_detection::SelectTomatoRequest_<ContainerAllocator>& v)
  {
    s << indent << "gripperPos[]" << std::endl;
    for (size_t i = 0; i < v.gripperPos.size(); ++i)
    {
      s << indent << "  gripperPos[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.gripperPos[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // TOMATO_DETECTION_MESSAGE_SELECTTOMATOREQUEST_H
