// Generated by gencpp from file yocs_msgs/NavigateToResult.msg
// DO NOT EDIT!


#ifndef YOCS_MSGS_MESSAGE_NAVIGATETORESULT_H
#define YOCS_MSGS_MESSAGE_NAVIGATETORESULT_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace yocs_msgs
{
template <class ContainerAllocator>
struct NavigateToResult_
{
  typedef NavigateToResult_<ContainerAllocator> Type;

  NavigateToResult_()
    : success(false)
    , distance(0.0)
    , message()  {
    }
  NavigateToResult_(const ContainerAllocator& _alloc)
    : success(false)
    , distance(0.0)
    , message(_alloc)  {
  (void)_alloc;
    }



   typedef uint8_t _success_type;
  _success_type success;

   typedef float _distance_type;
  _distance_type distance;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _message_type;
  _message_type message;





  typedef boost::shared_ptr< ::yocs_msgs::NavigateToResult_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::yocs_msgs::NavigateToResult_<ContainerAllocator> const> ConstPtr;

}; // struct NavigateToResult_

typedef ::yocs_msgs::NavigateToResult_<std::allocator<void> > NavigateToResult;

typedef boost::shared_ptr< ::yocs_msgs::NavigateToResult > NavigateToResultPtr;
typedef boost::shared_ptr< ::yocs_msgs::NavigateToResult const> NavigateToResultConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::yocs_msgs::NavigateToResult_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::yocs_msgs::NavigateToResult_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::yocs_msgs::NavigateToResult_<ContainerAllocator1> & lhs, const ::yocs_msgs::NavigateToResult_<ContainerAllocator2> & rhs)
{
  return lhs.success == rhs.success &&
    lhs.distance == rhs.distance &&
    lhs.message == rhs.message;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::yocs_msgs::NavigateToResult_<ContainerAllocator1> & lhs, const ::yocs_msgs::NavigateToResult_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace yocs_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::yocs_msgs::NavigateToResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::yocs_msgs::NavigateToResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::yocs_msgs::NavigateToResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::yocs_msgs::NavigateToResult_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::yocs_msgs::NavigateToResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::yocs_msgs::NavigateToResult_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::yocs_msgs::NavigateToResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6ca4aa1a986f42110caccf452bbbb702";
  }

  static const char* value(const ::yocs_msgs::NavigateToResult_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6ca4aa1a986f4211ULL;
  static const uint64_t static_value2 = 0x0caccf452bbbb702ULL;
};

template<class ContainerAllocator>
struct DataType< ::yocs_msgs::NavigateToResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "yocs_msgs/NavigateToResult";
  }

  static const char* value(const ::yocs_msgs::NavigateToResult_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::yocs_msgs::NavigateToResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"# Result\n"
"bool    success\n"
"float32 distance\n"
"string  message\n"
;
  }

  static const char* value(const ::yocs_msgs::NavigateToResult_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::yocs_msgs::NavigateToResult_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.success);
      stream.next(m.distance);
      stream.next(m.message);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct NavigateToResult_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::yocs_msgs::NavigateToResult_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::yocs_msgs::NavigateToResult_<ContainerAllocator>& v)
  {
    s << indent << "success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.success);
    s << indent << "distance: ";
    Printer<float>::stream(s, indent + "  ", v.distance);
    s << indent << "message: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.message);
  }
};

} // namespace message_operations
} // namespace ros

#endif // YOCS_MSGS_MESSAGE_NAVIGATETORESULT_H
