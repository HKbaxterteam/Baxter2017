// Generated by gencpp from file baxter_gui/send_commandResponse.msg
// DO NOT EDIT!


#ifndef BAXTER_GUI_MESSAGE_SEND_COMMANDRESPONSE_H
#define BAXTER_GUI_MESSAGE_SEND_COMMANDRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace baxter_gui
{
template <class ContainerAllocator>
struct send_commandResponse_
{
  typedef send_commandResponse_<ContainerAllocator> Type;

  send_commandResponse_()
    : answ()  {
    }
  send_commandResponse_(const ContainerAllocator& _alloc)
    : answ(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _answ_type;
  _answ_type answ;




  typedef boost::shared_ptr< ::baxter_gui::send_commandResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::baxter_gui::send_commandResponse_<ContainerAllocator> const> ConstPtr;

}; // struct send_commandResponse_

typedef ::baxter_gui::send_commandResponse_<std::allocator<void> > send_commandResponse;

typedef boost::shared_ptr< ::baxter_gui::send_commandResponse > send_commandResponsePtr;
typedef boost::shared_ptr< ::baxter_gui::send_commandResponse const> send_commandResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::baxter_gui::send_commandResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::baxter_gui::send_commandResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace baxter_gui

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::baxter_gui::send_commandResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::baxter_gui::send_commandResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::baxter_gui::send_commandResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::baxter_gui::send_commandResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::baxter_gui::send_commandResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::baxter_gui::send_commandResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::baxter_gui::send_commandResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1fe96119698f6053678ed113fa62db6e";
  }

  static const char* value(const ::baxter_gui::send_commandResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1fe96119698f6053ULL;
  static const uint64_t static_value2 = 0x678ed113fa62db6eULL;
};

template<class ContainerAllocator>
struct DataType< ::baxter_gui::send_commandResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "baxter_gui/send_commandResponse";
  }

  static const char* value(const ::baxter_gui::send_commandResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::baxter_gui::send_commandResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string answ\n\
\n\
";
  }

  static const char* value(const ::baxter_gui::send_commandResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::baxter_gui::send_commandResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.answ);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct send_commandResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::baxter_gui::send_commandResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::baxter_gui::send_commandResponse_<ContainerAllocator>& v)
  {
    s << indent << "answ: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.answ);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BAXTER_GUI_MESSAGE_SEND_COMMANDRESPONSE_H