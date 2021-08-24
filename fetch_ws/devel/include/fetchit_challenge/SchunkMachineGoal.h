// Generated by gencpp from file fetchit_challenge/SchunkMachineGoal.msg
// DO NOT EDIT!


#ifndef FETCHIT_CHALLENGE_MESSAGE_SCHUNKMACHINEGOAL_H
#define FETCHIT_CHALLENGE_MESSAGE_SCHUNKMACHINEGOAL_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace fetchit_challenge
{
template <class ContainerAllocator>
struct SchunkMachineGoal_
{
  typedef SchunkMachineGoal_<ContainerAllocator> Type;

  SchunkMachineGoal_()
    : state(0)  {
    }
  SchunkMachineGoal_(const ContainerAllocator& _alloc)
    : state(0)  {
  (void)_alloc;
    }



   typedef uint32_t _state_type;
  _state_type state;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(OPEN)
  #undef OPEN
#endif
#if defined(_WIN32) && defined(CLOSE)
  #undef CLOSE
#endif
#if defined(_WIN32) && defined(START_OPERATION)
  #undef START_OPERATION
#endif
#if defined(_WIN32) && defined(END_OPERATION)
  #undef END_OPERATION
#endif

  enum {
    OPEN = 0u,
    CLOSE = 1u,
    START_OPERATION = 2u,
    END_OPERATION = 3u,
  };


  typedef boost::shared_ptr< ::fetchit_challenge::SchunkMachineGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::fetchit_challenge::SchunkMachineGoal_<ContainerAllocator> const> ConstPtr;

}; // struct SchunkMachineGoal_

typedef ::fetchit_challenge::SchunkMachineGoal_<std::allocator<void> > SchunkMachineGoal;

typedef boost::shared_ptr< ::fetchit_challenge::SchunkMachineGoal > SchunkMachineGoalPtr;
typedef boost::shared_ptr< ::fetchit_challenge::SchunkMachineGoal const> SchunkMachineGoalConstPtr;

// constants requiring out of line definition

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::fetchit_challenge::SchunkMachineGoal_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::fetchit_challenge::SchunkMachineGoal_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::fetchit_challenge::SchunkMachineGoal_<ContainerAllocator1> & lhs, const ::fetchit_challenge::SchunkMachineGoal_<ContainerAllocator2> & rhs)
{
  return lhs.state == rhs.state;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::fetchit_challenge::SchunkMachineGoal_<ContainerAllocator1> & lhs, const ::fetchit_challenge::SchunkMachineGoal_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace fetchit_challenge

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::fetchit_challenge::SchunkMachineGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::fetchit_challenge::SchunkMachineGoal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::fetchit_challenge::SchunkMachineGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::fetchit_challenge::SchunkMachineGoal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::fetchit_challenge::SchunkMachineGoal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::fetchit_challenge::SchunkMachineGoal_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::fetchit_challenge::SchunkMachineGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9074e47af1bcfb0750a3fc3714bddb62";
  }

  static const char* value(const ::fetchit_challenge::SchunkMachineGoal_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9074e47af1bcfb07ULL;
  static const uint64_t static_value2 = 0x50a3fc3714bddb62ULL;
};

template<class ContainerAllocator>
struct DataType< ::fetchit_challenge::SchunkMachineGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "fetchit_challenge/SchunkMachineGoal";
  }

  static const char* value(const ::fetchit_challenge::SchunkMachineGoal_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::fetchit_challenge::SchunkMachineGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"uint32 OPEN=0\n"
"uint32 CLOSE=1\n"
"uint32 START_OPERATION=2\n"
"uint32 END_OPERATION=3\n"
"uint32 state \n"
;
  }

  static const char* value(const ::fetchit_challenge::SchunkMachineGoal_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::fetchit_challenge::SchunkMachineGoal_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.state);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SchunkMachineGoal_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::fetchit_challenge::SchunkMachineGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::fetchit_challenge::SchunkMachineGoal_<ContainerAllocator>& v)
  {
    s << indent << "state: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.state);
  }
};

} // namespace message_operations
} // namespace ros

#endif // FETCHIT_CHALLENGE_MESSAGE_SCHUNKMACHINEGOAL_H
