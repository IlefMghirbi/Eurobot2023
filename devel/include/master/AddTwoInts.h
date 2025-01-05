// Generated by gencpp from file master/AddTwoInts.msg
// DO NOT EDIT!


#ifndef MASTER_MESSAGE_ADDTWOINTS_H
#define MASTER_MESSAGE_ADDTWOINTS_H

#include <ros/service_traits.h>


#include <master/AddTwoIntsRequest.h>
#include <master/AddTwoIntsResponse.h>


namespace master
{

struct AddTwoInts
{

typedef AddTwoIntsRequest Request;
typedef AddTwoIntsResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct AddTwoInts
} // namespace master


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::master::AddTwoInts > {
  static const char* value()
  {
    return "6a2e34150c00229791cc89ff309fff21";
  }

  static const char* value(const ::master::AddTwoInts&) { return value(); }
};

template<>
struct DataType< ::master::AddTwoInts > {
  static const char* value()
  {
    return "master/AddTwoInts";
  }

  static const char* value(const ::master::AddTwoInts&) { return value(); }
};


// service_traits::MD5Sum< ::master::AddTwoIntsRequest> should match
// service_traits::MD5Sum< ::master::AddTwoInts >
template<>
struct MD5Sum< ::master::AddTwoIntsRequest>
{
  static const char* value()
  {
    return MD5Sum< ::master::AddTwoInts >::value();
  }
  static const char* value(const ::master::AddTwoIntsRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::master::AddTwoIntsRequest> should match
// service_traits::DataType< ::master::AddTwoInts >
template<>
struct DataType< ::master::AddTwoIntsRequest>
{
  static const char* value()
  {
    return DataType< ::master::AddTwoInts >::value();
  }
  static const char* value(const ::master::AddTwoIntsRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::master::AddTwoIntsResponse> should match
// service_traits::MD5Sum< ::master::AddTwoInts >
template<>
struct MD5Sum< ::master::AddTwoIntsResponse>
{
  static const char* value()
  {
    return MD5Sum< ::master::AddTwoInts >::value();
  }
  static const char* value(const ::master::AddTwoIntsResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::master::AddTwoIntsResponse> should match
// service_traits::DataType< ::master::AddTwoInts >
template<>
struct DataType< ::master::AddTwoIntsResponse>
{
  static const char* value()
  {
    return DataType< ::master::AddTwoInts >::value();
  }
  static const char* value(const ::master::AddTwoIntsResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // MASTER_MESSAGE_ADDTWOINTS_H
