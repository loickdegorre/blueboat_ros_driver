// Generated by gencpp from file bboat_pkg/reset_lamb_serv.msg
// DO NOT EDIT!


#ifndef BBOAT_PKG_MESSAGE_RESET_LAMB_SERV_H
#define BBOAT_PKG_MESSAGE_RESET_LAMB_SERV_H

#include <ros/service_traits.h>


#include <bboat_pkg/reset_lamb_servRequest.h>
#include <bboat_pkg/reset_lamb_servResponse.h>


namespace bboat_pkg
{

struct reset_lamb_serv
{

typedef reset_lamb_servRequest Request;
typedef reset_lamb_servResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct reset_lamb_serv
} // namespace bboat_pkg


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::bboat_pkg::reset_lamb_serv > {
  static const char* value()
  {
    return "e188e0aaebb633ed14c21333921ef038";
  }

  static const char* value(const ::bboat_pkg::reset_lamb_serv&) { return value(); }
};

template<>
struct DataType< ::bboat_pkg::reset_lamb_serv > {
  static const char* value()
  {
    return "bboat_pkg/reset_lamb_serv";
  }

  static const char* value(const ::bboat_pkg::reset_lamb_serv&) { return value(); }
};


// service_traits::MD5Sum< ::bboat_pkg::reset_lamb_servRequest> should match
// service_traits::MD5Sum< ::bboat_pkg::reset_lamb_serv >
template<>
struct MD5Sum< ::bboat_pkg::reset_lamb_servRequest>
{
  static const char* value()
  {
    return MD5Sum< ::bboat_pkg::reset_lamb_serv >::value();
  }
  static const char* value(const ::bboat_pkg::reset_lamb_servRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::bboat_pkg::reset_lamb_servRequest> should match
// service_traits::DataType< ::bboat_pkg::reset_lamb_serv >
template<>
struct DataType< ::bboat_pkg::reset_lamb_servRequest>
{
  static const char* value()
  {
    return DataType< ::bboat_pkg::reset_lamb_serv >::value();
  }
  static const char* value(const ::bboat_pkg::reset_lamb_servRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::bboat_pkg::reset_lamb_servResponse> should match
// service_traits::MD5Sum< ::bboat_pkg::reset_lamb_serv >
template<>
struct MD5Sum< ::bboat_pkg::reset_lamb_servResponse>
{
  static const char* value()
  {
    return MD5Sum< ::bboat_pkg::reset_lamb_serv >::value();
  }
  static const char* value(const ::bboat_pkg::reset_lamb_servResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::bboat_pkg::reset_lamb_servResponse> should match
// service_traits::DataType< ::bboat_pkg::reset_lamb_serv >
template<>
struct DataType< ::bboat_pkg::reset_lamb_servResponse>
{
  static const char* value()
  {
    return DataType< ::bboat_pkg::reset_lamb_serv >::value();
  }
  static const char* value(const ::bboat_pkg::reset_lamb_servResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // BBOAT_PKG_MESSAGE_RESET_LAMB_SERV_H
