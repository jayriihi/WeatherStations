// IHttpClient.h
// Abstract HTTP client used to post wind data to the Google Apps Script.

#pragma once

#include <string>

namespace ports {

class IHttpClient {
 public:
  virtual ~IHttpClient() = default;
  virtual int post(const std::string& body) = 0;
};

}  // namespace ports
