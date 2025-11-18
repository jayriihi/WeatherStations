// HttpClient.h
// Wrapper around HTTPClient for posting data to the Apps Script endpoint.

#pragma once

#include <string>

#include "ports/IHttpClient.h"

namespace drivers {

class HttpClient : public ports::IHttpClient {
 public:
  HttpClient(const char* baseUrl, const char* apiKey);
  int post(const std::string& body) override;

 private:
  const char* baseUrl_;
  const char* apiKey_;
};

}  // namespace drivers
