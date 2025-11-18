// HttpClient.cpp
// Uses HTTPClient + WiFiClientSecure to POST data to the configured endpoint.

#include "drivers/HttpClient.h"

#include <Arduino.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

#include "core/Logging.h"

namespace drivers {

HttpClient::HttpClient(const char* baseUrl, const char* apiKey)
    : baseUrl_(baseUrl), apiKey_(apiKey) {}

int HttpClient::post(const std::string& body) {
  if (WiFi.status() != WL_CONNECTED) {
    return -1;
  }

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  http.useHTTP10(true);
  http.setReuse(false);
  http.setTimeout(15000);

  String url = String(baseUrl_) + "?api_key=" + apiKey_;
  if (!http.begin(client, url)) {
    return -1;
  }

  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  http.addHeader("Accept", "*/*");
  http.addHeader("User-Agent", "curl/8.0.1");
  http.addHeader("Accept-Encoding", "identity");
  http.addHeader("Connection", "close");

  core::Logger::info("POST %s", url.c_str());
  core::Logger::info("BODY: %s", body.c_str());

  int code = http.POST(body.c_str());
  String resp = http.getString();
  http.end();

  if ((code < 0) || code == 408 || (code >= 500 && code < 600) ||
      (code >= 400 && code < 500 && code != 400)) {
    core::Logger::warn("HTTP %d", code);
    if (code >= 500) {
      core::Logger::warn("%s", resp.c_str());
    }
  }

  return code;
}

}  // namespace drivers
