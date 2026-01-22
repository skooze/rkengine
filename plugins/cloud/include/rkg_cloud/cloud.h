#pragma once

#include <string>
#include <vector>

namespace rkg::cloud {

class IObjectStorage {
 public:
  virtual ~IObjectStorage() = default;
  virtual bool put_object(const std::string& key, const std::vector<uint8_t>& data) = 0;
  virtual bool get_object(const std::string& key, std::vector<uint8_t>& data) = 0;
};

class IRemoteConfig {
 public:
  virtual ~IRemoteConfig() = default;
  virtual std::string fetch_config(const std::string& path) = 0;
};

class ITelemetry {
 public:
  virtual ~ITelemetry() = default;
  virtual void emit_event(const std::string& name, const std::string& payload_json) = 0;
};

class IBedrockClient {
 public:
  virtual ~IBedrockClient() = default;
  virtual std::string invoke_model(const std::string& model_id, const std::string& payload_json) = 0;
};

} // namespace rkg::cloud
