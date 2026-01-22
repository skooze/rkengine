#include "rkg_cloud/cloud.h"

#include "rkg/log.h"

namespace rkg::cloud {

class StubObjectStorage final : public IObjectStorage {
 public:
  bool put_object(const std::string&, const std::vector<uint8_t>&) override {
    rkg::log::warn("cloud:object_storage stub");
    return false;
  }
  bool get_object(const std::string&, std::vector<uint8_t>&) override {
    rkg::log::warn("cloud:object_storage stub");
    return false;
  }
};

class StubRemoteConfig final : public IRemoteConfig {
 public:
  std::string fetch_config(const std::string&) override {
    rkg::log::warn("cloud:remote_config stub");
    return "{}";
  }
};

class StubTelemetry final : public ITelemetry {
 public:
  void emit_event(const std::string&, const std::string&) override {
    rkg::log::warn("cloud:telemetry stub");
  }
};

class StubBedrockClient final : public IBedrockClient {
 public:
  std::string invoke_model(const std::string&, const std::string&) override {
    rkg::log::warn("cloud:bedrock stub");
    return "{}";
  }
};

} // namespace rkg::cloud
