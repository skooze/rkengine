#pragma once

#include "rkg/host_context.h"

#include <string>
#include <vector>

namespace rkg::script {

using NativeFn = void (*)(void* user_data);

struct NativeBinding {
  std::string name;
  NativeFn fn = nullptr;
  void* user_data = nullptr;
};

class IScriptRuntime {
 public:
  virtual ~IScriptRuntime() = default;
  virtual bool init(rkg::HostContext* host) = 0;
  virtual bool load_script(const std::string& source_or_path) = 0;
  virtual void tick(float dt_seconds) = 0;
  virtual void call(const std::string& fn, const std::vector<std::string>& args) = 0;
  virtual void bind_native(const NativeBinding& binding) = 0;
};

} // namespace rkg::script
