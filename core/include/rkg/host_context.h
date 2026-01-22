#pragma once

namespace rkg {

namespace ecs {
class Registry;
}

namespace platform {
class Platform;
}

struct HostContext {
  platform::Platform* platform = nullptr;
  ecs::Registry* registry = nullptr;
};

} // namespace rkg
