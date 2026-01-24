#pragma once

namespace rkg {

namespace ecs {
class Registry;
}

namespace input {
struct ActionState;
}

namespace platform {
class Platform;
}

struct HostContext {
  platform::Platform* platform = nullptr;
  ecs::Registry* registry = nullptr;
  input::ActionState (*get_action_state)(void* user, const char* action) = nullptr;
  void* action_state_user = nullptr;
};

} // namespace rkg
