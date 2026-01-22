#pragma once

#include <cstdint>

namespace rkg {

enum class PluginType : uint32_t {
  Unknown = 0,
  Renderer = 1,
  Scripting = 2,
  Data = 3,
  AI = 4,
  Cloud = 5,
  DebugUI = 6
};

enum class RendererCaps : uint32_t {
  None = 0,
  DrawMesh = 1 << 0,
  DrawPrimitive = 1 << 1,
  DebugUiSupported = 1 << 2
};

struct RkgPluginApi {
  uint32_t api_version;
  const char* name;
  PluginType type;
  uint32_t caps;
  bool (*init)(void* host_context);
  void (*shutdown)();
  void (*update)(float dt_seconds);
  void (*on_window_resized)(int width, int height);
};

static constexpr uint32_t kRkgPluginApiVersion = 2;

} // namespace rkg

extern "C" {
using RkgPluginApiC = rkg::RkgPluginApi;
}
