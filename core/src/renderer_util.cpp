#include "rkg/renderer_util.h"

namespace rkg {

std::string renderer_display_name(const std::string& plugin_name) {
  if (plugin_name == "renderer_vulkan") return "Vulkan";
  if (plugin_name == "renderer_d3d12") return "D3D12";
  if (plugin_name == "renderer_null") return "Null";
  return plugin_name.empty() ? "Unknown" : plugin_name;
}

} // namespace rkg
