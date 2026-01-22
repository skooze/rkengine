#include "rkg/renderer_select.h"

#include <algorithm>
#include <cctype>

namespace rkg {

namespace {
std::string to_lower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

void add_unique(std::vector<std::string>& out, const std::string& value) {
  if (std::find(out.begin(), out.end(), value) == out.end()) {
    out.push_back(value);
  }
}
} // namespace

std::vector<std::string> build_renderer_fallback_order(const std::string& requested) {
  const std::string req = to_lower(requested);
  std::vector<std::string> order;
  if (req == "vulkan") {
    add_unique(order, "renderer_vulkan");
  } else if (req == "d3d12") {
    add_unique(order, "renderer_d3d12");
  } else if (req == "null") {
    add_unique(order, "renderer_null");
  }

  add_unique(order, "renderer_vulkan");
  add_unique(order, "renderer_d3d12");
  add_unique(order, "renderer_null");
  return order;
}

} // namespace rkg
