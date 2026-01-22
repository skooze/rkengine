#pragma once

#include <cstdint>
#include <string>

namespace rkg::rhi {

struct SwapchainDesc {
  uint32_t width = 0;
  uint32_t height = 0;
  uint32_t image_count = 2;
};

class Device {
 public:
  virtual ~Device() = default;
};

class Swapchain {
 public:
  virtual ~Swapchain() = default;
};

class CommandList {
 public:
  virtual ~CommandList() = default;
};

class Buffer {
 public:
  virtual ~Buffer() = default;
};

class Texture {
 public:
  virtual ~Texture() = default;
};

class Shader {
 public:
  virtual ~Shader() = default;
  virtual std::string name() const = 0;
};

} // namespace rkg::rhi
