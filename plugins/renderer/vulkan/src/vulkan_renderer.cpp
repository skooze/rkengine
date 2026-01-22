#include "rkg/host_context.h"
#include "rkg/log.h"
#include "rkg/plugin_api.h"
#include "rkg/renderer_hooks.h"
#include "rkg_platform/platform.h"

#if RKG_ENABLE_IMGUI
#include "rkg_debug_ui/imgui_api.h"
#endif
#include <SDL3/SDL.h>
#include <SDL3/SDL_vulkan.h>
#include <vulkan/vulkan.h>

#include <algorithm>
#include <vector>

namespace {

struct VulkanState {
  rkg::HostContext* ctx = nullptr;
  SDL_Window* window = nullptr;
  VkInstance instance = VK_NULL_HANDLE;
  VkSurfaceKHR surface = VK_NULL_HANDLE;
  VkPhysicalDevice physical_device = VK_NULL_HANDLE;
  VkDevice device = VK_NULL_HANDLE;
  VkQueue graphics_queue = VK_NULL_HANDLE;
  VkQueue present_queue = VK_NULL_HANDLE;
  uint32_t graphics_queue_family = 0;
  uint32_t present_queue_family = 0;
  VkSwapchainKHR swapchain = VK_NULL_HANDLE;
  VkFormat swapchain_format = VK_FORMAT_UNDEFINED;
  VkExtent2D swapchain_extent{};
  std::vector<VkImage> swapchain_images;
  std::vector<VkImageView> swapchain_image_views;
  VkRenderPass render_pass = VK_NULL_HANDLE;
  std::vector<VkFramebuffer> framebuffers;
  VkCommandPool command_pool = VK_NULL_HANDLE;
  std::vector<VkCommandBuffer> command_buffers;
  VkSemaphore image_available = VK_NULL_HANDLE;
  VkSemaphore render_finished = VK_NULL_HANDLE;
  VkFence in_flight_fence = VK_NULL_HANDLE;
};

VulkanState g_state{};

bool create_instance() {
  uint32_t ext_count = 0;
  if (!SDL_Vulkan_GetInstanceExtensions(g_state.window, &ext_count, nullptr)) {
    rkg::log::error("SDL_Vulkan_GetInstanceExtensions failed");
    return false;
  }
  std::vector<const char*> extensions(ext_count);
  if (!SDL_Vulkan_GetInstanceExtensions(g_state.window, &ext_count, extensions.data())) {
    rkg::log::error("SDL_Vulkan_GetInstanceExtensions failed");
    return false;
  }

  VkApplicationInfo app_info{};
  app_info.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
  app_info.pApplicationName = "rkg_minimal_app";
  app_info.apiVersion = VK_API_VERSION_1_1;

  VkInstanceCreateInfo create_info{};
  create_info.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
  create_info.pApplicationInfo = &app_info;
  create_info.enabledExtensionCount = ext_count;
  create_info.ppEnabledExtensionNames = extensions.data();

  if (vkCreateInstance(&create_info, nullptr, &g_state.instance) != VK_SUCCESS) {
    rkg::log::error("vkCreateInstance failed");
    return false;
  }
  return true;
}

bool create_surface() {
  if (!SDL_Vulkan_CreateSurface(g_state.window, g_state.instance, &g_state.surface)) {
    rkg::log::error("SDL_Vulkan_CreateSurface failed");
    return false;
  }
  return true;
}

bool pick_physical_device() {
  uint32_t device_count = 0;
  vkEnumeratePhysicalDevices(g_state.instance, &device_count, nullptr);
  if (device_count == 0) {
    rkg::log::error("No Vulkan physical devices found");
    return false;
  }
  std::vector<VkPhysicalDevice> devices(device_count);
  vkEnumeratePhysicalDevices(g_state.instance, &device_count, devices.data());
  g_state.physical_device = devices[0];
  return true;
}

bool find_queue_families() {
  uint32_t queue_family_count = 0;
  vkGetPhysicalDeviceQueueFamilyProperties(g_state.physical_device, &queue_family_count, nullptr);
  std::vector<VkQueueFamilyProperties> families(queue_family_count);
  vkGetPhysicalDeviceQueueFamilyProperties(g_state.physical_device, &queue_family_count, families.data());

  bool found_graphics = false;
  bool found_present = false;
  for (uint32_t i = 0; i < queue_family_count; ++i) {
    if (families[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) {
      g_state.graphics_queue_family = i;
      found_graphics = true;
    }
    VkBool32 present_supported = VK_FALSE;
    vkGetPhysicalDeviceSurfaceSupportKHR(g_state.physical_device, i, g_state.surface, &present_supported);
    if (present_supported) {
      g_state.present_queue_family = i;
      found_present = true;
    }
    if (found_graphics && found_present) {
      return true;
    }
  }
  return false;
}

bool create_device() {
  float queue_priority = 1.0f;
  std::vector<VkDeviceQueueCreateInfo> queue_infos;
  VkDeviceQueueCreateInfo graphics_queue{};
  graphics_queue.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
  graphics_queue.queueFamilyIndex = g_state.graphics_queue_family;
  graphics_queue.queueCount = 1;
  graphics_queue.pQueuePriorities = &queue_priority;
  queue_infos.push_back(graphics_queue);

  if (g_state.present_queue_family != g_state.graphics_queue_family) {
    VkDeviceQueueCreateInfo present_queue{};
    present_queue.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
    present_queue.queueFamilyIndex = g_state.present_queue_family;
    present_queue.queueCount = 1;
    present_queue.pQueuePriorities = &queue_priority;
    queue_infos.push_back(present_queue);
  }

  const char* extensions[] = {VK_KHR_SWAPCHAIN_EXTENSION_NAME};
  VkDeviceCreateInfo create_info{};
  create_info.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
  create_info.queueCreateInfoCount = static_cast<uint32_t>(queue_infos.size());
  create_info.pQueueCreateInfos = queue_infos.data();
  create_info.enabledExtensionCount = 1;
  create_info.ppEnabledExtensionNames = extensions;

  if (vkCreateDevice(g_state.physical_device, &create_info, nullptr, &g_state.device) != VK_SUCCESS) {
    rkg::log::error("vkCreateDevice failed");
    return false;
  }

  vkGetDeviceQueue(g_state.device, g_state.graphics_queue_family, 0, &g_state.graphics_queue);
  vkGetDeviceQueue(g_state.device, g_state.present_queue_family, 0, &g_state.present_queue);
  return true;
}

VkSurfaceFormatKHR choose_surface_format(const std::vector<VkSurfaceFormatKHR>& formats) {
  for (const auto& fmt : formats) {
    if (fmt.format == VK_FORMAT_B8G8R8A8_SRGB && fmt.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) {
      return fmt;
    }
  }
  return formats[0];
}

VkPresentModeKHR choose_present_mode(const std::vector<VkPresentModeKHR>& modes) {
  for (const auto& mode : modes) {
    if (mode == VK_PRESENT_MODE_MAILBOX_KHR) {
      return mode;
    }
  }
  return VK_PRESENT_MODE_FIFO_KHR;
}

VkExtent2D choose_extent(const VkSurfaceCapabilitiesKHR& caps) {
  if (caps.currentExtent.width != UINT32_MAX) {
    return caps.currentExtent;
  }
  int w = 0;
  int h = 0;
  SDL_GetWindowSize(g_state.window, &w, &h);
  VkExtent2D extent{};
  extent.width = static_cast<uint32_t>(w);
  extent.height = static_cast<uint32_t>(h);
  extent.width = std::max(caps.minImageExtent.width, std::min(caps.maxImageExtent.width, extent.width));
  extent.height = std::max(caps.minImageExtent.height, std::min(caps.maxImageExtent.height, extent.height));
  return extent;
}

bool create_swapchain() {
  VkSurfaceCapabilitiesKHR caps{};
  vkGetPhysicalDeviceSurfaceCapabilitiesKHR(g_state.physical_device, g_state.surface, &caps);

  uint32_t format_count = 0;
  vkGetPhysicalDeviceSurfaceFormatsKHR(g_state.physical_device, g_state.surface, &format_count, nullptr);
  std::vector<VkSurfaceFormatKHR> formats(format_count);
  vkGetPhysicalDeviceSurfaceFormatsKHR(g_state.physical_device, g_state.surface, &format_count, formats.data());

  uint32_t present_mode_count = 0;
  vkGetPhysicalDeviceSurfacePresentModesKHR(g_state.physical_device, g_state.surface, &present_mode_count, nullptr);
  std::vector<VkPresentModeKHR> present_modes(present_mode_count);
  vkGetPhysicalDeviceSurfacePresentModesKHR(g_state.physical_device, g_state.surface, &present_mode_count,
                                            present_modes.data());

  VkSurfaceFormatKHR surface_format = choose_surface_format(formats);
  VkPresentModeKHR present_mode = choose_present_mode(present_modes);
  VkExtent2D extent = choose_extent(caps);

  uint32_t image_count = caps.minImageCount + 1;
  if (caps.maxImageCount > 0 && image_count > caps.maxImageCount) {
    image_count = caps.maxImageCount;
  }

  VkSwapchainCreateInfoKHR create_info{};
  create_info.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
  create_info.surface = g_state.surface;
  create_info.minImageCount = image_count;
  create_info.imageFormat = surface_format.format;
  create_info.imageColorSpace = surface_format.colorSpace;
  create_info.imageExtent = extent;
  create_info.imageArrayLayers = 1;
  create_info.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;

  uint32_t queue_family_indices[] = {g_state.graphics_queue_family, g_state.present_queue_family};
  if (g_state.graphics_queue_family != g_state.present_queue_family) {
    create_info.imageSharingMode = VK_SHARING_MODE_CONCURRENT;
    create_info.queueFamilyIndexCount = 2;
    create_info.pQueueFamilyIndices = queue_family_indices;
  } else {
    create_info.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
  }

  create_info.preTransform = caps.currentTransform;
  create_info.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
  create_info.presentMode = present_mode;
  create_info.clipped = VK_TRUE;

  if (vkCreateSwapchainKHR(g_state.device, &create_info, nullptr, &g_state.swapchain) != VK_SUCCESS) {
    rkg::log::error("vkCreateSwapchainKHR failed");
    return false;
  }

  g_state.swapchain_format = surface_format.format;
  g_state.swapchain_extent = extent;

  uint32_t swapchain_image_count = 0;
  vkGetSwapchainImagesKHR(g_state.device, g_state.swapchain, &swapchain_image_count, nullptr);
  g_state.swapchain_images.resize(swapchain_image_count);
  vkGetSwapchainImagesKHR(g_state.device, g_state.swapchain, &swapchain_image_count,
                          g_state.swapchain_images.data());

  g_state.swapchain_image_views.resize(swapchain_image_count);
  for (size_t i = 0; i < g_state.swapchain_images.size(); ++i) {
    VkImageViewCreateInfo view_info{};
    view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    view_info.image = g_state.swapchain_images[i];
    view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
    view_info.format = g_state.swapchain_format;
    view_info.components.r = VK_COMPONENT_SWIZZLE_IDENTITY;
    view_info.components.g = VK_COMPONENT_SWIZZLE_IDENTITY;
    view_info.components.b = VK_COMPONENT_SWIZZLE_IDENTITY;
    view_info.components.a = VK_COMPONENT_SWIZZLE_IDENTITY;
    view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    view_info.subresourceRange.baseMipLevel = 0;
    view_info.subresourceRange.levelCount = 1;
    view_info.subresourceRange.baseArrayLayer = 0;
    view_info.subresourceRange.layerCount = 1;

    if (vkCreateImageView(g_state.device, &view_info, nullptr, &g_state.swapchain_image_views[i]) != VK_SUCCESS) {
      rkg::log::error("vkCreateImageView failed");
      return false;
    }
  }

  return true;
}

bool create_render_pass() {
  VkAttachmentDescription color_attachment{};
  color_attachment.format = g_state.swapchain_format;
  color_attachment.samples = VK_SAMPLE_COUNT_1_BIT;
  color_attachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
  color_attachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
  color_attachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
  color_attachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
  color_attachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  color_attachment.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

  VkAttachmentReference color_attachment_ref{};
  color_attachment_ref.attachment = 0;
  color_attachment_ref.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

  VkSubpassDescription subpass{};
  subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
  subpass.colorAttachmentCount = 1;
  subpass.pColorAttachments = &color_attachment_ref;

  VkSubpassDependency dependency{};
  dependency.srcSubpass = VK_SUBPASS_EXTERNAL;
  dependency.dstSubpass = 0;
  dependency.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
  dependency.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
  dependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

  VkRenderPassCreateInfo render_pass_info{};
  render_pass_info.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
  render_pass_info.attachmentCount = 1;
  render_pass_info.pAttachments = &color_attachment;
  render_pass_info.subpassCount = 1;
  render_pass_info.pSubpasses = &subpass;
  render_pass_info.dependencyCount = 1;
  render_pass_info.pDependencies = &dependency;

  if (vkCreateRenderPass(g_state.device, &render_pass_info, nullptr, &g_state.render_pass) != VK_SUCCESS) {
    rkg::log::error("vkCreateRenderPass failed");
    return false;
  }
  return true;
}

bool create_framebuffers() {
  g_state.framebuffers.resize(g_state.swapchain_image_views.size());
  for (size_t i = 0; i < g_state.swapchain_image_views.size(); ++i) {
    VkImageView attachments[] = {g_state.swapchain_image_views[i]};
    VkFramebufferCreateInfo framebuffer_info{};
    framebuffer_info.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
    framebuffer_info.renderPass = g_state.render_pass;
    framebuffer_info.attachmentCount = 1;
    framebuffer_info.pAttachments = attachments;
    framebuffer_info.width = g_state.swapchain_extent.width;
    framebuffer_info.height = g_state.swapchain_extent.height;
    framebuffer_info.layers = 1;

    if (vkCreateFramebuffer(g_state.device, &framebuffer_info, nullptr, &g_state.framebuffers[i]) != VK_SUCCESS) {
      rkg::log::error("vkCreateFramebuffer failed");
      return false;
    }
  }
  return true;
}

bool create_command_pool() {
  VkCommandPoolCreateInfo pool_info{};
  pool_info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
  pool_info.queueFamilyIndex = g_state.graphics_queue_family;
  pool_info.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;

  if (vkCreateCommandPool(g_state.device, &pool_info, nullptr, &g_state.command_pool) != VK_SUCCESS) {
    rkg::log::error("vkCreateCommandPool failed");
    return false;
  }
  return true;
}

bool create_command_buffers() {
  g_state.command_buffers.resize(g_state.framebuffers.size());
  VkCommandBufferAllocateInfo alloc_info{};
  alloc_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
  alloc_info.commandPool = g_state.command_pool;
  alloc_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
  alloc_info.commandBufferCount = static_cast<uint32_t>(g_state.command_buffers.size());

  if (vkAllocateCommandBuffers(g_state.device, &alloc_info, g_state.command_buffers.data()) != VK_SUCCESS) {
    rkg::log::error("vkAllocateCommandBuffers failed");
    return false;
  }
  return true;
}

bool create_sync_objects() {
  VkSemaphoreCreateInfo sem_info{};
  sem_info.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;

  if (vkCreateSemaphore(g_state.device, &sem_info, nullptr, &g_state.image_available) != VK_SUCCESS) {
    rkg::log::error("vkCreateSemaphore failed");
    return false;
  }
  if (vkCreateSemaphore(g_state.device, &sem_info, nullptr, &g_state.render_finished) != VK_SUCCESS) {
    rkg::log::error("vkCreateSemaphore failed");
    return false;
  }

  VkFenceCreateInfo fence_info{};
  fence_info.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
  fence_info.flags = VK_FENCE_CREATE_SIGNALED_BIT;
  if (vkCreateFence(g_state.device, &fence_info, nullptr, &g_state.in_flight_fence) != VK_SUCCESS) {
    rkg::log::error("vkCreateFence failed");
    return false;
  }

  return true;
}

void destroy_swapchain() {
  for (auto fb : g_state.framebuffers) {
    vkDestroyFramebuffer(g_state.device, fb, nullptr);
  }
  g_state.framebuffers.clear();

  if (g_state.render_pass != VK_NULL_HANDLE) {
    vkDestroyRenderPass(g_state.device, g_state.render_pass, nullptr);
    g_state.render_pass = VK_NULL_HANDLE;
  }

  for (auto view : g_state.swapchain_image_views) {
    vkDestroyImageView(g_state.device, view, nullptr);
  }
  g_state.swapchain_image_views.clear();

  if (g_state.swapchain != VK_NULL_HANDLE) {
    vkDestroySwapchainKHR(g_state.device, g_state.swapchain, nullptr);
    g_state.swapchain = VK_NULL_HANDLE;
  }
}

void destroy_vulkan() {
  if (g_state.device != VK_NULL_HANDLE) {
    vkDeviceWaitIdle(g_state.device);
  }

  if (g_state.in_flight_fence != VK_NULL_HANDLE) {
    vkDestroyFence(g_state.device, g_state.in_flight_fence, nullptr);
  }
  if (g_state.image_available != VK_NULL_HANDLE) {
    vkDestroySemaphore(g_state.device, g_state.image_available, nullptr);
  }
  if (g_state.render_finished != VK_NULL_HANDLE) {
    vkDestroySemaphore(g_state.device, g_state.render_finished, nullptr);
  }
  if (g_state.command_pool != VK_NULL_HANDLE) {
    vkDestroyCommandPool(g_state.device, g_state.command_pool, nullptr);
  }

  destroy_swapchain();

  if (g_state.device != VK_NULL_HANDLE) {
    vkDestroyDevice(g_state.device, nullptr);
  }
  if (g_state.surface != VK_NULL_HANDLE) {
    vkDestroySurfaceKHR(g_state.instance, g_state.surface, nullptr);
  }
  if (g_state.instance != VK_NULL_HANDLE) {
    vkDestroyInstance(g_state.instance, nullptr);
  }
}

bool record_command_buffer(VkCommandBuffer cmd, uint32_t image_index) {
  VkCommandBufferBeginInfo begin_info{};
  begin_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
  if (vkBeginCommandBuffer(cmd, &begin_info) != VK_SUCCESS) {
    return false;
  }

  VkClearValue clear_color{};
  clear_color.color = {{0.02f, 0.02f, 0.05f, 1.0f}};

  VkRenderPassBeginInfo render_pass_info{};
  render_pass_info.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
  render_pass_info.renderPass = g_state.render_pass;
  render_pass_info.framebuffer = g_state.framebuffers[image_index];
  render_pass_info.renderArea.offset = {0, 0};
  render_pass_info.renderArea.extent = g_state.swapchain_extent;
  render_pass_info.clearValueCount = 1;
  render_pass_info.pClearValues = &clear_color;

  vkCmdBeginRenderPass(cmd, &render_pass_info, VK_SUBPASS_CONTENTS_INLINE);

  VkClearAttachment clear_attachment{};
  clear_attachment.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  clear_attachment.colorAttachment = 0;
  clear_attachment.clearValue.color = {{0.15f, 0.65f, 0.95f, 1.0f}};

  VkClearRect rect{};
  rect.baseArrayLayer = 0;
  rect.layerCount = 1;
  rect.rect.offset = {static_cast<int32_t>(g_state.swapchain_extent.width / 4),
                      static_cast<int32_t>(g_state.swapchain_extent.height / 4)};
  rect.rect.extent = {g_state.swapchain_extent.width / 2, g_state.swapchain_extent.height / 2};

  vkCmdClearAttachments(cmd, 1, &clear_attachment, 1, &rect);

#if RKG_ENABLE_IMGUI
  rkg::debug_ui::render(cmd);
#endif

  vkCmdEndRenderPass(cmd);

  return vkEndCommandBuffer(cmd) == VK_SUCCESS;
}

bool draw_frame() {
  vkWaitForFences(g_state.device, 1, &g_state.in_flight_fence, VK_TRUE, UINT64_MAX);
  vkResetFences(g_state.device, 1, &g_state.in_flight_fence);

  uint32_t image_index = 0;
  VkResult result = vkAcquireNextImageKHR(
      g_state.device, g_state.swapchain, UINT64_MAX, g_state.image_available, VK_NULL_HANDLE, &image_index);
  if (result != VK_SUCCESS) {
    return false;
  }

  vkResetCommandBuffer(g_state.command_buffers[image_index], 0);
  if (!record_command_buffer(g_state.command_buffers[image_index], image_index)) {
    return false;
  }

  VkSemaphore wait_semaphores[] = {g_state.image_available};
  VkPipelineStageFlags wait_stages[] = {VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT};
  VkSemaphore signal_semaphores[] = {g_state.render_finished};

  VkSubmitInfo submit_info{};
  submit_info.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
  submit_info.waitSemaphoreCount = 1;
  submit_info.pWaitSemaphores = wait_semaphores;
  submit_info.pWaitDstStageMask = wait_stages;
  submit_info.commandBufferCount = 1;
  submit_info.pCommandBuffers = &g_state.command_buffers[image_index];
  submit_info.signalSemaphoreCount = 1;
  submit_info.pSignalSemaphores = signal_semaphores;

  if (vkQueueSubmit(g_state.graphics_queue, 1, &submit_info, g_state.in_flight_fence) != VK_SUCCESS) {
    return false;
  }

  VkPresentInfoKHR present_info{};
  present_info.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
  present_info.waitSemaphoreCount = 1;
  present_info.pWaitSemaphores = signal_semaphores;
  present_info.swapchainCount = 1;
  present_info.pSwapchains = &g_state.swapchain;
  present_info.pImageIndices = &image_index;

  result = vkQueuePresentKHR(g_state.present_queue, &present_info);
  return result == VK_SUCCESS;
}

bool vulkan_init(void* host) {
  g_state.ctx = static_cast<rkg::HostContext*>(host);
  if (!g_state.ctx || !g_state.ctx->platform) {
    rkg::log::error("renderer:vulkan missing platform");
    return false;
  }

  g_state.window = static_cast<SDL_Window*>(g_state.ctx->platform->native_window());
  if (!g_state.window) {
    rkg::log::error("renderer:vulkan missing SDL window");
    return false;
  }

  if (!create_instance()) return false;
  if (!create_surface()) return false;
  if (!pick_physical_device()) return false;
  if (!find_queue_families()) return false;
  if (!create_device()) return false;
  if (!create_swapchain()) return false;
  if (!create_render_pass()) return false;
  if (!create_framebuffers()) return false;
  if (!create_command_pool()) return false;
  if (!create_command_buffers()) return false;
  if (!create_sync_objects()) return false;

  rkg::VulkanHooks hooks;
  hooks.instance = g_state.instance;
  hooks.physical_device = g_state.physical_device;
  hooks.device = g_state.device;
  hooks.queue = g_state.graphics_queue;
  hooks.render_pass = g_state.render_pass;
  hooks.window = g_state.window;
  hooks.queue_family = g_state.graphics_queue_family;
  hooks.image_count = static_cast<uint32_t>(g_state.swapchain_images.size());
  rkg::register_vulkan_hooks(&hooks);

  rkg::log::info("renderer:vulkan init");
  return true;
}

void vulkan_shutdown() {
  rkg::register_vulkan_hooks(nullptr);
  destroy_vulkan();
  rkg::log::info("renderer:vulkan shutdown");
}

void vulkan_update(float) {
  if (!draw_frame()) {
    rkg::log::warn("renderer:vulkan draw_frame failed");
  }
}

rkg::RkgPluginApi g_api{
    rkg::kRkgPluginApiVersion,
    "renderer_vulkan",
    rkg::PluginType::Renderer,
    static_cast<uint32_t>(rkg::RendererCaps::DrawPrimitive) |
        static_cast<uint32_t>(rkg::RendererCaps::DebugUiSupported),
    &vulkan_init,
    &vulkan_shutdown,
    &vulkan_update,
    nullptr};
} // namespace

extern "C" rkg::RkgPluginApi* rkg_plugin_get_api_renderer_vulkan(uint32_t host_api_version) {
  if (host_api_version != rkg::kRkgPluginApiVersion) {
    return nullptr;
  }
  return &g_api;
}

#if defined(RKG_BUILD_DYNAMIC_PLUGIN)
extern "C" rkg::RkgPluginApi* rkg_plugin_get_api(uint32_t host_api_version) {
  return rkg_plugin_get_api_renderer_vulkan(host_api_version);
}
#endif
