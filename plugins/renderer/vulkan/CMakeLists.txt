rkg_add_library(rkg_plugins_renderer_vulkan
  src/vulkan_renderer.cpp
)

target_include_directories(rkg_plugins_renderer_vulkan
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(rkg_plugins_renderer_vulkan
  PUBLIC
    rkg_core
    rkg_platform
    rkg_renderer_rhi
    Vulkan::Vulkan
    SDL3::SDL3
)

target_compile_definitions(rkg_plugins_renderer_vulkan PUBLIC RKG_ENABLE_VULKAN=1)
