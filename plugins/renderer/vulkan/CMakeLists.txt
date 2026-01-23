rkg_add_library(rkg_plugins_renderer_vulkan
  src/vulkan_renderer.cpp
)

target_include_directories(rkg_plugins_renderer_vulkan
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

if(RKG_ENABLE_IMGUI)
  target_include_directories(rkg_plugins_renderer_vulkan
    PUBLIC
      ${CMAKE_CURRENT_LIST_DIR}/../../debug_ui/imgui/include
  )
endif()

target_link_libraries(rkg_plugins_renderer_vulkan
  PUBLIC
    rkg_core
    rkg_platform
    rkg_renderer_rhi
    Vulkan::Vulkan
    SDL3::SDL3
)

if(RKG_ENABLE_IMGUI)
  target_link_libraries(rkg_plugins_renderer_vulkan PUBLIC rkg_plugins_debug_ui_imgui)
endif()

target_compile_definitions(rkg_plugins_renderer_vulkan PUBLIC RKG_ENABLE_VULKAN=1)
