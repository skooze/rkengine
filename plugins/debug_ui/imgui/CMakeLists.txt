include(FetchContent)

FetchContent_Declare(
  imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG v1.90.4
)
FetchContent_MakeAvailable(imgui)

set(IMGUI_SOURCES
  ${imgui_SOURCE_DIR}/imgui.cpp
  ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp
  ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_sdl3.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_vulkan.cpp
)

rkg_add_library(rkg_plugins_debug_ui_imgui
  src/imgui_plugin.cpp
  src/imgui_runtime.cpp
  ${IMGUI_SOURCES}
)

target_include_directories(rkg_plugins_debug_ui_imgui
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${imgui_SOURCE_DIR}
  PRIVATE
    ${imgui_SOURCE_DIR}/backends
)

target_link_libraries(rkg_plugins_debug_ui_imgui
  PUBLIC
    rkg_core
    rkg_platform
    Vulkan::Vulkan
    SDL3::SDL3
)

target_compile_definitions(rkg_plugins_debug_ui_imgui PRIVATE IMGUI_DISABLE_OBSOLETE_FUNCTIONS=1)
