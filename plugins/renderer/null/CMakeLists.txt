rkg_add_library(rkg_plugins_renderer_null
  src/null_renderer.cpp
)

target_include_directories(rkg_plugins_renderer_null
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(rkg_plugins_renderer_null
  PUBLIC
    rkg_core
    rkg_platform
    rkg_renderer_rhi
)
