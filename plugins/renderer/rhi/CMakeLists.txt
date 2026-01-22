rkg_add_library(rkg_renderer_rhi
  src/rhi.cpp
)

target_include_directories(rkg_renderer_rhi
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(rkg_renderer_rhi PUBLIC rkg_core)
