rkg_add_library(rkg_plugins_renderer_d3d12
  src/d3d12_renderer.cpp
)

target_include_directories(rkg_plugins_renderer_d3d12
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(rkg_plugins_renderer_d3d12
  PUBLIC
    rkg_core
    rkg_platform
    rkg_renderer_rhi
)

if(WIN32)
  target_compile_definitions(rkg_plugins_renderer_d3d12 PUBLIC RKG_D3D12_SUPPORTED=1)
  target_link_libraries(rkg_plugins_renderer_d3d12 PUBLIC d3d12 dxgi d3dcompiler)
else()
  target_compile_definitions(rkg_plugins_renderer_d3d12 PUBLIC RKG_D3D12_SUPPORTED=0)
endif()
