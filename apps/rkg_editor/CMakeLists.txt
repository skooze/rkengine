rkg_add_executable(rkg_editor
  src/main.cpp
  src/subprocess_runner.cpp
)

target_link_libraries(rkg_editor
  PUBLIC
    rkg_runtime
    rkg_core
    rkg_platform
    rkg_renderer_rhi
    rkg_plugins_renderer_null
    rkg_plugins_data
)

if(RKG_ENABLE_D3D12)
  target_link_libraries(rkg_editor PUBLIC rkg_plugins_renderer_d3d12)
endif()

if(RKG_ENABLE_VULKAN)
  target_link_libraries(rkg_editor PUBLIC rkg_plugins_renderer_vulkan)
endif()

if(RKG_ENABLE_SCRIPT_LUA)
  target_link_libraries(rkg_editor PUBLIC rkg_plugins_script_lua)
endif()
if(RKG_ENABLE_SCRIPT_JS)
  target_link_libraries(rkg_editor PUBLIC rkg_plugins_script_js)
endif()
if(RKG_ENABLE_SCRIPT_PYTHON)
  target_link_libraries(rkg_editor PUBLIC rkg_plugins_script_python)
endif()

if(TARGET rkg_plugins_debug_ui_imgui)
  target_link_libraries(rkg_editor PUBLIC rkg_plugins_debug_ui_imgui)
endif()
