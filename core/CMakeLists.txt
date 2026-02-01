rkg_add_library(rkg_core
  src/log.cpp
  src/movement_log.cpp
  src/ecs.cpp
  src/plugin_host.cpp
  src/config.cpp
  src/instructions.cpp
  src/project.cpp
  src/paths.cpp
  src/input.cpp
  src/math.cpp
  src/locomotion.cpp
  src/renderer_hooks.cpp
  src/renderer_select.cpp
  src/renderer_util.cpp
  src/replay.cpp
  src/content_pack.cpp
  src/ai_results.cpp
  src/asset_import.cpp
  src/json_write.cpp
  src/stb_image_impl.cpp
  src/staged_runs.cpp
  src/run_cleanup.cpp
  src/snapshot_restore.cpp
)

target_include_directories(rkg_core
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}/../platform/include
)

target_link_libraries(rkg_core
  PUBLIC
    rkg_thirdparty_cgltf
    rkg_thirdparty_stb
)

if(RKG_ENABLE_DATA_JSON)
  target_link_libraries(rkg_core PUBLIC nlohmann_json::nlohmann_json)
  target_compile_definitions(rkg_core PUBLIC RKG_ENABLE_DATA_JSON=1)
else()
  target_compile_definitions(rkg_core PUBLIC RKG_ENABLE_DATA_JSON=0)
endif()

if(RKG_ENABLE_DATA_YAML)
  target_link_libraries(rkg_core PUBLIC yaml-cpp)
  target_compile_definitions(rkg_core PUBLIC RKG_ENABLE_DATA_YAML=1)
else()
  target_compile_definitions(rkg_core PUBLIC RKG_ENABLE_DATA_YAML=0)
endif()
