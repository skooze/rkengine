rkg_add_library(rkg_core
  src/log.cpp
  src/ecs.cpp
  src/plugin_host.cpp
  src/config.cpp
  src/instructions.cpp
  src/project.cpp
  src/paths.cpp
  src/input.cpp
  src/renderer_hooks.cpp
  src/renderer_select.cpp
  src/renderer_util.cpp
  src/replay.cpp
  src/content_pack.cpp
  src/ai_results.cpp
)

target_include_directories(rkg_core
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
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
