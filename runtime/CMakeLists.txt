rkg_add_library(rkg_runtime
  src/asset_cache.cpp
  src/runtime_host.cpp
)

target_include_directories(rkg_runtime
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

if(RKG_ENABLE_IMGUI)
  target_include_directories(rkg_runtime
    PUBLIC
      ${CMAKE_CURRENT_LIST_DIR}/../plugins/debug_ui/imgui/include
  )
endif()

target_link_libraries(rkg_runtime
  PUBLIC
    rkg_core
    rkg_platform
)
