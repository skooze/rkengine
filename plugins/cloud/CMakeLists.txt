rkg_add_library(rkg_plugins_cloud
  src/cloud_stub.cpp
)

target_include_directories(rkg_plugins_cloud
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(rkg_plugins_cloud PUBLIC rkg_core)

target_compile_definitions(rkg_plugins_cloud PUBLIC RKG_ENABLE_CLOUD=0)
