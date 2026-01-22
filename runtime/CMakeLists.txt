rkg_add_library(rkg_runtime
  src/runtime_host.cpp
)

target_include_directories(rkg_runtime
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(rkg_runtime
  PUBLIC
    rkg_core
    rkg_platform
)
