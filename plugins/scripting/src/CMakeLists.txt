rkg_add_library(rkg_script
  script_runtime.cpp
)

target_include_directories(rkg_script
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/../include
)

target_link_libraries(rkg_script PUBLIC rkg_core)
