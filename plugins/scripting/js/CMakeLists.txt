rkg_add_library(rkg_plugins_script_js
  src/js_runtime.cpp
)

target_include_directories(rkg_plugins_script_js
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(rkg_plugins_script_js
  PUBLIC
    rkg_core
    rkg_script
)

target_compile_definitions(rkg_plugins_script_js PRIVATE RKG_HAVE_JS=0)
