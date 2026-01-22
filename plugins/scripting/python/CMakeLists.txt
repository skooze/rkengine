rkg_add_library(rkg_plugins_script_python
  src/python_runtime.cpp
)

target_include_directories(rkg_plugins_script_python
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(rkg_plugins_script_python
  PUBLIC
    rkg_core
    rkg_script
)

target_compile_definitions(rkg_plugins_script_python PRIVATE RKG_HAVE_PYTHON=0)
