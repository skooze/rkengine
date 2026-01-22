rkg_add_library(rkg_plugins_ai
  src/ai_stub.cpp
)

target_include_directories(rkg_plugins_ai
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(rkg_plugins_ai PUBLIC rkg_core)
