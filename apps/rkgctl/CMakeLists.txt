add_library(rkgctl_lib STATIC
  src/main.cpp
)

target_compile_definitions(rkgctl_lib PRIVATE RKGCTL_LIB=1)

target_include_directories(rkgctl_lib
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(rkgctl_lib
  PUBLIC
    rkg_core
    rkg_plugins_data
    rkg_platform_watch
)

if(RKG_ENABLE_OPENAI)
  target_link_libraries(rkgctl_lib PUBLIC CURL::libcurl)
endif()

if(RKG_ENABLE_DATA_JSON)
  target_link_libraries(rkgctl_lib PUBLIC nlohmann_json::nlohmann_json)
else()
  message(FATAL_ERROR "rkgctl requires JSON support (RKG_ENABLE_DATA_JSON=ON)")
endif()

if(RKG_ENABLE_DATA_YAML)
  target_link_libraries(rkgctl_lib PUBLIC yaml-cpp)
endif()

rkg_add_executable(rkgctl
  src/main.cpp
)

target_include_directories(rkgctl
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(rkgctl
  PUBLIC
    rkgctl_lib
)
