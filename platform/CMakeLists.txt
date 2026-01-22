rkg_add_library(rkg_platform_watch
  src/file_watcher.cpp
)

target_include_directories(rkg_platform_watch
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(rkg_platform_watch PUBLIC rkg_core)

rkg_add_library(rkg_platform
  src/platform.cpp
)

target_include_directories(rkg_platform
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

if(NOT APPLE)
  target_sources(rkg_platform PRIVATE linux/sdl_platform.cpp)
  target_link_libraries(rkg_platform PRIVATE SDL3::SDL3)
  target_compile_definitions(rkg_platform PRIVATE RKG_PLATFORM_SDL=1)
else()
  target_sources(rkg_platform PRIVATE windows/platform_stub.cpp)
  target_compile_definitions(rkg_platform PRIVATE RKG_PLATFORM_SDL=0)
endif()

target_link_libraries(rkg_platform PUBLIC rkg_core rkg_platform_watch)
