cmake_minimum_required(VERSION 3.20)

project(rkg VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(helpers)

# Build options
if(WIN32)
  set(RKG_ENABLE_D3D12_DEFAULT ON)
else()
  set(RKG_ENABLE_D3D12_DEFAULT OFF)
endif()

if(UNIX AND NOT APPLE)
  set(RKG_ENABLE_VULKAN_DEFAULT ON)
else()
  set(RKG_ENABLE_VULKAN_DEFAULT OFF)
endif()

option(RKG_ENABLE_VULKAN "Enable Vulkan renderer plugin" ${RKG_ENABLE_VULKAN_DEFAULT})
option(RKG_ENABLE_D3D12 "Enable D3D12 renderer plugin" ${RKG_ENABLE_D3D12_DEFAULT})
option(RKG_ENABLE_SCRIPT_LUA "Enable Lua scripting plugin" ON)
option(RKG_ENABLE_SCRIPT_JS "Enable JS scripting plugin" ON)
option(RKG_ENABLE_SCRIPT_PYTHON "Enable Python scripting plugin" ON)
option(RKG_ENABLE_DATA_SQLITE "Enable SQLite data plugin" ON)
option(RKG_ENABLE_DATA_YAML "Enable YAML data plugin" ON)
option(RKG_ENABLE_DATA_JSON "Enable JSON data plugin" ON)
option(RKG_ENABLE_AI "Enable AI orchestration plugin" ON)
option(RKG_ENABLE_CLOUD "Enable Cloud plugin" OFF)
option(RKG_ENABLE_IMGUI "Enable ImGui debug UI plugin" ON)
option(RKG_ENABLE_OPENAI "Enable OpenAI provider (rkgctl)" ON)
option(RKG_ENABLE_PHYSICS_BASIC "Enable basic physics plugin" ON)
option(RKG_BUILD_TESTS "Build tests" ON)

message(STATUS "RKG_ENABLE_VULKAN=${RKG_ENABLE_VULKAN}")
message(STATUS "RKG_ENABLE_D3D12=${RKG_ENABLE_D3D12}")

if(RKG_ENABLE_OPENAI)
  find_package(CURL QUIET)
  if(NOT CURL_FOUND)
    message(WARNING "libcurl not found; disabling OpenAI provider.")
    set(RKG_ENABLE_OPENAI OFF)
  endif()
endif()

add_compile_definitions(
  RKG_ENABLE_VULKAN=$<BOOL:${RKG_ENABLE_VULKAN}>
  RKG_ENABLE_D3D12=$<BOOL:${RKG_ENABLE_D3D12}>
  RKG_ENABLE_SCRIPT_LUA=$<BOOL:${RKG_ENABLE_SCRIPT_LUA}>
  RKG_ENABLE_SCRIPT_JS=$<BOOL:${RKG_ENABLE_SCRIPT_JS}>
  RKG_ENABLE_SCRIPT_PYTHON=$<BOOL:${RKG_ENABLE_SCRIPT_PYTHON}>
  RKG_ENABLE_DATA_SQLITE=$<BOOL:${RKG_ENABLE_DATA_SQLITE}>
  RKG_ENABLE_DATA_YAML=$<BOOL:${RKG_ENABLE_DATA_YAML}>
  RKG_ENABLE_DATA_JSON=$<BOOL:${RKG_ENABLE_DATA_JSON}>
  RKG_ENABLE_AI=$<BOOL:${RKG_ENABLE_AI}>
  RKG_ENABLE_CLOUD=$<BOOL:${RKG_ENABLE_CLOUD}>
  RKG_ENABLE_IMGUI=$<BOOL:${RKG_ENABLE_IMGUI}>
  RKG_ENABLE_OPENAI=$<BOOL:${RKG_ENABLE_OPENAI}>
  RKG_ENABLE_PHYSICS_BASIC=$<BOOL:${RKG_ENABLE_PHYSICS_BASIC}>
)

# Dependencies (prefer system, fallback to FetchContent)
include(FetchContent)

if(RKG_ENABLE_DATA_JSON)
  find_package(nlohmann_json QUIET)
  if(NOT nlohmann_json_FOUND)
    message(STATUS "nlohmann_json not found; fetching")
    FetchContent_Declare(
      nlohmann_json
      GIT_REPOSITORY https://github.com/nlohmann/json.git
      GIT_TAG v3.11.3
    )
    FetchContent_MakeAvailable(nlohmann_json)
  endif()
endif()

if(RKG_ENABLE_DATA_YAML)
  find_package(yaml-cpp QUIET)
  if(NOT yaml-cpp_FOUND)
    message(STATUS "yaml-cpp not found; fetching")
    FetchContent_Declare(
      yaml-cpp
      GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
      GIT_TAG yaml-cpp-0.8.0
    )
    FetchContent_MakeAvailable(yaml-cpp)
  endif()
endif()

if(RKG_ENABLE_DATA_SQLITE)
  find_package(SQLite3 QUIET)
  if(SQLite3_FOUND)
    add_library(rkg_sqlite3 INTERFACE)
    target_include_directories(rkg_sqlite3 INTERFACE ${SQLite3_INCLUDE_DIRS})
    target_link_libraries(rkg_sqlite3 INTERFACE ${SQLite3_LIBRARIES})
    target_compile_definitions(rkg_sqlite3 INTERFACE RKG_HAVE_SQLITE3=1)
  else()
    message(WARNING "SQLite3 not found; SQLite features will be disabled.")
    add_library(rkg_sqlite3 INTERFACE)
    target_compile_definitions(rkg_sqlite3 INTERFACE RKG_HAVE_SQLITE3=0)
  endif()
else()
  add_library(rkg_sqlite3 INTERFACE)
  target_compile_definitions(rkg_sqlite3 INTERFACE RKG_HAVE_SQLITE3=0)
endif()

if(NOT APPLE)
  find_package(SDL3 REQUIRED)
endif()

if(RKG_ENABLE_VULKAN)
  find_package(Vulkan QUIET)
  if(NOT Vulkan_FOUND)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
      pkg_check_modules(VULKAN QUIET vulkan)
    endif()
    if(VULKAN_FOUND)
      add_library(Vulkan::Vulkan INTERFACE IMPORTED)
      target_include_directories(Vulkan::Vulkan INTERFACE ${VULKAN_INCLUDE_DIRS})
      target_link_libraries(Vulkan::Vulkan INTERFACE ${VULKAN_LIBRARIES})
      message(STATUS "Vulkan found via pkg-config")
      set(Vulkan_FOUND TRUE)
    else()
      message(WARNING "Vulkan not found; disabling Vulkan renderer.")
      set(RKG_ENABLE_VULKAN OFF)
    endif()
  endif()
endif()

add_subdirectory(core)
add_subdirectory(platform)
add_subdirectory(runtime)
add_subdirectory(plugins)
add_subdirectory(apps)
add_subdirectory(examples)
add_subdirectory(projects)

if(RKG_BUILD_TESTS)
  add_subdirectory(tests)
endif()
