find_package(Lua QUIET)
include(CheckSymbolExists)

rkg_add_library(rkg_plugins_script_lua
  src/lua_runtime.cpp
)

target_include_directories(rkg_plugins_script_lua
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(rkg_plugins_script_lua
  PUBLIC
    rkg_core
    rkg_script
)

set(RKG_LUA_AVAILABLE OFF)
set(RKG_LUA_LINK_LIBS "")
set(RKG_LUA_INCLUDE_DIRS "")

macro(_rkg_append_unique list_name value)
  if("${value}" STREQUAL "")
    return()
  endif()
  list(APPEND ${list_name} ${value})
  list(REMOVE_DUPLICATES ${list_name})
endmacro()

if(Lua_FOUND OR LUA_FOUND)
  message(STATUS "Lua detection: CMake package found (FindLua).")
  if(Lua_VERSION)
    message(STATUS "Lua detection: package version ${Lua_VERSION}")
  endif()
  if(LUA_INCLUDE_DIRS)
    _rkg_append_unique(RKG_LUA_INCLUDE_DIRS "${LUA_INCLUDE_DIRS}")
  elseif(LUA_INCLUDE_DIR)
    _rkg_append_unique(RKG_LUA_INCLUDE_DIRS "${LUA_INCLUDE_DIR}")
  endif()
  if(TARGET Lua::Lua)
    _rkg_append_unique(RKG_LUA_LINK_LIBS "Lua::Lua")
  elseif(LUA_LIBRARIES)
    _rkg_append_unique(RKG_LUA_LINK_LIBS "${LUA_LIBRARIES}")
  elseif(LUA_LIBRARY)
    _rkg_append_unique(RKG_LUA_LINK_LIBS "${LUA_LIBRARY}")
  endif()
else()
  message(STATUS "Lua detection: CMake package not found, using manual fallback.")
endif()

if(LUA_AUXLIB)
  if(EXISTS "${LUA_AUXLIB}")
    _rkg_append_unique(RKG_LUA_LINK_LIBS "${LUA_AUXLIB}")
  else()
    message(WARNING "Lua detection: LUA_AUXLIB is set but does not exist -> ${LUA_AUXLIB}")
  endif()
endif()
if(LUA_LUALIB)
  if(EXISTS "${LUA_LUALIB}")
    _rkg_append_unique(RKG_LUA_LINK_LIBS "${LUA_LUALIB}")
  else()
    message(WARNING "Lua detection: LUA_LUALIB is set but does not exist -> ${LUA_LUALIB}")
  endif()
endif()

if(NOT RKG_LUA_LINK_LIBS)
  if(WIN32)
    find_library(RKG_LUA_AUX_LIB
      NAMES lauxlib54 lauxlib5.4 lauxlib53 lauxlib5.3 lauxlib52 lauxlib5.2 lauxlib51 lauxlib5.1 lauxlib
      HINTS ENV LUA_DIR
      PATH_SUFFIXES lib
    )
    find_library(RKG_LUA_LU_LIB
      NAMES lualib54 lualib5.4 lualib53 lualib5.3 lualib52 lualib5.2 lualib51 lualib5.1 lualib
      HINTS ENV LUA_DIR
      PATH_SUFFIXES lib
    )
    find_library(RKG_LUA_CORE_LIB
      NAMES lua54 lua53 lua52 lua51 lua
      HINTS ENV LUA_DIR
      PATH_SUFFIXES lib
    )
  else()
    find_library(RKG_LUA_AUX_LIB
      NAMES lauxlib5.4 lauxlib54 lauxlib5.3 lauxlib53 lauxlib5.2 lauxlib52 lauxlib5.1 lauxlib51 lauxlib
      HINTS ENV LUA_DIR
      PATH_SUFFIXES lib
    )
    find_library(RKG_LUA_LU_LIB
      NAMES lualib5.4 lualib54 lualib5.3 lualib53 lualib5.2 lualib52 lualib5.1 lualib51 lualib
      HINTS ENV LUA_DIR
      PATH_SUFFIXES lib
    )
    find_library(RKG_LUA_CORE_LIB
      NAMES lua54 lua5.4 lua-5.4 lua.5.4 lua53 lua5.3 lua-5.3 lua.5.3 lua52 lua5.2 lua-5.2 lua.5.2 lua51 lua5.1 lua-5.1 lua.5.1 lua
      HINTS ENV LUA_DIR
      PATH_SUFFIXES lib
    )
  endif()
  if(RKG_LUA_CORE_LIB)
    _rkg_append_unique(RKG_LUA_LINK_LIBS "${RKG_LUA_CORE_LIB}")
  endif()
  if(RKG_LUA_AUX_LIB)
    _rkg_append_unique(RKG_LUA_LINK_LIBS "${RKG_LUA_AUX_LIB}")
  endif()
  if(RKG_LUA_LU_LIB)
    _rkg_append_unique(RKG_LUA_LINK_LIBS "${RKG_LUA_LU_LIB}")
  endif()
endif()

if(NOT RKG_LUA_INCLUDE_DIRS)
  find_path(RKG_LUA_INCLUDE_HINT
    NAMES lua.h
    HINTS ENV LUA_DIR
    PATH_SUFFIXES
      include
      include/lua
      include/lua5.4
      include/lua5.3
      include/lua5.2
      include/lua5.1
      include/lua-5.4
      include/lua-5.3
      include/lua-5.2
      include/lua-5.1
  )
  if(RKG_LUA_INCLUDE_HINT)
    _rkg_append_unique(RKG_LUA_INCLUDE_DIRS "${RKG_LUA_INCLUDE_HINT}")
  endif()
endif()

if(RKG_LUA_INCLUDE_DIRS)
  target_include_directories(rkg_plugins_script_lua PUBLIC ${RKG_LUA_INCLUDE_DIRS})
endif()

set(RKG_LUA_VERSION "unknown")
foreach(_rkg_dir IN LISTS RKG_LUA_INCLUDE_DIRS)
  if(EXISTS "${_rkg_dir}/lua.h")
    file(STRINGS "${_rkg_dir}/lua.h" _rkg_lua_version_lines
      REGEX "^[ \t]*#define[ \t]+LUA_VERSION_NUM[ \t]+[0-9]+"
    )
    if(_rkg_lua_version_lines)
      list(GET _rkg_lua_version_lines 0 _rkg_version_line)
      string(REGEX REPLACE ".*#define[ \t]+LUA_VERSION_NUM[ \t]+([0-9]+).*" "\\1" RKG_LUA_VERSION "${_rkg_version_line}")
      break()
    endif()
  endif()
endforeach()

message(STATUS "Lua detection summary:")
message(STATUS "  includes: ${RKG_LUA_INCLUDE_DIRS}")
message(STATUS "  libraries: ${RKG_LUA_LINK_LIBS}")
message(STATUS "  header version: ${RKG_LUA_VERSION}")
message(STATUS "  Lua_FOUND: ${Lua_FOUND}; LUA_FOUND: ${LUA_FOUND}")

set(CMAKE_REQUIRED_INCLUDES ${RKG_LUA_INCLUDE_DIRS})
set(CMAKE_REQUIRED_LIBRARIES ${RKG_LUA_LINK_LIBS})
check_symbol_exists(luaL_newstate "lua.h;lauxlib.h;lualib.h" RKG_LUA_HAS_NEWSTATE)
check_symbol_exists(luaL_openlibs "lua.h;lauxlib.h;lualib.h" RKG_LUA_HAS_OPENLIBS)
check_symbol_exists(luaL_loadfilex "lua.h;lauxlib.h;lualib.h" RKG_LUA_HAS_LOADFILEX)
check_symbol_exists(luaL_dostring "lua.h;lauxlib.h;lualib.h" RKG_LUA_HAS_DOSTRING)
check_symbol_exists(lua_pcallk "lua.h;lauxlib.h;lualib.h" RKG_LUA_HAS_PCALLK)
check_symbol_exists(luaL_checkinteger "lua.h;lauxlib.h;lualib.h" RKG_LUA_HAS_CHECKINTEGER)
set(CMAKE_REQUIRED_INCLUDES "")
set(CMAKE_REQUIRED_LIBRARIES "")

if(
  RKG_LUA_HAS_NEWSTATE
  AND RKG_LUA_HAS_OPENLIBS
  AND RKG_LUA_HAS_LOADFILEX
  AND RKG_LUA_HAS_DOSTRING
  AND RKG_LUA_HAS_PCALLK
  AND RKG_LUA_HAS_CHECKINTEGER
  AND NOT RKG_LUA_VERSION STREQUAL "unknown"
  AND RKG_LUA_VERSION GREATER_EQUAL 502
  AND RKG_LUA_LINK_LIBS
  AND RKG_LUA_INCLUDE_DIRS
)
  set(RKG_LUA_AVAILABLE ON)
endif()

if(RKG_LUA_AVAILABLE)
  message(STATUS "Lua runtime plugin enabled with valid symbols and matching headers/libs.")
  target_link_libraries(rkg_plugins_script_lua PUBLIC ${RKG_LUA_LINK_LIBS})
  target_compile_definitions(rkg_plugins_script_lua PRIVATE RKG_HAVE_LUA=1)
else()
  message(WARNING "Lua detection failed: plugin disabled to avoid unresolved symbols.")
  message(WARNING "  symbol checks: luaL_newstate=${RKG_LUA_HAS_NEWSTATE}, luaL_openlibs=${RKG_LUA_HAS_OPENLIBS}, luaL_loadfilex=${RKG_LUA_HAS_LOADFILEX}, luaL_dostring=${RKG_LUA_HAS_DOSTRING}, lua_pcallk=${RKG_LUA_HAS_PCALLK}, luaL_checkinteger=${RKG_LUA_HAS_CHECKINTEGER}")
  target_compile_definitions(rkg_plugins_script_lua PRIVATE RKG_HAVE_LUA=0)
endif()
