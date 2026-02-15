find_package(Lua QUIET)
include(CheckSymbolExists)
include(CheckCSourceRuns)

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
set(RKG_LUA_VERSION "unknown")

macro(_rkg_append_unique list_name value)
  if("${value}" STREQUAL "")
    return()
  endif()
  list(APPEND ${list_name} ${value})
  list(REMOVE_DUPLICATES ${list_name})
endmacro()

macro(_rkg_append_lua_include_dir list_name root_dir)
  set(_rkg_include_dir "")
  set(_rkg_found_headers FALSE)
  foreach(_rkg_suffix
    ""
    lua
    lua5.4
    lua-5.4
    lua.5.4
    lua5.3
    lua-5.3
    lua.5.3
    lua5.2
    lua-5.2
    lua.5.2
    lua5.1
    lua-5.1
    lua.5.1
    luajit-2.1
    luajit-2.0
  )
    set(_rkg_candidate "${root_dir}")
    if(NOT "${_rkg_suffix}" STREQUAL "")
      set(_rkg_candidate "${_rkg_candidate}/${_rkg_suffix}")
    endif()
    if(
      EXISTS "${_rkg_candidate}/lua.h"
      AND EXISTS "${_rkg_candidate}/lauxlib.h"
      AND EXISTS "${_rkg_candidate}/lualib.h"
    )
      set(_rkg_include_dir "${_rkg_candidate}")
      set(_rkg_found_headers TRUE)
      break()
    elseif(
      EXISTS "${_rkg_candidate}/lua/lua.h"
      AND EXISTS "${_rkg_candidate}/lua/lauxlib.h"
      AND EXISTS "${_rkg_candidate}/lua/lualib.h"
    )
      set(_rkg_include_dir "${_rkg_candidate}/lua")
      set(_rkg_found_headers TRUE)
      break()
    endif()
  endforeach()
  if(_rkg_include_dir)
    _rkg_append_unique(${list_name} "${_rkg_include_dir}")
  elseif(_rkg_found_headers STREQUAL "FALSE")
    message(STATUS "Lua detection: include dir candidate '${root_dir}' rejected (missing lua.h/lauxlib.h/lualib.h)")
  else()
    message(STATUS "Lua detection: dropping include dir without header: ${root_dir}")
  endif()
endmacro()

if(Lua_FOUND OR LUA_FOUND)
  message(STATUS "Lua detection: CMake package found (FindLua).")
  if(Lua_VERSION)
    message(STATUS "Lua detection: package version ${Lua_VERSION}")
  elseif(LUA_VERSION)
    message(STATUS "Lua detection: package version ${LUA_VERSION}")
  endif()
  if(LUA_INCLUDE_DIRS)
    _rkg_append_unique(RKG_LUA_INCLUDE_DIRS "${LUA_INCLUDE_DIRS}")
  elseif(LUA_INCLUDE_DIR)
    _rkg_append_unique(RKG_LUA_INCLUDE_DIRS "${LUA_INCLUDE_DIR}")
  endif()
  if(TARGET Lua::Lua)
    _rkg_append_unique(RKG_LUA_LINK_LIBS "Lua::Lua")
    get_target_property(RKG_LUA_LUA_TARGET_LINK_LIBS Lua::Lua INTERFACE_LINK_LIBRARIES)
    if(RKG_LUA_LUA_TARGET_LINK_LIBS AND NOT RKG_LUA_LUA_TARGET_LINK_LIBS STREQUAL "RKG_LUA_LUA_TARGET_LINK_LIBS-NOTFOUND")
      message(STATUS "Lua detection: Lua::Lua interface libraries: ${RKG_LUA_LUA_TARGET_LINK_LIBS}")
      _rkg_append_unique(RKG_LUA_LINK_LIBS "${RKG_LUA_LUA_TARGET_LINK_LIBS}")
    endif()
  elseif(LUA_LIBRARIES)
    _rkg_append_unique(RKG_LUA_LINK_LIBS "${LUA_LIBRARIES}")
  elseif(LUA_LIBRARY)
    _rkg_append_unique(RKG_LUA_LINK_LIBS "${LUA_LIBRARY}")
  endif()
else()
  message(STATUS "Lua detection: CMake package not found, using fallback search.")
endif()

if(LUA_AUXLIB)
  if(EXISTS "${LUA_AUXLIB}")
    message(STATUS "Lua detection: using Lua auxlib override: ${LUA_AUXLIB}")
    _rkg_append_unique(RKG_LUA_LINK_LIBS "${LUA_AUXLIB}")
  else()
    message(WARNING "Lua detection: LUA_AUXLIB is set but does not exist -> ${LUA_AUXLIB}")
  endif()
endif()
if(LUA_LUALIB)
  if(EXISTS "${LUA_LUALIB}")
    message(STATUS "Lua detection: using Lua lualib override: ${LUA_LUALIB}")
    _rkg_append_unique(RKG_LUA_LINK_LIBS "${LUA_LUALIB}")
  else()
    message(WARNING "Lua detection: LUA_LUALIB is set but does not exist -> ${LUA_LUALIB}")
  endif()
endif()

if(NOT RKG_LUA_LINK_LIBS)
  message(STATUS "Lua detection: searching for library names (linux/windows fallback).")
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
      NAMES lua54 lua53 lua52 lua51 lua luajit-5.1 luajit
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
      NAMES lua54 lua5.4 lua-5.4 lua.5.4 lua53 lua5.3 lua-5.3 lua.5.3 lua52 lua5.2 lua-5.2 lua.5.2 lua51 lua5.1 lua-5.1 lua.5.1 lua luajit-5.1 luajit
      HINTS ENV LUA_DIR
      PATH_SUFFIXES lib
    )
  endif()
  if(RKG_LUA_CORE_LIB)
    _rkg_append_unique(RKG_LUA_LINK_LIBS "${RKG_LUA_CORE_LIB}")
    message(STATUS "Lua detection: found core library: ${RKG_LUA_CORE_LIB}")
  endif()
  if(RKG_LUA_AUX_LIB)
    _rkg_append_unique(RKG_LUA_LINK_LIBS "${RKG_LUA_AUX_LIB}")
    message(STATUS "Lua detection: found aux library: ${RKG_LUA_AUX_LIB}")
  endif()
  if(RKG_LUA_LU_LIB)
    _rkg_append_unique(RKG_LUA_LINK_LIBS "${RKG_LUA_LU_LIB}")
    message(STATUS "Lua detection: found lib library: ${RKG_LUA_LU_LIB}")
  endif()
  if(NOT RKG_LUA_LINK_LIBS)
    find_library(RKG_LUA_SYSTEM_LIB
      NAMES lua5.4 lua54 lua5.3 lua53 lua5.2 lua52 lua5.1 lua51 lua luajit-5.1 luajit
      HINTS ENV LUA_DIR /usr/local /usr /opt/homebrew
      PATH_SUFFIXES lib
    )
    _rkg_append_unique(RKG_LUA_LINK_LIBS "${RKG_LUA_SYSTEM_LIB}")
    if(RKG_LUA_SYSTEM_LIB)
      message(STATUS "Lua detection: found fallback system library: ${RKG_LUA_SYSTEM_LIB}")
    else()
      message(STATUS "Lua detection: fallback system library not found.")
    endif()
  endif()
endif()

set(_RKG_LUA_INCLUDE_RAW "${RKG_LUA_INCLUDE_DIRS}")
set(_RKG_LUA_INCLUDE_INPUT "${RKG_LUA_INCLUDE_DIRS}")
set(_RKG_LUA_SEARCH_HINTS
  $ENV{LUA_DIR}
  $ENV{VCPKG_ROOT}
  /opt/homebrew
  /usr/local
  /usr
)
set(_RKG_LUA_SEARCH_HINTS_INCLUDE)
foreach(_rkg_hint IN LISTS _RKG_LUA_SEARCH_HINTS)
  list(APPEND _RKG_LUA_SEARCH_HINTS_INCLUDE "${_rkg_hint}/include")
  list(APPEND _RKG_LUA_SEARCH_HINTS_INCLUDE "${_rkg_hint}")
endforeach()
list(APPEND _RKG_LUA_SEARCH_HINTS_INCLUDE
  /opt/homebrew/include
  /usr/local/include
  /usr/include
  /usr/include/${CMAKE_LIBRARY_ARCHITECTURE}
)
list(REMOVE_DUPLICATES _RKG_LUA_SEARCH_HINTS_INCLUDE)

message(STATUS "Lua detection: searching for headers (package + fallback scan).")
find_path(RKG_LUA_INCLUDE_HINT
  NAMES lua.h
  PATHS ${_RKG_LUA_SEARCH_HINTS_INCLUDE}
  PATH_SUFFIXES
    lua
    lua5.4
    lua-5.4
    lua.5.4
    lua5.3
    lua-5.3
    lua.5.3
    lua5.2
    lua-5.2
    lua.5.2
    lua5.1
    lua-5.1
    lua.5.1
    luajit-2.1
    luajit-2.0
)
if(RKG_LUA_INCLUDE_HINT)
  list(APPEND _RKG_LUA_INCLUDE_INPUT "${RKG_LUA_INCLUDE_HINT}")
  message(STATUS "Lua detection: header search hit: ${RKG_LUA_INCLUDE_HINT}")
else()
  message(STATUS "Lua detection: header search did not find a candidate.")
endif()

set(_RKG_LUA_VALID_INCLUDE_DIRS "")
foreach(_rkg_dir IN LISTS _RKG_LUA_INCLUDE_INPUT)
  _rkg_append_lua_include_dir(_RKG_LUA_VALID_INCLUDE_DIRS "${_rkg_dir}")
endforeach()
set(_RKG_LUA_INCLUDE_DIRS_FALLBACK "")
foreach(_rkg_link_dir IN LISTS CMAKE_SYSTEM_PREFIX_PATH)
  _rkg_append_lua_include_dir(_RKG_LUA_INCLUDE_DIRS_FALLBACK "${_rkg_link_dir}/include")
  _rkg_append_lua_include_dir(_RKG_LUA_INCLUDE_DIRS_FALLBACK "${_rkg_link_dir}")
endforeach()
if(_RKG_LUA_INCLUDE_DIRS_FALLBACK)
  list(APPEND _RKG_LUA_VALID_INCLUDE_DIRS ${_RKG_LUA_INCLUDE_DIRS_FALLBACK})
  list(REMOVE_DUPLICATES _RKG_LUA_VALID_INCLUDE_DIRS)
endif()
if(_RKG_LUA_VALID_INCLUDE_DIRS)
  set(RKG_LUA_INCLUDE_DIRS "${_RKG_LUA_VALID_INCLUDE_DIRS}")
else()
  set(RKG_LUA_INCLUDE_DIRS "")
endif()

if(RKG_LUA_INCLUDE_DIRS)
  target_include_directories(rkg_plugins_script_lua PUBLIC ${RKG_LUA_INCLUDE_DIRS})
endif()

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

message(STATUS "Lua detection summary (pre-validation):")
message(STATUS "  includes (package): ${_RKG_LUA_INCLUDE_RAW}")
message(STATUS "  includes (resolved): ${RKG_LUA_INCLUDE_DIRS}")
message(STATUS "  libraries: ${RKG_LUA_LINK_LIBS}")
message(STATUS "  header version: ${RKG_LUA_VERSION}")
message(STATUS "  Lua_FOUND: ${Lua_FOUND}; LUA_FOUND: ${LUA_FOUND}")

set(RKG_LUA_HAS_NEWSTATE OFF)
set(RKG_LUA_HAS_OPENLIBS OFF)
set(RKG_LUA_HAS_DODOFILE OFF)
set(RKG_LUA_HAS_DOSTRING OFF)
set(RKG_LUA_HAS_PCALL OFF)
set(RKG_LUA_HAS_CHECKINTEGER OFF)
set(RKG_LUA_HAS_CHECKNUMBER OFF)
set(RKG_LUA_HAS_GETGLOBAL OFF)
set(RKG_LUA_HAS_ISFUNCTION OFF)
set(RKG_LUA_HAS_POP OFF)
set(RKG_LUA_LINK_TEST_OK OFF)
if(RKG_LUA_INCLUDE_DIRS)
  check_symbol_exists(luaL_newstate "lua.h;lauxlib.h;lualib.h" RKG_LUA_HAS_NEWSTATE)
  check_symbol_exists(luaL_openlibs "lua.h;lauxlib.h;lualib.h" RKG_LUA_HAS_OPENLIBS)
  check_symbol_exists(luaL_dofile "lua.h;lauxlib.h;lualib.h" RKG_LUA_HAS_DODOFILE)
  check_symbol_exists(luaL_dostring "lua.h;lauxlib.h;lualib.h" RKG_LUA_HAS_DOSTRING)
  check_symbol_exists(lua_pcall "lua.h;lauxlib.h;lualib.h" RKG_LUA_HAS_PCALL)
  check_symbol_exists(luaL_checkinteger "lua.h;lauxlib.h;lualib.h" RKG_LUA_HAS_CHECKINTEGER)
  check_symbol_exists(luaL_checknumber "lua.h;lauxlib.h;lualib.h" RKG_LUA_HAS_CHECKNUMBER)
  check_symbol_exists(lua_getglobal "lua.h;lauxlib.h;lualib.h" RKG_LUA_HAS_GETGLOBAL)
  check_symbol_exists(lua_isfunction "lua.h;lauxlib.h;lualib.h" RKG_LUA_HAS_ISFUNCTION)
  check_symbol_exists(lua_pop "lua.h;lauxlib.h;lualib.h" RKG_LUA_HAS_POP)
endif()

set(_RKG_LUA_PROBE_LIBS "")
foreach(_rkg_lib IN LISTS RKG_LUA_LINK_LIBS)
  if(_rkg_lib MATCHES "::")
    if(TARGET "${_rkg_lib}")
      get_target_property(_rkg_tgt_location "${_rkg_lib}" IMPORTED_LOCATION)
      if(_rkg_tgt_location AND NOT _rkg_tgt_location STREQUAL "_rkg_tgt_location-NOTFOUND")
        _rkg_append_unique(_RKG_LUA_PROBE_LIBS "${_rkg_tgt_location}")
      endif()
      get_target_property(_rkg_tgt_implib "${_rkg_lib}" IMPORTED_IMPLIB)
      if(_rkg_tgt_implib AND NOT _rkg_tgt_implib STREQUAL "_rkg_tgt_implib-NOTFOUND")
        _rkg_append_unique(_RKG_LUA_PROBE_LIBS "${_rkg_tgt_implib}")
      endif()
    endif()
  else()
    _rkg_append_unique(_RKG_LUA_PROBE_LIBS "${_rkg_lib}")
  endif()
endforeach()
list(REMOVE_DUPLICATES _RKG_LUA_PROBE_LIBS)

if(RKG_LUA_INCLUDE_DIRS AND _RKG_LUA_PROBE_LIBS)
  set(_rkg_lua_api_probe_code [=[
    #include <stddef.h>
    #include <lua.h>
    #include <lauxlib.h>
    #include <lualib.h>

    int main(void) {
      lua_State* (*_rkg_new_state)(void) = luaL_newstate;
      void (*_rkg_open_libs)(lua_State*) = luaL_openlibs;
      int (*_rkg_dofile)(lua_State*, const char*) = luaL_dofile;
      int (*_rkg_load_string)(lua_State*, const char*) = luaL_loadstring;
      int (*_rkg_pcall)(lua_State*, int, int, int) = lua_pcall;
      int (*_rkg_type)(lua_State*, int) = lua_type;
      int (*_rkg_is_function)(lua_State*, int) = lua_isfunction;
      void (*_rkg_pop)(lua_State*, int) = lua_pop;
      void (*_rkg_settop)(lua_State*, int) = lua_settop;
      int (*_rkg_get_global)(lua_State*, const char*) = lua_getglobal;
      void (*_rkg_push_number)(lua_State*, lua_Number) = lua_pushnumber;
      int (*_rkg_push_string)(lua_State*, const char*) = lua_pushstring;
      const char* (*_rkg_to_lstring)(lua_State*, int, size_t*) = lua_tolstring;
      void* (*_rkg_to_userdata)(lua_State*, int) = lua_touserdata;
      lua_Integer (*_rkg_check_integer)(lua_State*, int) = luaL_checkinteger;
      lua_Number (*_rkg_check_number)(lua_State*, int) = luaL_checknumber;
      void (*_rkg_push_light_userdata)(lua_State*, void*) = lua_pushlightuserdata;
      void (*_rkg_push_cclosure)(lua_State*, lua_CFunction, int) = lua_pushcclosure;
      void (*_rkg_set_global)(lua_State*, const char*) = lua_setglobal;
      void (*_rkg_close)(lua_State*) = lua_close;
      return _rkg_new_state ? 0 : 1;
    }
  ]=])
  set(CMAKE_REQUIRED_INCLUDES ${RKG_LUA_INCLUDE_DIRS})
  set(CMAKE_REQUIRED_LIBRARIES ${_RKG_LUA_PROBE_LIBS})
  check_c_source_runs("${_rkg_lua_api_probe_code}" RKG_LUA_LINK_TEST_OK)
  set(CMAKE_REQUIRED_INCLUDES "")
  set(CMAKE_REQUIRED_LIBRARIES "")
else()
  set(RKG_LUA_LINK_TEST_OK OFF)
endif()

if(
  RKG_LUA_LINK_TEST_OK
  AND RKG_LUA_HAS_NEWSTATE
  AND RKG_LUA_HAS_OPENLIBS
  AND RKG_LUA_HAS_DODOFILE
  AND RKG_LUA_HAS_DOSTRING
  AND RKG_LUA_HAS_PCALL
  AND RKG_LUA_HAS_CHECKINTEGER
  AND RKG_LUA_HAS_CHECKNUMBER
  AND RKG_LUA_HAS_GETGLOBAL
  AND RKG_LUA_HAS_ISFUNCTION
  AND RKG_LUA_HAS_POP
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
  message(STATUS "  include dirs found: ${RKG_LUA_INCLUDE_DIRS}")
  message(STATUS "  link libs found: ${RKG_LUA_LINK_LIBS}")
  message(STATUS "  symbol checks: luaL_newstate=${RKG_LUA_HAS_NEWSTATE}, luaL_openlibs=${RKG_LUA_HAS_OPENLIBS}, luaL_dofile=${RKG_LUA_HAS_DODOFILE}, luaL_dostring=${RKG_LUA_HAS_DOSTRING}, lua_pcall=${RKG_LUA_HAS_PCALL}, luaL_checkinteger=${RKG_LUA_HAS_CHECKINTEGER}, luaL_checknumber=${RKG_LUA_HAS_CHECKNUMBER}, lua_getglobal=${RKG_LUA_HAS_GETGLOBAL}, lua_isfunction=${RKG_LUA_HAS_ISFUNCTION}, lua_pop=${RKG_LUA_HAS_POP}, link-probe=${RKG_LUA_LINK_TEST_OK}, version=${RKG_LUA_VERSION}")
  target_compile_definitions(rkg_plugins_script_lua PRIVATE RKG_HAVE_LUA=0)
endif()
