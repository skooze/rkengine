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

if(Lua_FOUND)
  if(LUA_INCLUDE_DIRS)
    list(APPEND RKG_LUA_INCLUDE_DIRS ${LUA_INCLUDE_DIRS})
  endif()
  if(LUA_INCLUDE_DIR)
    list(APPEND RKG_LUA_INCLUDE_DIRS ${LUA_INCLUDE_DIR})
  endif()

  find_library(LUA_AUXLIB NAMES lauxlib5.4 lauxlib)
  find_library(LUA_LUALIB NAMES lualib5.4 lualib)

  if(TARGET Lua::Lua)
    list(APPEND RKG_LUA_LINK_LIBS Lua::Lua)
  else()
    if(RKG_LUA_INCLUDE_DIRS)
      target_include_directories(rkg_plugins_script_lua PUBLIC ${RKG_LUA_INCLUDE_DIRS})
    endif()
    list(APPEND RKG_LUA_LINK_LIBS ${LUA_LIBRARIES})
  endif()

  if(LUA_AUXLIB)
    list(APPEND RKG_LUA_LINK_LIBS ${LUA_AUXLIB})
  endif()
  if(LUA_LUALIB)
    list(APPEND RKG_LUA_LINK_LIBS ${LUA_LUALIB})
  endif()

  # Verify we can link a basic lauxlib symbol; if not, disable Lua to avoid link failures.
  set(CMAKE_REQUIRED_INCLUDES ${RKG_LUA_INCLUDE_DIRS})
  set(CMAKE_REQUIRED_LIBRARIES ${RKG_LUA_LINK_LIBS})
  check_symbol_exists(luaL_newstate "lua.h;lauxlib.h" RKG_LUA_HAS_LAUXLIB)
  set(CMAKE_REQUIRED_INCLUDES "")
  set(CMAKE_REQUIRED_LIBRARIES "")

  if(RKG_LUA_HAS_LAUXLIB OR TARGET Lua::Lua)
    set(RKG_LUA_AVAILABLE ON)
  else()
    message(WARNING "Lua found but lauxlib symbols are not linkable; disabling Lua scripting.")
  endif()
endif()

if(RKG_LUA_AVAILABLE)
  target_link_libraries(rkg_plugins_script_lua PUBLIC ${RKG_LUA_LINK_LIBS})
  target_compile_definitions(rkg_plugins_script_lua PRIVATE RKG_HAVE_LUA=1)
else()
  target_compile_definitions(rkg_plugins_script_lua PRIVATE RKG_HAVE_LUA=0)
endif()
