find_package(Lua QUIET)

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

if(Lua_FOUND)
  if(TARGET Lua::Lua)
    target_link_libraries(rkg_plugins_script_lua PUBLIC Lua::Lua)
  else()
    target_include_directories(rkg_plugins_script_lua PUBLIC ${LUA_INCLUDE_DIR})
    target_link_libraries(rkg_plugins_script_lua PUBLIC ${LUA_LIBRARIES})
  endif()
  target_compile_definitions(rkg_plugins_script_lua PRIVATE RKG_HAVE_LUA=1)
else()
  target_compile_definitions(rkg_plugins_script_lua PRIVATE RKG_HAVE_LUA=0)
endif()
