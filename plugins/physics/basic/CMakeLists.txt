rkg_add_library(rkg_plugins_physics_basic
  src/physics_basic.cpp
)

target_link_libraries(rkg_plugins_physics_basic
  PUBLIC
    rkg_core
)

target_compile_definitions(rkg_plugins_physics_basic PUBLIC RKG_ENABLE_PHYSICS_BASIC=1)
