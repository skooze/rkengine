rkg_add_executable(rkg_tests
  src/test_smoke.cpp
)

target_link_libraries(rkg_tests PUBLIC rkg_core rkg_runtime rkgctl_lib)

if(RKG_ENABLE_PHYSICS_BASIC)
  target_link_libraries(rkg_tests PUBLIC rkg_plugins_physics_basic)
endif()
