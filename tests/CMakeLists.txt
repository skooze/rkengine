rkg_add_executable(rkg_tests
  src/test_smoke.cpp
)

target_link_libraries(rkg_tests PUBLIC rkg_core rkgctl_lib)
