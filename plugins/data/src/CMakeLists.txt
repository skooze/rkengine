rkg_add_library(rkg_plugins_data
  database_sqlite.cpp
  data_yaml.cpp
  data_json.cpp
  data_io.cpp
)

target_include_directories(rkg_plugins_data
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/../include
)

target_link_libraries(rkg_plugins_data
  PUBLIC
    rkg_core
    rkg_sqlite3
)

if(RKG_ENABLE_DATA_JSON)
  target_link_libraries(rkg_plugins_data PUBLIC nlohmann_json::nlohmann_json)
  target_compile_definitions(rkg_plugins_data PUBLIC RKG_ENABLE_DATA_JSON=1)
else()
  target_compile_definitions(rkg_plugins_data PUBLIC RKG_ENABLE_DATA_JSON=0)
endif()

if(RKG_ENABLE_DATA_YAML)
  target_link_libraries(rkg_plugins_data PUBLIC yaml-cpp)
  target_compile_definitions(rkg_plugins_data PUBLIC RKG_ENABLE_DATA_YAML=1)
else()
  target_compile_definitions(rkg_plugins_data PUBLIC RKG_ENABLE_DATA_YAML=0)
endif()
