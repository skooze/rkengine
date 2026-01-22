#include "rkg_data/serialization.h"

#include "rkg/log.h"

#if RKG_ENABLE_DATA_JSON
#include <nlohmann/json.hpp>
#endif

#include <fstream>

namespace rkg::data {

#if RKG_ENABLE_DATA_JSON
bool load_json_file(const std::filesystem::path& path, nlohmann::json& out) {
  std::ifstream in(path);
  if (!in) {
    rkg::log::warn(std::string("JSON read failed: ") + path.string());
    return false;
  }
  try {
    in >> out;
  } catch (const std::exception& e) {
    rkg::log::warn(std::string("JSON parse failed: ") + e.what());
    return false;
  }
  return true;
}

bool save_json_file(const std::filesystem::path& path, const nlohmann::json& node) {
  std::ofstream out(path);
  if (!out) {
    rkg::log::warn(std::string("JSON write failed: ") + path.string());
    return false;
  }
  out << node.dump(2);
  return true;
}
#endif

} // namespace rkg::data
