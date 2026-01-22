#include "rkg_data/serialization.h"

#include "rkg/log.h"

#if RKG_ENABLE_DATA_YAML
#include <yaml-cpp/yaml.h>
#endif

namespace rkg::data {

#if RKG_ENABLE_DATA_YAML
bool load_yaml_file(const std::filesystem::path& path, YAML::Node& out) {
  try {
    out = YAML::LoadFile(path.string());
    return true;
  } catch (const std::exception& e) {
    rkg::log::warn(std::string("YAML load failed: ") + e.what());
    return false;
  }
}

bool save_yaml_file(const std::filesystem::path& path, const YAML::Node& node) {
  try {
    YAML::Emitter emitter;
    emitter << node;
    return write_text_file(path, emitter.c_str());
  } catch (const std::exception& e) {
    rkg::log::warn(std::string("YAML save failed: ") + e.what());
    return false;
  }
}
#endif

} // namespace rkg::data
