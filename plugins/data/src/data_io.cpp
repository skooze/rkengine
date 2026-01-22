#include "rkg_data/serialization.h"

#include "rkg/log.h"

#include <fstream>
#include <sstream>

namespace rkg::data {

std::string read_text_file(const std::filesystem::path& path) {
  std::ifstream in(path);
  if (!in) {
    rkg::log::warn(std::string("failed to read file: ") + path.string());
    return {};
  }
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

bool write_text_file(const std::filesystem::path& path, const std::string& contents) {
  std::ofstream out(path, std::ios::trunc);
  if (!out) {
    rkg::log::warn(std::string("failed to write file: ") + path.string());
    return false;
  }
  out << contents;
  return true;
}

} // namespace rkg::data
