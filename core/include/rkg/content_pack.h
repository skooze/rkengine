#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

namespace rkg::content {

struct PackEntry {
  uint64_t id = 0;
  uint32_t type = 0;
  uint64_t offset = 0;
  uint64_t size = 0;
  std::string path;
};

class PackReader {
 public:
  bool load(const std::filesystem::path& path, std::string& error);
  const PackEntry* find_by_path(const std::string& path) const;
  bool read_entry_data(const PackEntry& entry, std::string& out) const;
  const std::vector<PackEntry>& entries() const { return entries_; }
  const std::filesystem::path& pack_path() const { return path_; }

 private:
  std::filesystem::path path_;
  std::vector<PackEntry> entries_;
  std::unordered_map<std::string, size_t> index_by_path_;
};

} // namespace rkg::content
