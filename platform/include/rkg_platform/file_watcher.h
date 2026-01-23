#pragma once

#include <filesystem>
#include <string>
#include <vector>

namespace rkg::platform {

struct FileChange {
  enum class Type { Added, Modified, Removed };
  std::filesystem::path path;
  Type type = Type::Modified;
};

class FileWatcher {
 public:
  FileWatcher();
  ~FileWatcher();

  bool start(const std::filesystem::path& root);
  void poll(std::vector<FileChange>& out_changes);
  void stop();
  std::string backend_name() const;

  struct Impl;

 private:
  Impl* impl_ = nullptr;
};

} // namespace rkg::platform
