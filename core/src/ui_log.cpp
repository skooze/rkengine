#include "rkg/ui_log.h"

#include <mutex>
#include <fstream>

namespace rkg::ui_log {

static std::mutex g_mutex;
static std::ofstream g_file;
static std::filesystem::path g_path;

void open(const std::filesystem::path& path) {
  std::lock_guard<std::mutex> lock(g_mutex);
  g_path = path;
  if (g_file.is_open()) {
    g_file.close();
  }
  g_file.open(g_path, std::ios::out | std::ios::trunc);
}

void close() {
  std::lock_guard<std::mutex> lock(g_mutex);
  if (g_file.is_open()) {
    g_file.close();
  }
}

bool enabled() {
  std::lock_guard<std::mutex> lock(g_mutex);
  return g_file.is_open();
}

void write(std::string_view line) {
  std::lock_guard<std::mutex> lock(g_mutex);
  if (!g_file.is_open()) {
    return;
  }
  g_file << line << "\n";
  g_file.flush();
}

} // namespace rkg::ui_log
