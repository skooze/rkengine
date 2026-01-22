#include "rkg/log.h"

#include <algorithm>
#include <chrono>
#include <csignal>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>

#if defined(_WIN32)
#include <windows.h>
#endif

namespace rkg::log {

namespace {
std::mutex g_log_mutex;
std::ofstream g_log_file;
std::deque<std::string> g_ring;
constexpr size_t kRingMax = 200;
std::string g_app_name = "rkg";
std::filesystem::path g_root_path;

std::string timestamp_now() {
  using namespace std::chrono;
  const auto now = system_clock::now();
  const auto tt = system_clock::to_time_t(now);
  std::tm tm{};
#if defined(_WIN32)
  localtime_s(&tm, &tt);
#else
  localtime_r(&tt, &tm);
#endif
  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
  return oss.str();
}

std::string timestamp_for_filename() {
  using namespace std::chrono;
  const auto now = system_clock::now();
  const auto tt = system_clock::to_time_t(now);
  std::tm tm{};
#if defined(_WIN32)
  localtime_s(&tm, &tt);
#else
  localtime_r(&tt, &tm);
#endif
  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
  return oss.str();
}

void log_line(const char* level, std::string_view msg) {
  std::lock_guard<std::mutex> lock(g_log_mutex);
  const std::string line = "[" + timestamp_now() + "][" + level + "] " + std::string(msg);
  std::cout << line << "\n";
  if (g_log_file.is_open()) {
    g_log_file << line << "\n";
    g_log_file.flush();
  }
  g_ring.push_back(line);
  if (g_ring.size() > kRingMax) {
    g_ring.pop_front();
  }
}
} // namespace

void init() {
  init("rkg", std::filesystem::current_path());
}

void init(const std::string& app_name, const std::filesystem::path& root) {
  g_app_name = app_name;
  g_root_path = root;
  std::filesystem::path log_dir = g_root_path / "build" / "logs";
  std::error_code ec;
  std::filesystem::create_directories(log_dir, ec);
  const std::string file_name = g_app_name + "_" + timestamp_for_filename() + ".log";
  const std::filesystem::path log_path = log_dir / file_name;
  g_log_file.open(log_path, std::ios::out | std::ios::app);
  log_line("INFO", "log init");
#if defined(_WIN32)
  log_line("INFO", "platform: windows");
#elif defined(__linux__)
  log_line("INFO", "platform: linux");
#else
  log_line("INFO", "platform: unknown");
#endif
#ifdef RKG_DEBUG
  log_line("INFO", "build: debug");
#else
  log_line("INFO", "build: release");
#endif
#ifdef RKG_GIT_HASH
  log_line("INFO", std::string("git: ") + RKG_GIT_HASH);
#endif
}

void shutdown() {
  log_line("INFO", "log shutdown");
  if (g_log_file.is_open()) {
    g_log_file.close();
  }
}

void info(std::string_view msg) {
  log_line("INFO", msg);
}

void warn(std::string_view msg) {
  log_line("WARN", msg);
}

void error(std::string_view msg) {
  log_line("ERROR", msg);
}

std::vector<std::string> recent(size_t max_entries) {
  std::lock_guard<std::mutex> lock(g_log_mutex);
  const size_t count = std::min(max_entries, g_ring.size());
  return std::vector<std::string>(g_ring.end() - count, g_ring.end());
}

namespace {
void signal_handler(int sig) {
  log_line("ERROR", std::string("crash signal: ") + std::to_string(sig));
  std::_Exit(1);
}

#if defined(_WIN32)
LONG WINAPI exception_filter(EXCEPTION_POINTERS*) {
  log_line("ERROR", "unhandled exception");
  return EXCEPTION_EXECUTE_HANDLER;
}
#endif
} // namespace

void install_crash_handlers() {
  std::signal(SIGSEGV, signal_handler);
  std::signal(SIGABRT, signal_handler);
  std::signal(SIGFPE, signal_handler);
  std::signal(SIGILL, signal_handler);
#if defined(_WIN32)
  SetUnhandledExceptionFilter(exception_filter);
#endif
}

} // namespace rkg::log
