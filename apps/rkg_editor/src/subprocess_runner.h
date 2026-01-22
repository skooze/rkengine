#pragma once

#include <atomic>
#include <deque>
#include <filesystem>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace rkg::editor {

class SubprocessRunner {
 public:
  SubprocessRunner() = default;
  ~SubprocessRunner();

  SubprocessRunner(const SubprocessRunner&) = delete;
  SubprocessRunner& operator=(const SubprocessRunner&) = delete;

  bool start(const std::vector<std::string>& args, const std::filesystem::path& cwd);
  void request_stop();
  void update();

  bool running() const { return running_.load(); }
  bool finished() const { return finished_.load(); }
  int exit_code() const { return exit_code_.load(); }
  const std::string& error_message() const { return error_message_; }

  std::vector<std::string> recent_lines(size_t max_lines) const;
  void clear_output();

 private:
  void append_line(const std::string& line);
  void reader_loop();

  std::atomic<bool> running_{false};
  std::atomic<bool> finished_{false};
  std::atomic<int> exit_code_{-1};

  std::string error_message_;

  mutable std::mutex output_mutex_;
  std::deque<std::string> output_lines_;

#if defined(_WIN32)
  void* process_handle_ = nullptr;
#else
  int read_fd_ = -1;
  int pid_ = -1;
#endif

  std::thread reader_thread_;
};

} // namespace rkg::editor
