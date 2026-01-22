#include "subprocess_runner.h"

#include <algorithm>
#include <cstring>

#if defined(_WIN32)
#include <windows.h>
#else
#include <fcntl.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace rkg::editor {
namespace {
constexpr size_t kMaxLines = 200;
}

SubprocessRunner::~SubprocessRunner() {
  request_stop();
  if (reader_thread_.joinable()) {
    reader_thread_.join();
  }
}

void SubprocessRunner::clear_output() {
  std::lock_guard<std::mutex> lock(output_mutex_);
  output_lines_.clear();
}

void SubprocessRunner::append_line(const std::string& line) {
  std::lock_guard<std::mutex> lock(output_mutex_);
  output_lines_.push_back(line);
  while (output_lines_.size() > kMaxLines) {
    output_lines_.pop_front();
  }
}

std::vector<std::string> SubprocessRunner::recent_lines(size_t max_lines) const {
  std::lock_guard<std::mutex> lock(output_mutex_);
  const size_t count = std::min(max_lines, output_lines_.size());
  return std::vector<std::string>(output_lines_.end() - count, output_lines_.end());
}

bool SubprocessRunner::start(const std::vector<std::string>& args, const std::filesystem::path& cwd) {
  if (running_) {
    error_message_ = "process already running";
    return false;
  }
  if (reader_thread_.joinable()) {
    reader_thread_.join();
  }
  error_message_.clear();
  clear_output();
  finished_ = false;
  exit_code_ = -1;

#if defined(_WIN32)
  (void)args;
  (void)cwd;
  error_message_ = "subprocess runner not implemented on Windows";
  running_ = false;
  finished_ = true;
  return false;
#else
  if (args.empty()) {
    error_message_ = "missing command";
    return false;
  }

  int pipefd[2];
  if (pipe(pipefd) != 0) {
    error_message_ = "pipe failed";
    return false;
  }

  pid_t pid = fork();
  if (pid == 0) {
    if (!cwd.empty()) {
      ::chdir(cwd.c_str());
    }
    ::dup2(pipefd[1], STDOUT_FILENO);
    ::dup2(pipefd[1], STDERR_FILENO);
    ::close(pipefd[0]);
    ::close(pipefd[1]);

    std::vector<char*> cargs;
    cargs.reserve(args.size() + 1);
    for (const auto& arg : args) {
      cargs.push_back(const_cast<char*>(arg.c_str()));
    }
    cargs.push_back(nullptr);
    ::execvp(cargs[0], cargs.data());
    _exit(127);
  }

  if (pid < 0) {
    ::close(pipefd[0]);
    ::close(pipefd[1]);
    error_message_ = "fork failed";
    return false;
  }

  ::close(pipefd[1]);
  read_fd_ = pipefd[0];
  pid_ = static_cast<int>(pid);
  running_ = true;
  finished_ = false;

  reader_thread_ = std::thread(&SubprocessRunner::reader_loop, this);
  return true;
#endif
}

void SubprocessRunner::request_stop() {
#if defined(_WIN32)
  if (process_handle_) {
    TerminateProcess(static_cast<HANDLE>(process_handle_), 1);
  }
#else
  if (running_ && pid_ > 0) {
    ::kill(pid_, SIGTERM);
  }
#endif
}

void SubprocessRunner::update() {
  if (finished_ && reader_thread_.joinable()) {
    reader_thread_.join();
  }
}

void SubprocessRunner::reader_loop() {
#if defined(_WIN32)
  return;
#else
  FILE* stream = ::fdopen(read_fd_, "r");
  if (stream) {
    char buffer[512];
    while (fgets(buffer, sizeof(buffer), stream)) {
      std::string line(buffer);
      while (!line.empty() && (line.back() == '\n' || line.back() == '\r')) {
        line.pop_back();
      }
      if (!line.empty()) {
        append_line(line);
      }
    }
    ::fclose(stream);
  } else if (read_fd_ >= 0) {
    ::close(read_fd_);
  }

  int status = 0;
  if (pid_ > 0) {
    ::waitpid(pid_, &status, 0);
    if (WIFEXITED(status)) {
      exit_code_ = WEXITSTATUS(status);
    } else if (WIFSIGNALED(status)) {
      exit_code_ = 128 + WTERMSIG(status);
    }
  }
  running_ = false;
  finished_ = true;
#endif
}

} // namespace rkg::editor
