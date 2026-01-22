#include "rkg_platform/file_watcher.h"

#include "rkg/log.h"

#include <chrono>
#include <unordered_map>

#if defined(_WIN32)
#include <windows.h>
#endif

#if defined(__linux__)
#include <sys/inotify.h>
#include <unistd.h>
#endif

namespace rkg::platform {

struct FileWatcher::Impl {
  virtual ~Impl() = default;
  virtual bool start(const std::filesystem::path& root) = 0;
  virtual void poll(std::vector<FileChange>& out_changes) = 0;
  virtual void stop() = 0;
  virtual std::string name() const = 0;
};

class PollingWatcher final : public FileWatcher::Impl {
 public:
  bool start(const std::filesystem::path& root) override {
    root_ = root;
    snapshot();
    return true;
  }

  void poll(std::vector<FileChange>& out_changes) override {
    std::unordered_map<std::string, uint64_t> current;
    if (!std::filesystem::exists(root_)) return;
    for (const auto& entry : std::filesystem::recursive_directory_iterator(root_)) {
      if (!entry.is_regular_file()) continue;
      const auto path = entry.path();
      const auto key = path.generic_string();
      const auto mtime = file_mtime_epoch(path);
      current[key] = mtime;
      auto it = mtimes_.find(key);
      if (it == mtimes_.end()) {
        out_changes.push_back({path, FileChange::Type::Added});
      } else if (it->second != mtime) {
        out_changes.push_back({path, FileChange::Type::Modified});
      }
    }
    for (const auto& prev : mtimes_) {
      if (current.find(prev.first) == current.end()) {
        out_changes.push_back({prev.first, FileChange::Type::Removed});
      }
    }
    mtimes_.swap(current);
  }

  void stop() override {
    mtimes_.clear();
  }

  std::string name() const override { return "polling"; }

 private:
  uint64_t file_mtime_epoch(const std::filesystem::path& path) {
    std::error_code ec;
    auto ftime = std::filesystem::last_write_time(path, ec);
    if (ec) return 0;
    auto sctp = std::chrono::time_point_cast<std::chrono::system_clock::duration>(
        ftime - std::filesystem::file_time_type::clock::now() + std::chrono::system_clock::now());
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::seconds>(sctp.time_since_epoch()).count());
  }

  void snapshot() {
    mtimes_.clear();
    if (!std::filesystem::exists(root_)) return;
    for (const auto& entry : std::filesystem::recursive_directory_iterator(root_)) {
      if (!entry.is_regular_file()) continue;
      const auto path = entry.path();
      mtimes_[path.generic_string()] = file_mtime_epoch(path);
    }
  }

  std::filesystem::path root_;
  std::unordered_map<std::string, uint64_t> mtimes_;
};

#if defined(_WIN32)
class WindowsWatcher final : public FileWatcher::Impl {
 public:
  bool start(const std::filesystem::path& root) override {
    stop();
    root_ = root;
    if (!std::filesystem::exists(root_)) {
      return false;
    }
    handle_ = CreateFileW(root_.wstring().c_str(),
                          FILE_LIST_DIRECTORY,
                          FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
                          nullptr,
                          OPEN_EXISTING,
                          FILE_FLAG_BACKUP_SEMANTICS | FILE_FLAG_OVERLAPPED,
                          nullptr);
    if (handle_ == INVALID_HANDLE_VALUE) {
      rkg::log::warn("ReadDirectoryChangesW open failed; using polling watcher");
      return false;
    }
    event_ = CreateEventW(nullptr, TRUE, FALSE, nullptr);
    buffer_.resize(16 * 1024);
    return issue_read();
  }

  void poll(std::vector<FileChange>& out_changes) override {
    if (handle_ == INVALID_HANDLE_VALUE) return;
    DWORD bytes = 0;
    if (!GetOverlappedResult(handle_, &overlapped_, &bytes, FALSE)) {
      if (GetLastError() == ERROR_IO_INCOMPLETE) {
        return;
      }
      issue_read();
      return;
    }
    if (bytes == 0) {
      issue_read();
      return;
    }
    parse_changes(bytes, out_changes);
    issue_read();
  }

  void stop() override {
    if (handle_ != INVALID_HANDLE_VALUE) {
      CancelIo(handle_);
      CloseHandle(handle_);
      handle_ = INVALID_HANDLE_VALUE;
    }
    if (event_ != nullptr) {
      CloseHandle(event_);
      event_ = nullptr;
    }
    buffer_.clear();
    ZeroMemory(&overlapped_, sizeof(overlapped_));
  }

  std::string name() const override { return "read_dir_changes"; }

 private:
  bool issue_read() {
    if (handle_ == INVALID_HANDLE_VALUE) return false;
    ResetEvent(event_);
    ZeroMemory(&overlapped_, sizeof(overlapped_));
    overlapped_.hEvent = event_;
    const DWORD notify_filter = FILE_NOTIFY_CHANGE_FILE_NAME |
                                FILE_NOTIFY_CHANGE_DIR_NAME |
                                FILE_NOTIFY_CHANGE_LAST_WRITE;
    return ReadDirectoryChangesW(handle_,
                                 buffer_.data(),
                                 static_cast<DWORD>(buffer_.size()),
                                 TRUE,
                                 notify_filter,
                                 nullptr,
                                 &overlapped_,
                                 nullptr) != 0;
  }

  void parse_changes(DWORD bytes, std::vector<FileChange>& out_changes) {
    size_t offset = 0;
    while (offset < bytes) {
      auto* info = reinterpret_cast<FILE_NOTIFY_INFORMATION*>(buffer_.data() + offset);
      std::wstring name(info->FileName, info->FileNameLength / sizeof(WCHAR));
      std::filesystem::path path = root_ / name;
      FileChange::Type type = FileChange::Type::Modified;
      switch (info->Action) {
        case FILE_ACTION_ADDED:
          type = FileChange::Type::Added;
          break;
        case FILE_ACTION_REMOVED:
          type = FileChange::Type::Removed;
          break;
        case FILE_ACTION_RENAMED_NEW_NAME:
        case FILE_ACTION_RENAMED_OLD_NAME:
        case FILE_ACTION_MODIFIED:
        default:
          type = FileChange::Type::Modified;
          break;
      }
      out_changes.push_back({path, type});
      if (info->NextEntryOffset == 0) break;
      offset += info->NextEntryOffset;
    }
  }

  std::filesystem::path root_;
  HANDLE handle_ = INVALID_HANDLE_VALUE;
  HANDLE event_ = nullptr;
  OVERLAPPED overlapped_{};
  std::vector<char> buffer_;
};
#endif

#if defined(__linux__)
class InotifyWatcher final : public FileWatcher::Impl {
 public:
  bool start(const std::filesystem::path& root) override {
    root_ = root;
    fd_ = inotify_init1(IN_NONBLOCK);
    if (fd_ < 0) {
      rkg::log::warn("inotify init failed; using polling watcher");
      return false;
    }
    add_watch_recursive(root_);
    return true;
  }

  void poll(std::vector<FileChange>& out_changes) override {
    if (fd_ < 0) return;
    alignas(struct inotify_event) char buffer[4096];
    ssize_t len = 0;
    while ((len = read(fd_, buffer, sizeof(buffer))) > 0) {
      size_t offset = 0;
      while (offset < static_cast<size_t>(len)) {
        const auto* event = reinterpret_cast<const struct inotify_event*>(buffer + offset);
        offset += sizeof(struct inotify_event) + event->len;
        auto it = watch_paths_.find(event->wd);
        if (it == watch_paths_.end()) {
          continue;
        }
        std::filesystem::path path = it->second;
        if (event->len > 0) {
          path /= std::string(event->name);
        }

        if (event->mask & (IN_CREATE | IN_MOVED_TO)) {
          out_changes.push_back({path, FileChange::Type::Added});
        } else if (event->mask & (IN_DELETE | IN_MOVED_FROM)) {
          out_changes.push_back({path, FileChange::Type::Removed});
        } else if (event->mask & IN_MODIFY) {
          out_changes.push_back({path, FileChange::Type::Modified});
        }
      }
    }
  }

  void stop() override {
    if (fd_ >= 0) {
      close(fd_);
      fd_ = -1;
    }
    watch_paths_.clear();
  }

  std::string name() const override { return "inotify"; }

 private:
  void add_watch_recursive(const std::filesystem::path& dir) {
    if (!std::filesystem::exists(dir)) return;
    if (std::filesystem::is_directory(dir)) {
      const int wd = inotify_add_watch(fd_, dir.c_str(),
                                       IN_CREATE | IN_MODIFY | IN_DELETE | IN_MOVED_FROM | IN_MOVED_TO);
      if (wd >= 0) {
        watch_paths_[wd] = dir;
      }
      for (const auto& entry : std::filesystem::directory_iterator(dir)) {
        if (entry.is_directory()) {
          add_watch_recursive(entry.path());
        }
      }
    }
  }

  int fd_ = -1;
  std::filesystem::path root_;
  std::unordered_map<int, std::filesystem::path> watch_paths_;
};
#endif

FileWatcher::FileWatcher() {
#if defined(_WIN32)
  auto* win_impl = new WindowsWatcher();
  impl_ = win_impl;
  if (!impl_->start(".")) {
    delete impl_;
    impl_ = new PollingWatcher();
  } else {
    impl_->stop();
  }
#elif defined(__linux__)
  auto* inotify_impl = new InotifyWatcher();
  impl_ = inotify_impl;
  if (!impl_->start(".")) {
    delete impl_;
    impl_ = new PollingWatcher();
  } else {
    impl_->stop();
  }
#else
  impl_ = new PollingWatcher();
#endif
}

FileWatcher::~FileWatcher() {
  if (impl_) {
    impl_->stop();
    delete impl_;
  }
}

bool FileWatcher::start(const std::filesystem::path& root) {
#if defined(__linux__)
  if (impl_->name() == "inotify") {
    impl_->stop();
  }
#endif
  return impl_->start(root);
}

void FileWatcher::poll(std::vector<FileChange>& out_changes) {
  if (impl_) {
    impl_->poll(out_changes);
  }
}

void FileWatcher::stop() {
  if (impl_) {
    impl_->stop();
  }
}

std::string FileWatcher::backend_name() const {
  return impl_ ? impl_->name() : "none";
}

} // namespace rkg::platform
