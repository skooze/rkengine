#include "rkg/paths.h"

#include "rkg/log.h"

#include <cstdlib>

#if defined(_WIN32)
#include <windows.h>
#else
#include <unistd.h>
#endif

namespace rkg {

namespace {
std::filesystem::path executable_dir(const char* argv0) {
#if defined(_WIN32)
  char buffer[MAX_PATH];
  DWORD len = GetModuleFileNameA(nullptr, buffer, MAX_PATH);
  if (len > 0) {
    return std::filesystem::path(buffer).parent_path();
  }
#elif defined(__linux__)
  char buffer[4096];
  const ssize_t len = readlink("/proc/self/exe", buffer, sizeof(buffer) - 1);
  if (len > 0) {
    buffer[len] = '\0';
    return std::filesystem::path(buffer).parent_path();
  }
#endif
  if (argv0) {
    return std::filesystem::absolute(argv0).parent_path();
  }
  return std::filesystem::current_path();
}

std::filesystem::path find_root_from(const std::filesystem::path& start) {
  std::filesystem::path cur = start;
  for (int i = 0; i < 6; ++i) {
    if (std::filesystem::exists(cur / "projects") && std::filesystem::exists(cur / "config")) {
      return cur;
    }
    if (cur.has_parent_path()) {
      cur = cur.parent_path();
    } else {
      break;
    }
  }
  return start;
}

std::filesystem::path resolve_project_path(const std::filesystem::path& root,
                                           const std::optional<std::filesystem::path>& override,
                                           const std::string& default_project_name) {
  if (const char* env = std::getenv("RKG_PROJECT")) {
    return std::filesystem::path(env);
  }
  if (override.has_value()) {
    const auto p = override.value();
    if (p.is_absolute()) return p;
    return root / p;
  }
  return root / "projects" / default_project_name;
}
} // namespace

ResolvedPaths resolve_paths(const char* argv0,
                            const std::optional<std::filesystem::path>& project_override,
                            const std::string& default_project_name) {
  ResolvedPaths out;
  if (const char* env_root = std::getenv("RKG_ROOT")) {
    out.root = std::filesystem::path(env_root);
  } else {
    out.root = find_root_from(executable_dir(argv0));
  }

  out.project = resolve_project_path(out.root, project_override, default_project_name);
  out.content_root = out.project / "content";
  out.shared_content = out.root / "content";
  out.logs_dir = out.root / "build" / "logs";
  out.content_cache_root = out.root / "build" / "content_cache";
  out.dist_root = out.root / "build" / "dist";

  if (!std::filesystem::exists(out.project)) {
    log::warn(std::string("project path not found: ") + out.project.string());
  }

  return out;
}

} // namespace rkg
