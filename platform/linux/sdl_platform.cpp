#include "rkg_platform/platform.h"

#include "rkg/log.h"

#define SDL_MAIN_HANDLED
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include <cerrno>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <pwd.h>
#include <dlfcn.h>
#include <chrono>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>
#include <thread>
#include <sys/resource.h>
#include <string>
#include <vector>

namespace rkg::platform::detail {

KeyCode map_sdl_scancode(SDL_Scancode scancode) {
  switch (scancode) {
    case SDL_SCANCODE_W:
      return KeyCode::W;
    case SDL_SCANCODE_A:
      return KeyCode::A;
    case SDL_SCANCODE_S:
      return KeyCode::S;
    case SDL_SCANCODE_D:
      return KeyCode::D;
    case SDL_SCANCODE_ESCAPE:
      return KeyCode::Escape;
    case SDL_SCANCODE_F1:
      return KeyCode::F1;
    case SDL_SCANCODE_F5:
      return KeyCode::F5;
    default:
      return KeyCode::Unknown;
  }
}

std::string format_errno();

namespace {

std::string list_video_drivers() {
  const int count = SDL_GetNumVideoDrivers();
  if (count <= 0) return "none";
  std::string list;
  for (int i = 0; i < count; ++i) {
    const char* name = SDL_GetVideoDriver(i);
    if (!name) continue;
    if (!list.empty()) list += ", ";
    list += name;
  }
  return list.empty() ? "none" : list;
}

const char* sdl_log_category_name(int category) {
  switch (category) {
    case SDL_LOG_CATEGORY_APPLICATION:
      return "app";
    case SDL_LOG_CATEGORY_SYSTEM:
      return "system";
    case SDL_LOG_CATEGORY_VIDEO:
      return "video";
    case SDL_LOG_CATEGORY_RENDER:
      return "render";
    case SDL_LOG_CATEGORY_INPUT:
      return "input";
    case SDL_LOG_CATEGORY_TEST:
      return "test";
    case SDL_LOG_CATEGORY_ERROR:
      return "error";
    default:
      return "other";
  }
}

const char* sdl_log_priority_name(SDL_LogPriority priority) {
  switch (priority) {
    case SDL_LOG_PRIORITY_VERBOSE:
      return "verbose";
    case SDL_LOG_PRIORITY_DEBUG:
      return "debug";
    case SDL_LOG_PRIORITY_INFO:
      return "info";
    case SDL_LOG_PRIORITY_WARN:
      return "warn";
    case SDL_LOG_PRIORITY_ERROR:
      return "error";
    case SDL_LOG_PRIORITY_CRITICAL:
      return "critical";
    default:
      return "unknown";
  }
}

void sdl_log_output(void*, int category, SDL_LogPriority priority, const char* message) {
  if (!message) {
    return;
  }
  std::string line = std::string("SDL ") + sdl_log_category_name(category) + "/" +
                     sdl_log_priority_name(priority) + ": " + message;
  if (priority >= SDL_LOG_PRIORITY_ERROR) {
    rkg::log::error(line);
  } else if (priority >= SDL_LOG_PRIORITY_WARN) {
    rkg::log::warn(line);
  } else {
    rkg::log::info(line);
  }
}

void install_sdl_logging() {
  static bool installed = false;
  if (installed) return;
  SDL_SetLogOutputFunction(sdl_log_output, nullptr);
  SDL_SetLogPriorities(SDL_LOG_PRIORITY_VERBOSE);
  SDL_SetLogPriority(SDL_LOG_CATEGORY_VIDEO, SDL_LOG_PRIORITY_VERBOSE);
  SDL_SetLogPriority(SDL_LOG_CATEGORY_INPUT, SDL_LOG_PRIORITY_VERBOSE);
  SDL_SetLogPriority(SDL_LOG_CATEGORY_SYSTEM, SDL_LOG_PRIORITY_VERBOSE);
  installed = true;
}

bool have_shared_lib(const char* name) {
  if (!name || !*name) return false;
  void* handle = dlopen(name, RTLD_LAZY | RTLD_LOCAL);
  if (!handle) {
    return false;
  }
  dlclose(handle);
  return true;
}

void log_missing_libs(const char* label, const std::vector<const char*>& libs) {
  for (const char* lib : libs) {
    if (!have_shared_lib(lib)) {
      rkg::log::warn(std::string(label) + " missing shared library: " + lib);
    }
  }
}

void log_wayland_connect(const char* display_name) {
  void* lib = dlopen("libwayland-client.so.0", RTLD_LAZY | RTLD_LOCAL);
  if (!lib) {
    rkg::log::warn("Wayland connect check skipped: libwayland-client.so.0 not loaded.");
    return;
  }
  using DisplayConnectFn = void* (*)(const char*);
  using DisplayDisconnectFn = void (*)(void*);
  auto connect_fn = reinterpret_cast<DisplayConnectFn>(dlsym(lib, "wl_display_connect"));
  auto disconnect_fn = reinterpret_cast<DisplayDisconnectFn>(dlsym(lib, "wl_display_disconnect"));
  if (!connect_fn || !disconnect_fn) {
    rkg::log::warn("Wayland connect check skipped: wl_display_connect not found.");
    dlclose(lib);
    return;
  }
  errno = 0;
  void* display = connect_fn(display_name && *display_name ? display_name : nullptr);
  if (display) {
    rkg::log::info(std::string("Wayland wl_display_connect ok: ") +
                   (display_name && *display_name ? display_name : "(default)"));
    disconnect_fn(display);
  } else {
    rkg::log::warn(std::string("Wayland wl_display_connect failed: ") +
                   (display_name && *display_name ? display_name : "(default)") + format_errno());
  }
  dlclose(lib);
}

void log_x11_connect(const char* display_name) {
  void* lib = dlopen("libX11.so.6", RTLD_LAZY | RTLD_LOCAL);
  if (!lib) {
    rkg::log::warn("X11 connect check skipped: libX11.so.6 not loaded.");
    return;
  }
  using OpenDisplayFn = void* (*)(const char*);
  using CloseDisplayFn = int (*)(void*);
  auto open_fn = reinterpret_cast<OpenDisplayFn>(dlsym(lib, "XOpenDisplay"));
  auto close_fn = reinterpret_cast<CloseDisplayFn>(dlsym(lib, "XCloseDisplay"));
  if (!open_fn || !close_fn) {
    rkg::log::warn("X11 connect check skipped: XOpenDisplay not found.");
    dlclose(lib);
    return;
  }
  errno = 0;
  void* display = open_fn(display_name && *display_name ? display_name : nullptr);
  if (display) {
    rkg::log::info(std::string("X11 XOpenDisplay ok: ") +
                   (display_name && *display_name ? display_name : "(default)"));
    close_fn(display);
  } else {
    rkg::log::warn(std::string("X11 XOpenDisplay failed: ") +
                   (display_name && *display_name ? display_name : "(default)") + format_errno());
  }
  dlclose(lib);
}

std::string x11_socket_path(const char* display) {
  if (!display || *display != ':') return "";
  const char* p = display + 1;
  int num = 0;
  if (!std::isdigit(static_cast<unsigned char>(*p))) return "";
  while (std::isdigit(static_cast<unsigned char>(*p))) {
    num = (num * 10) + (*p - '0');
    ++p;
  }
  return std::string("/tmp/.X11-unix/X") + std::to_string(num);
}

std::string format_mode(mode_t mode) {
  std::string out;
  out.reserve(10);
  out.push_back(S_ISDIR(mode) ? 'd' : S_ISLNK(mode) ? 'l' : S_ISSOCK(mode) ? 's' : '-');
  const char* rwx = "rwx";
  for (int i = 0; i < 9; ++i) {
    out.push_back((mode & (1 << (8 - i))) ? rwx[i % 3] : '-');
  }
  return out;
}

bool log_unix_socket_connect(const std::string& path, const char* label) {
  if (path.empty()) return false;
  const int fd = socket(AF_UNIX, SOCK_STREAM, 0);
  if (fd < 0) {
    rkg::log::warn(std::string(label) + " socket open failed: " + path + format_errno());
    return false;
  }
  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  if (path.size() >= sizeof(addr.sun_path)) {
    rkg::log::warn(std::string(label) + " socket path too long: " + path);
    close(fd);
    return false;
  }
  std::strncpy(addr.sun_path, path.c_str(), sizeof(addr.sun_path) - 1);
  bool ok = (connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0);
  if (ok) {
    rkg::log::info(std::string(label) + " socket connect ok: " + path);
  } else {
    rkg::log::warn(std::string(label) + " socket connect failed: " + path + format_errno());
  }
  close(fd);
  return ok;
}

void log_rlimit_nofile() {
  struct rlimit lim;
  if (getrlimit(RLIMIT_NOFILE, &lim) == 0) {
    rkg::log::info(std::string("RLIMIT_NOFILE: ") + std::to_string(lim.rlim_cur) +
                   "/" + std::to_string(lim.rlim_max));
  }
#ifdef RLIMIT_NPROC
  if (getrlimit(RLIMIT_NPROC, &lim) == 0) {
    rkg::log::info(std::string("RLIMIT_NPROC: ") + std::to_string(lim.rlim_cur) +
                   "/" + std::to_string(lim.rlim_max));
  }
#endif
}

#ifndef SDL_HINT_INPUT_LINUXEV
#define SDL_HINT_INPUT_LINUXEV "SDL_INPUT_LINUXEV"
#endif
#ifndef SDL_HINT_INPUT_EVDEV
#define SDL_HINT_INPUT_EVDEV "SDL_INPUT_EVDEV"
#endif

} // namespace

bool platform_init(Platform* self, const WindowDesc& desc) {
  const Uint32 init_flags = SDL_INIT_VIDEO;
  Uint32 window_flags = SDL_WINDOW_RESIZABLE;
#if RKG_ENABLE_VULKAN
  window_flags |= SDL_WINDOW_VULKAN;
#endif
  const bool is_root = (geteuid() == 0);

  if (is_root) {
    const char* sudo_user = std::getenv("SUDO_USER");
    if (sudo_user && *sudo_user) {
      struct passwd* pw = getpwnam(sudo_user);
      if (pw) {
        if (!std::getenv("XDG_RUNTIME_DIR")) {
          std::string runtime_dir = std::string("/run/user/") + std::to_string(pw->pw_uid);
          if (access(runtime_dir.c_str(), R_OK | X_OK) == 0) {
            setenv("XDG_RUNTIME_DIR", runtime_dir.c_str(), 1);
          }
        }
        if (!std::getenv("XAUTHORITY") && pw->pw_dir) {
          std::string xauth = std::string(pw->pw_dir) + "/.Xauthority";
          if (access(xauth.c_str(), R_OK) == 0) {
            setenv("XAUTHORITY", xauth.c_str(), 1);
          }
        }
        if (!std::getenv("WAYLAND_DISPLAY")) {
          std::string wayland_socket = std::string("/run/user/") + std::to_string(pw->pw_uid) + "/wayland-0";
          if (access(wayland_socket.c_str(), R_OK | W_OK) == 0) {
            setenv("WAYLAND_DISPLAY", "wayland-0", 1);
          }
        }
      }
    }
  }

  SDL_SetMainReady();
  log_rlimit_nofile();
  install_sdl_logging();

  const char* env_driver = std::getenv("SDL_VIDEODRIVER");
  const char* env_linuxev = std::getenv("SDL_INPUT_LINUXEV");
  const char* env_evdev = std::getenv("SDL_INPUT_EVDEV");
  const char* env_xdg_runtime = std::getenv("XDG_RUNTIME_DIR");
  const char* display = std::getenv("DISPLAY");
  const char* wayland_display = std::getenv("WAYLAND_DISPLAY");
  const char* session_type = std::getenv("XDG_SESSION_TYPE");
  const char* env_xauthority = std::getenv("XAUTHORITY");
  const char* env_home = std::getenv("HOME");
  if (!env_xauthority || !*env_xauthority) {
    if (env_home && *env_home) {
      std::string xauth = std::string(env_home) + "/.Xauthority";
      if (access(xauth.c_str(), R_OK) == 0) {
        setenv("XAUTHORITY", xauth.c_str(), 1);
        env_xauthority = std::getenv("XAUTHORITY");
      }
    }
  }
  std::string err;
  std::string last_driver;
  std::vector<std::string> attempt_errors;
  auto apply_evdev_setting = [&](bool disable_evdev) {
    if (!disable_evdev) return;
    if (!env_linuxev || !*env_linuxev) {
      setenv("SDL_INPUT_LINUXEV", "0", 1);
    }
    if (!env_evdev || !*env_evdev) {
      setenv("SDL_INPUT_EVDEV", "0", 1);
    }
    SDL_SetHint(SDL_HINT_INPUT_LINUXEV, "0");
    SDL_SetHint(SDL_HINT_INPUT_EVDEV, "0");
  };

  auto try_init_with_driver = [&](const char* driver, bool disable_evdev) -> bool {
    apply_evdev_setting(disable_evdev);
    if (driver && *driver) {
      setenv("SDL_VIDEODRIVER", driver, 1);
#ifdef SDL_HINT_VIDEO_DRIVER
      SDL_SetHint(SDL_HINT_VIDEO_DRIVER, driver);
#endif
    } else {
      unsetenv("SDL_VIDEODRIVER");
#ifdef SDL_HINT_VIDEO_DRIVER
      SDL_ResetHint(SDL_HINT_VIDEO_DRIVER);
#endif
    }
    last_driver = (driver && *driver) ? driver : "(default)";
    bool init_ok = false;
    for (int attempt = 0; attempt < 3; ++attempt) {
      SDL_ClearError();
      errno = 0;
      if (SDL_Init(init_flags) == 0) {
        init_ok = true;
        break;
      }
      if (errno == EAGAIN) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50 * (attempt + 1)));
        continue;
      }
      break;
    }
    if (!init_ok) {
      const char* sdl_err = SDL_GetError();
      err = (sdl_err && *sdl_err) ? sdl_err : "unknown error";
      const std::string errno_suffix = format_errno();
      if (!errno_suffix.empty() && err.find("errno=") == std::string::npos) {
        err += errno_suffix;
      }
      attempt_errors.push_back(last_driver + ": " + err);
      SDL_Quit();
      return false;
    }
    SDL_Window* window = nullptr;
    for (int attempt = 0; attempt < 2; ++attempt) {
      SDL_ClearError();
      errno = 0;
      window = SDL_CreateWindow(desc.title, desc.width, desc.height, window_flags);
      if (window) break;
      if (errno == EAGAIN) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        continue;
      }
      break;
    }
    if (!window) {
      const char* sdl_err = SDL_GetError();
      err = (sdl_err && *sdl_err) ? sdl_err : "unknown error";
      const std::string errno_suffix = format_errno();
      if (!errno_suffix.empty() && err.find("errno=") == std::string::npos) {
        err += errno_suffix;
      }
      attempt_errors.push_back(last_driver + ": " + err);
      SDL_Quit();
      return false;
    }
    self->window_ = window;
    self->last_ticks_ = SDL_GetTicks();
    self->quit_ = false;
    return true;
  };

  auto init_events = [&](bool disable_evdev) -> bool {
    if (disable_evdev) {
      setenv("SDL_INPUT_LINUXEV", "0", 1);
      setenv("SDL_INPUT_EVDEV", "0", 1);
      SDL_SetHint(SDL_HINT_INPUT_LINUXEV, "0");
      SDL_SetHint(SDL_HINT_INPUT_EVDEV, "0");
    } else {
      unsetenv("SDL_INPUT_LINUXEV");
      unsetenv("SDL_INPUT_EVDEV");
    }
    if (SDL_InitSubSystem(SDL_INIT_EVENTS) != 0) {
      const char* sdl_err = SDL_GetError();
      rkg::log::warn(std::string("SDL_InitSubSystem(SDL_INIT_EVENTS) failed: ") +
                     ((sdl_err && *sdl_err) ? sdl_err : "unknown error"));
      SDL_ClearError();
      return false;
    }
    return true;
  };

  std::vector<const char*> fallback_drivers;
  bool have_wayland = (wayland_display && *wayland_display && env_xdg_runtime && *env_xdg_runtime);
  if (have_wayland) {
    std::string wl_socket = std::string(env_xdg_runtime) + "/" + wayland_display;
    struct stat st = {};
    if (stat(wl_socket.c_str(), &st) == 0) {
      rkg::log::info(std::string("Wayland socket: ") + wl_socket +
                     " mode=" + format_mode(st.st_mode) +
                     " uid=" + std::to_string(st.st_uid) +
                     " gid=" + std::to_string(st.st_gid));
    } else {
      rkg::log::warn(std::string("Wayland socket stat failed: ") + wl_socket + format_errno());
      have_wayland = false;
    }
    if (have_wayland && access(wl_socket.c_str(), R_OK | W_OK) != 0) {
      have_wayland = false;
      rkg::log::warn(std::string("Wayland socket not accessible: ") + wl_socket + format_errno());
    }
    if (have_wayland) {
      log_unix_socket_connect(wl_socket, "Wayland");
      log_wayland_connect(wayland_display);
    }
  }
  const bool have_x11 = (display && *display);
  if (have_x11) {
    const std::string x_sock = x11_socket_path(display);
    if (!x_sock.empty()) {
      struct stat st = {};
      if (stat(x_sock.c_str(), &st) == 0) {
        rkg::log::info(std::string("X11 socket: ") + x_sock +
                       " mode=" + format_mode(st.st_mode) +
                       " uid=" + std::to_string(st.st_uid) +
                       " gid=" + std::to_string(st.st_gid));
      } else {
        rkg::log::warn(std::string("X11 socket stat failed: ") + x_sock + format_errno());
      }
      if (access(x_sock.c_str(), R_OK | W_OK) != 0) {
        rkg::log::warn(std::string("X11 socket not accessible: ") + x_sock + format_errno());
      }
      log_unix_socket_connect(x_sock, "X11");
      log_x11_connect(display);
    } else {
      rkg::log::warn(std::string("Unrecognized DISPLAY format: ") + display);
    }
  }
  if (have_wayland) {
    if (!have_shared_lib("libwayland-client.so.0")) {
      rkg::log::warn("Wayland display detected but libwayland-client.so.0 is missing.");
    }
    if (!have_shared_lib("libxkbcommon.so.0")) {
      rkg::log::warn("Wayland display detected but libxkbcommon.so.0 is missing.");
    }
    if (!have_shared_lib("libdecor-0.so.0")) {
      rkg::log::warn("Wayland display detected but libdecor-0.so.0 is missing.");
    }
    log_missing_libs("Wayland", {
      "libwayland-client.so.0",
      "libwayland-egl.so.1",
      "libEGL.so.1",
      "libGLESv2.so.2",
      "libdrm.so.2",
      "libgbm.so.1",
      "libxkbcommon.so.0",
      "libdecor-0.so.0"
    });
  }
  if (have_x11) {
    if (!have_shared_lib("libX11.so.6")) {
      rkg::log::warn("X11 display detected but libX11.so.6 is missing.");
    }
    if (!have_shared_lib("libXcursor.so.1")) {
      rkg::log::warn("X11 display detected but libXcursor.so.1 is missing.");
    }
    if (!have_shared_lib("libXrandr.so.2")) {
      rkg::log::warn("X11 display detected but libXrandr.so.2 is missing.");
    }
    if (!have_shared_lib("libXi.so.6")) {
      rkg::log::warn("X11 display detected but libXi.so.6 is missing.");
    }
    if (!have_shared_lib("libXext.so.6")) {
      rkg::log::warn("X11 display detected but libXext.so.6 is missing.");
    }
    if (!have_shared_lib("libXfixes.so.3")) {
      rkg::log::warn("X11 display detected but libXfixes.so.3 is missing.");
    }
    if (!have_shared_lib("libXrender.so.1")) {
      rkg::log::warn("X11 display detected but libXrender.so.1 is missing.");
    }
    if (!have_shared_lib("libxcb.so.1")) {
      rkg::log::warn("X11 display detected but libxcb.so.1 is missing.");
    }
    log_missing_libs("X11", {
      "libX11.so.6",
      "libX11-xcb.so.1",
      "libXcursor.so.1",
      "libXrandr.so.2",
      "libXi.so.6",
      "libXext.so.6",
      "libXfixes.so.3",
      "libXrender.so.1",
      "libxcb.so.1",
      "libxcb-randr.so.0",
      "libxcb-xfixes.so.0",
      "libxcb-shm.so.0",
      "libxcb-render-util.so.0",
      "libxcb-icccm.so.4"
    });
  }
  if (have_wayland) {
    fallback_drivers.push_back("wayland");
  }
  if (have_x11) {
    fallback_drivers.push_back("x11");
  }
  if (!have_wayland && !have_x11) {
    fallback_drivers.push_back("kmsdrm");
#if !RKG_ENABLE_VULKAN
    fallback_drivers.push_back("offscreen");
    fallback_drivers.push_back("dummy");
#endif
  }
  auto attempt_with_evdev = [&](bool disable_evdev) -> bool {
    if (try_init_with_driver(env_driver, disable_evdev)) {
      if (!init_events(disable_evdev)) {
        init_events(true);
      }
      return true;
    }
    for (const char* driver : fallback_drivers) {
      if (env_driver && *env_driver && std::string(env_driver) == driver) {
        continue;
      }
      if (try_init_with_driver(driver, disable_evdev)) {
        rkg::log::warn(std::string("SDL_Init recovered by forcing video driver: ") + driver);
        if (!init_events(disable_evdev)) {
          init_events(true);
        }
        return true;
      }
    }
    return false;
  };

  const bool prefer_disable_evdev = (geteuid() != 0);
  if (prefer_disable_evdev) {
    if (attempt_with_evdev(true) || attempt_with_evdev(false)) {
      return true;
    }
  } else {
    if (attempt_with_evdev(false) || attempt_with_evdev(true)) {
      return true;
    }
  }

  rkg::log::error(std::string("SDL_Init failed: ") + err +
                  (last_driver.empty() ? "" : std::string(" (driver=") + last_driver + ")"));
  if (!attempt_errors.empty()) {
    rkg::log::error("SDL init attempts:");
    for (const auto& entry : attempt_errors) {
      rkg::log::error("  " + entry);
    }
  }
  rkg::log::error(std::string("SDL video drivers: ") + list_video_drivers());
  const char* forced = std::getenv("SDL_VIDEODRIVER");
  rkg::log::error(std::string("Env DISPLAY=") + (display ? display : "") +
                  " WAYLAND_DISPLAY=" + (wayland_display ? wayland_display : "") +
                  " XDG_SESSION_TYPE=" + (session_type ? session_type : "") +
                  " XDG_RUNTIME_DIR=" + (env_xdg_runtime ? env_xdg_runtime : "") +
                  " XAUTHORITY=" + (env_xauthority ? env_xauthority : "") +
                  " SDL_VIDEODRIVER=" + (forced ? forced : "") +
                  " EUID=" + (is_root ? "0" : "user"));
  if (env_xauthority && *env_xauthority) {
    if (access(env_xauthority, R_OK) != 0) {
      rkg::log::warn(std::string("XAUTHORITY not readable: ") + env_xauthority + format_errno());
    }
  }
  return false;
}

void platform_shutdown(Platform* self) {
  if (self->window_) {
    SDL_DestroyWindow(static_cast<SDL_Window*>(self->window_));
    self->window_ = nullptr;
  }
  SDL_Quit();
}

void platform_poll_events(Platform* self) {
  if ((SDL_WasInit(SDL_INIT_EVENTS) & SDL_INIT_EVENTS) == 0) {
    return;
  }
  SDL_Event event;
  while (SDL_PollEvent(&event)) {
    if (self->event_callback_) {
      self->event_callback_(&event, self->event_user_data_);
    }
    if (event.type == SDL_EVENT_QUIT) {
      self->quit_ = true;
    }
    if (event.type == SDL_EVENT_KEY_DOWN) {
      const auto code = map_sdl_scancode(event.key.scancode);
      if (code != KeyCode::Unknown) {
        self->keys_down_.insert(code);
      }
      if (code == KeyCode::Escape) {
        self->quit_ = true;
      }
    }
    if (event.type == SDL_EVENT_KEY_UP) {
      const auto code = map_sdl_scancode(event.key.scancode);
      if (code != KeyCode::Unknown) {
        self->keys_down_.erase(code);
      }
    }
  }
}

bool platform_should_quit(const Platform* self) {
  return self->quit_;
}

float platform_delta_seconds(Platform* self) {
  const auto now = SDL_GetTicks();
  const auto diff = now - self->last_ticks_;
  self->last_ticks_ = now;
  return static_cast<float>(diff) / 1000.0f;
}

void* platform_native_window(const Platform* self) {
  return self->window_;
}

std::string format_errno() {
  const int err = errno;
  if (err == 0) return "";
  return std::string(" errno=") + std::to_string(err) + " (" + std::strerror(err) + ")";
}

} // namespace rkg::platform::detail
