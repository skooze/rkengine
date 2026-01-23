#include "rkg_platform/platform.h"

#include "rkg/log.h"

#include <SDL3/SDL.h>
#include <cstdlib>
#include <pwd.h>
#include <dlfcn.h>
#include <sys/stat.h>
#include <unistd.h>
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

bool have_shared_lib(const char* name) {
  if (!name || !*name) return false;
  void* handle = dlopen(name, RTLD_LAZY | RTLD_LOCAL);
  if (!handle) {
    return false;
  }
  dlclose(handle);
  return true;
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
    } else {
      unsetenv("SDL_VIDEODRIVER");
    }
    last_driver = (driver && *driver) ? driver : "(default)";
    SDL_ClearError();
    if (SDL_Init(init_flags) != 0) {
      const char* sdl_err = SDL_GetError();
      err = (sdl_err && *sdl_err) ? sdl_err : "unknown error";
      attempt_errors.push_back(last_driver + ": " + err);
      SDL_Quit();
      return false;
    }
    SDL_Window* window = SDL_CreateWindow(desc.title, desc.width, desc.height, window_flags);
    if (!window) {
      const char* sdl_err = SDL_GetError();
      err = (sdl_err && *sdl_err) ? sdl_err : "unknown error";
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
    if (access(wl_socket.c_str(), R_OK | W_OK) != 0) {
      have_wayland = false;
    }
  }
  const bool have_x11 = (display && *display);
  if (have_wayland) {
    if (!have_shared_lib("libwayland-client.so.0")) {
      rkg::log::warn("Wayland display detected but libwayland-client.so.0 is missing.");
    }
  }
  if (have_x11) {
    if (!have_shared_lib("libX11.so.6")) {
      rkg::log::warn("X11 display detected but libX11.so.6 is missing.");
    }
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

} // namespace rkg::platform::detail
