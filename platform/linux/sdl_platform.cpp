#include "rkg_platform/platform.h"

#include "rkg/log.h"

#include <SDL3/SDL.h>
#include <cstdlib>
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
  const char* env_driver = SDL_getenv("SDL_VIDEODRIVER");
  const char* env_linuxev = std::getenv("SDL_INPUT_LINUXEV");
  const char* env_evdev = std::getenv("SDL_INPUT_EVDEV");
  const char* env_xdg_runtime = std::getenv("XDG_RUNTIME_DIR");
  std::string err;
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
    if (SDL_Init(init_flags) != 0) {
      const char* sdl_err = SDL_GetError();
      err = (sdl_err && *sdl_err) ? sdl_err : "unknown error";
      SDL_Quit();
      return false;
    }
    SDL_Window* window = SDL_CreateWindow(desc.title, desc.width, desc.height, window_flags);
    if (!window) {
      const char* sdl_err = SDL_GetError();
      err = (sdl_err && *sdl_err) ? sdl_err : "unknown error";
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

  if (try_init_with_driver(env_driver, false)) {
    if (!init_events(false)) {
      init_events(true);
    }
    return true;
  }

  const std::vector<const char*> fallback_drivers = {
      "wayland",
      "x11",
      "kmsdrm",
      "offscreen",
      "dummy"};
  for (const char* driver : fallback_drivers) {
    if (env_driver && *env_driver && std::string(env_driver) == driver) {
      continue;
    }
    if (try_init_with_driver(driver, false)) {
      rkg::log::warn(std::string("SDL_Init failed, recovered by forcing video driver: ") + driver);
      if (!init_events(false)) {
        init_events(true);
      }
      return true;
    }
  }

  if (try_init_with_driver(env_driver, true)) {
    rkg::log::warn("SDL_Init recovered by disabling evdev input");
    init_events(true);
    return true;
  }

  for (const char* driver : fallback_drivers) {
    if (env_driver && *env_driver && std::string(env_driver) == driver) {
      continue;
    }
    if (try_init_with_driver(driver, true)) {
      rkg::log::warn(std::string("SDL_Init recovered by disabling evdev input and forcing video driver: ") + driver);
      init_events(true);
      return true;
    }
  }

  rkg::log::error(std::string("SDL_Init failed: ") + err);
  rkg::log::error(std::string("SDL video drivers: ") + list_video_drivers());
  const char* display = SDL_getenv("DISPLAY");
  const char* wayland = SDL_getenv("WAYLAND_DISPLAY");
  const char* session = SDL_getenv("XDG_SESSION_TYPE");
  const char* forced = SDL_getenv("SDL_VIDEODRIVER");
  rkg::log::error(std::string("Env DISPLAY=") + (display ? display : "") +
                  " WAYLAND_DISPLAY=" + (wayland ? wayland : "") +
                  " XDG_SESSION_TYPE=" + (session ? session : "") +
                  " XDG_RUNTIME_DIR=" + (env_xdg_runtime ? env_xdg_runtime : "") +
                  " SDL_VIDEODRIVER=" + (forced ? forced : ""));
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
