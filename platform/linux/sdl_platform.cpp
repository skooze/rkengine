#include "rkg_platform/platform.h"

#include "rkg/log.h"

#include <SDL3/SDL.h>
#include <string>

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

bool platform_init(Platform* self, const WindowDesc& desc) {
  if (SDL_Init(SDL_INIT_VIDEO) != 0) {
    rkg::log::error(std::string("SDL_Init failed: ") + SDL_GetError());
    return false;
  }

  Uint32 flags = SDL_WINDOW_RESIZABLE;
#if RKG_ENABLE_VULKAN
  flags |= SDL_WINDOW_VULKAN;
#endif
  SDL_Window* window = SDL_CreateWindow(desc.title, desc.width, desc.height, flags);
  if (!window) {
    rkg::log::error(std::string("SDL_CreateWindow failed: ") + SDL_GetError());
    SDL_Quit();
    return false;
  }

  self->window_ = window;
  self->last_ticks_ = SDL_GetTicks();
  self->quit_ = false;
  return true;
}

void platform_shutdown(Platform* self) {
  if (self->window_) {
    SDL_DestroyWindow(static_cast<SDL_Window*>(self->window_));
    self->window_ = nullptr;
  }
  SDL_Quit();
}

void platform_poll_events(Platform* self) {
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
