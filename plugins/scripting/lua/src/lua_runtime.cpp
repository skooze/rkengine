#include "rkg/log.h"
#include "rkg/plugin_api.h"
#include "rkg_script/script_runtime.h"
#include "rkg/ecs.h"

#if RKG_HAVE_LUA
#include <lua.h>
#include <lauxlib.h>
#include <lualib.h>
#endif

#include <filesystem>
#include <vector>

namespace {

class LuaRuntime final : public rkg::script::IScriptRuntime {
 public:
  bool init(rkg::HostContext* host) override {
    host_ = host;
#if RKG_HAVE_LUA
    state_ = luaL_newstate();
    luaL_openlibs(state_);
    register_bindings();
    rkg::log::info("script:lua init");
    return true;
#else
    rkg::log::warn("script:lua not available (Lua not found)");
    return false;
#endif
  }

  bool load_script(const std::string& source_or_path) override {
#if RKG_HAVE_LUA
    if (!state_) return false;
    if (std::filesystem::exists(source_or_path)) {
      return luaL_dofile(state_, source_or_path.c_str()) == 0;
    }
    return luaL_dostring(state_, source_or_path.c_str()) == 0;
#else
    (void)source_or_path;
    return false;
#endif
  }

  void tick(float dt_seconds) override {
#if RKG_HAVE_LUA
    if (!state_) return;
    lua_getglobal(state_, "tick");
    if (lua_isfunction(state_, -1)) {
      lua_pushnumber(state_, dt_seconds);
      if (lua_pcall(state_, 1, 0, 0) != 0) {
        rkg::log::warn("script:lua tick failed");
      }
    } else {
      lua_pop(state_, 1);
    }
#else
    (void)dt_seconds;
#endif
  }

  void call(const std::string& fn, const std::vector<std::string>& args) override {
#if RKG_HAVE_LUA
    if (!state_) return;
    lua_getglobal(state_, fn.c_str());
    if (!lua_isfunction(state_, -1)) {
      lua_pop(state_, 1);
      return;
    }
    for (const auto& arg : args) {
      lua_pushstring(state_, arg.c_str());
    }
    if (lua_pcall(state_, static_cast<int>(args.size()), 0, 0) != 0) {
      rkg::log::warn("script:lua call failed");
    }
#else
    (void)fn;
    (void)args;
#endif
  }

  void bind_native(const rkg::script::NativeBinding& binding) override {
#if RKG_HAVE_LUA
    if (!state_ || !binding.fn) return;
    native_bindings_.push_back(binding);
    register_bindings();
#else
    (void)binding;
#endif
  }

  ~LuaRuntime() override {
#if RKG_HAVE_LUA
    if (state_) {
      lua_close(state_);
      state_ = nullptr;
    }
#endif
  }

 private:
#if RKG_HAVE_LUA
  static int lua_log(lua_State* L) {
    const char* msg = lua_tostring(L, 1);
    if (msg) rkg::log::info(msg);
    return 0;
  }

  static int lua_move_entity(lua_State* L) {
    auto* host = static_cast<rkg::HostContext*>(lua_touserdata(L, lua_upvalueindex(1)));
    if (!host || !host->registry) return 0;
    const auto entity = static_cast<rkg::ecs::Entity>(luaL_checkinteger(L, 1));
    const float dx = static_cast<float>(luaL_checknumber(L, 2));
    const float dy = static_cast<float>(luaL_checknumber(L, 3));
    const float dz = static_cast<float>(luaL_checknumber(L, 4));
    auto* transform = host->registry->get_transform(entity);
    if (transform) {
      transform->position[0] += dx;
      transform->position[1] += dy;
      transform->position[2] += dz;
    }
    return 0;
  }

  void register_bindings() {
    if (!state_) return;
    lua_pushcfunction(state_, &lua_log);
    lua_setglobal(state_, "rkg_log");

    lua_pushlightuserdata(state_, host_);
    lua_pushcclosure(state_, &lua_move_entity, 1);
    lua_setglobal(state_, "rkg_move_entity");

    for (const auto& binding : native_bindings_) {
      lua_pushlightuserdata(state_, reinterpret_cast<void*>(binding.fn));
      lua_pushlightuserdata(state_, binding.user_data);
      lua_pushcclosure(
          state_,
          [](lua_State* L) -> int {
            auto* fn = reinterpret_cast<rkg::script::NativeFn>(lua_touserdata(L, lua_upvalueindex(1)));
            auto* user = lua_touserdata(L, lua_upvalueindex(2));
            if (fn) fn(user);
            return 0;
          },
          2);
      lua_setglobal(state_, binding.name.c_str());
    }
  }

  lua_State* state_ = nullptr;
#endif

  rkg::HostContext* host_ = nullptr;
  std::vector<rkg::script::NativeBinding> native_bindings_;
};

LuaRuntime g_runtime;

bool lua_plugin_init(void* host) {
  return g_runtime.init(static_cast<rkg::HostContext*>(host));
}

void lua_plugin_shutdown() {}

void lua_plugin_update(float dt) {
  g_runtime.tick(dt);
}

rkg::RkgPluginApi g_api{
    rkg::kRkgPluginApiVersion,
    "script_lua",
    rkg::PluginType::Scripting,
    0,
    &lua_plugin_init,
    &lua_plugin_shutdown,
    &lua_plugin_update,
    nullptr};
} // namespace

extern "C" rkg::RkgPluginApi* rkg_plugin_get_api_script_lua(uint32_t host_api_version) {
  if (host_api_version != rkg::kRkgPluginApiVersion) {
    return nullptr;
  }
  return &g_api;
}

#if defined(RKG_BUILD_DYNAMIC_PLUGIN)
extern "C" rkg::RkgPluginApi* rkg_plugin_get_api(uint32_t host_api_version) {
  return rkg_plugin_get_api_script_lua(host_api_version);
}
#endif
