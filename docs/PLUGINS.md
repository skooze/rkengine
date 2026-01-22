# Plugins

RKG treats everything beyond the core as a plugin. Plugins are **statically linked today** but have a **dynamic-ready API**.

## Plugin API
Each plugin exposes an API table:
```
struct RkgPluginApi {
  uint32_t api_version;
  const char* name;
  PluginType type;
  uint32_t caps;
  bool (*init)(void* host_context);
  void (*shutdown)();
  void (*update)(float dt_seconds);
  void (*on_window_resized)(int width, int height);
};
```

Dynamic loading (future) will use:
```
extern "C" RkgPluginApi* rkg_plugin_get_api(uint32_t host_api_version);
```

For static linking, each plugin exports a **unique** entrypoint (e.g., `rkg_plugin_get_api_renderer_null`) to avoid collisions.

## Adding a Plugin (Checklist)
1) Create a folder under `plugins/<category>/<name>/`.
2) Implement `RkgPluginApi` and a unique entrypoint.
3) Add a `CMakeLists.txt` to build a library target.
4) Register the plugin in the app (or future loader).

## Enabling via Config
`config/engine.yaml` selects plugins, for example:
```
engine:
  renderer: vulkan
  scripting: [lua, js]
```

Projects can also enable plugins in `project.yaml`, e.g.:
```
project:
  plugins:
    - debug_ui_imgui
```
`debug_ui_imgui` requires Vulkan and SDL3. The renderer exposes caps such as `DebugUiSupported` so apps can
enable/disable the overlay gracefully.

## Versioning
`kRkgPluginApiVersion` is defined in `core/include/rkg/plugin_api.h`.
Plugins should check the host version and return `nullptr` if incompatible.
