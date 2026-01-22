#include "rkg/runtime_host.h"

#include "rkg/log.h"
#include "rkg/replay.h"

#include <filesystem>
#include <optional>
#include <string>

namespace fs = std::filesystem;

int main(int argc, char** argv) {
  std::optional<fs::path> record_path;
  std::optional<fs::path> replay_path;
  float fixed_dt = 0.0f;
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--record" && i + 1 < argc) {
      record_path = fs::path(argv[++i]);
    } else if (arg == "--replay" && i + 1 < argc) {
      replay_path = fs::path(argv[++i]);
    } else if (arg == "--fixed-dt" && i + 1 < argc) {
      fixed_dt = std::stof(argv[++i]);
    }
  }

  rkg::runtime::RuntimeHost runtime;
  rkg::runtime::RuntimeHostInit init;
  init.argv0 = argc > 0 ? argv[0] : nullptr;
  init.default_project = "demo_game";
  init.app_name = "rkg_demo_game";
  init.window = {1280, 720, "rkg_demo_game"};

  std::string error;
  if (!runtime.init(init, error)) {
    rkg::log::error(error.empty() ? "rkg_demo_game init failed" : error);
    return 1;
  }

  rkg::replay::ReplayRecorder recorder;
  rkg::replay::ReplayPlayer player;
  bool replay_active = false;
  if (replay_path.has_value()) {
    replay_active = player.load(replay_path.value());
    if (replay_active && player.fixed_dt() > 0.0f) {
      fixed_dt = player.fixed_dt();
    }
  }
  if (record_path.has_value() && !replay_active) {
    recorder.begin(record_path.value(), fixed_dt);
  }

  while (!runtime.platform().should_quit()) {
    runtime.platform().poll_events();
    float dt = runtime.platform().delta_seconds();
    if (fixed_dt > 0.0f) {
      dt = fixed_dt;
    }

    rkg::replay::RecordedFrame replay_frame;
    if (replay_active) {
      if (!player.next_frame(replay_frame)) {
        runtime.platform().request_quit();
      }
    } else {
      runtime.update_input();
      if (recorder.is_recording()) {
        recorder.record_frame(dt, runtime.input().states());
      }
    }

    auto action_state = [&](const std::string& name) -> rkg::input::ActionState {
      if (replay_active) {
        auto it = replay_frame.actions.find(name);
        if (it != replay_frame.actions.end()) {
          return {it->second.pressed, it->second.held, it->second.released};
        }
        return {};
      }
      return runtime.input_action(name);
    };

    rkg::runtime::FrameParams params;
    params.frame_dt = dt;
    params.sim_dt = dt;
    params.run_simulation = true;
    params.update_input = false;
    runtime.tick(params, action_state);
  }

  if (recorder.is_recording()) {
    recorder.end();
  }

  runtime.shutdown();
  return 0;
}
