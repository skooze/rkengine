#pragma once

#include "rkg/input.h"

#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

namespace rkg::replay {

struct RecordedAction {
  bool pressed = false;
  bool held = false;
  bool released = false;
};

struct RecordedFrame {
  float dt = 0.0f;
  std::unordered_map<std::string, RecordedAction> actions;
};

class ReplayRecorder {
 public:
  void begin(const std::filesystem::path& path, float fixed_dt = 0.0f);
  void record_frame(float dt, const std::unordered_map<std::string, input::ActionState>& actions);
  void end();
  bool is_recording() const { return recording_; }

 private:
  std::filesystem::path path_;
  std::vector<RecordedFrame> frames_;
  float fixed_dt_ = 0.0f;
  bool recording_ = false;
};

class ReplayPlayer {
 public:
  bool load(const std::filesystem::path& path);
  bool next_frame(RecordedFrame& out);
  void reset();
  float fixed_dt() const { return fixed_dt_; }

 private:
  std::vector<RecordedFrame> frames_;
  size_t index_ = 0;
  float fixed_dt_ = 0.0f;
};

} // namespace rkg::replay
