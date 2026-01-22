#include "rkg/replay.h"

#include "rkg/log.h"

#include <nlohmann/json.hpp>

#include <fstream>

namespace rkg::replay {

void ReplayRecorder::begin(const std::filesystem::path& path, float fixed_dt) {
  path_ = path;
  fixed_dt_ = fixed_dt;
  frames_.clear();
  recording_ = true;
}

void ReplayRecorder::record_frame(float dt, const std::unordered_map<std::string, input::ActionState>& actions) {
  if (!recording_) return;
  RecordedFrame frame;
  frame.dt = dt;
  for (const auto& kv : actions) {
    frame.actions[kv.first] = {kv.second.pressed, kv.second.held, kv.second.released};
  }
  frames_.push_back(std::move(frame));
}

void ReplayRecorder::end() {
  if (!recording_) return;
  nlohmann::json doc;
  doc["format"] = "rkg_replay_v1";
  doc["fixed_dt"] = fixed_dt_;
  doc["frames"] = nlohmann::json::array();
  for (const auto& frame : frames_) {
    nlohmann::json f;
    f["dt"] = frame.dt;
    f["actions"] = nlohmann::json::object();
    for (const auto& akv : frame.actions) {
      nlohmann::json a;
      a["pressed"] = akv.second.pressed;
      a["held"] = akv.second.held;
      a["released"] = akv.second.released;
      f["actions"][akv.first] = a;
    }
    doc["frames"].push_back(f);
  }
  std::filesystem::create_directories(path_.parent_path());
  std::ofstream out(path_);
  out << doc.dump(2);
  recording_ = false;
  rkg::log::info("replay recorded");
}

bool ReplayPlayer::load(const std::filesystem::path& path) {
  std::ifstream in(path);
  if (!in) {
    rkg::log::warn("replay file not found");
    return false;
  }
  nlohmann::json doc;
  in >> doc;
  if (!doc.contains("frames") || !doc["frames"].is_array()) {
    rkg::log::warn("replay file invalid");
    return false;
  }
  frames_.clear();
  fixed_dt_ = doc.value("fixed_dt", 0.0f);
  for (const auto& f : doc["frames"]) {
    RecordedFrame frame;
    frame.dt = f.value("dt", 0.0f);
    if (f.contains("actions") && f["actions"].is_object()) {
      for (auto it = f["actions"].begin(); it != f["actions"].end(); ++it) {
        RecordedAction action;
        action.pressed = it.value().value("pressed", false);
        action.held = it.value().value("held", false);
        action.released = it.value().value("released", false);
        frame.actions[it.key()] = action;
      }
    }
    frames_.push_back(std::move(frame));
  }
  index_ = 0;
  return true;
}

bool ReplayPlayer::next_frame(RecordedFrame& out) {
  if (index_ >= frames_.size()) {
    return false;
  }
  out = frames_[index_++];
  return true;
}

void ReplayPlayer::reset() {
  index_ = 0;
}

} // namespace rkg::replay
