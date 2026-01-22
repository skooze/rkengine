#pragma once

#include "rkg/ecs.h"

#include <filesystem>
#include <string>
#include <unordered_map>

namespace rkg {

struct InstructionContext {
  ecs::Registry* registry = nullptr;
  std::unordered_map<std::string, ecs::Entity>* entities_by_name = nullptr;
};

bool apply_runtime_instructions(const std::filesystem::path& path, InstructionContext& ctx);

} // namespace rkg
