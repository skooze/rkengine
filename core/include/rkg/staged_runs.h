#pragma once

#include <filesystem>
#include <optional>
#include <string>

namespace rkg {

std::optional<std::filesystem::path> find_latest_staged_run_dir(const std::filesystem::path& root,
                                                                std::string* error);

} // namespace rkg
