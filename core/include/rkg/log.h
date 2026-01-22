#pragma once

#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace rkg::log {

void init();
void init(const std::string& app_name, const std::filesystem::path& root);
void shutdown();
void install_crash_handlers();

void info(std::string_view msg);
void warn(std::string_view msg);
void error(std::string_view msg);

std::vector<std::string> recent(size_t max_entries = 200);

} // namespace rkg::log
