#pragma once

#include <filesystem>
#include <string_view>

namespace rkg::ui_log {

// Opens (truncates) the UI log file.
void open(const std::filesystem::path& path);
// Closes the log file if open.
void close();
// Returns true if UI logging is active.
bool enabled();
// Appends a single line to the UI log (adds trailing newline).
void write(std::string_view line);

} // namespace rkg::ui_log
