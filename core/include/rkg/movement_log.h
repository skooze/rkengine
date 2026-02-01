#pragma once

#include <filesystem>
#include <string_view>

namespace rkg::movement_log {

// Opens (truncates) the movement log file.
void open(const std::filesystem::path& path);
// Closes the log file if open.
void close();
// Returns true if movement logging is active.
bool enabled();
// Appends a single line to the movement log (adds trailing newline).
void write(std::string_view line);

} // namespace rkg::movement_log
