#pragma once

#include "rkg/asset_types.h"

#include <filesystem>

namespace rkg::asset {

ImportResult import_glb(const std::filesystem::path& input_path,
                        const std::filesystem::path& output_dir,
                        const ImportOptions& options);

}  // namespace rkg::asset
