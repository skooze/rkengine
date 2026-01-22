#include "rkg/content_pack.h"

#include <fstream>

namespace rkg::content {

namespace {
uint16_t read_u16(std::ifstream& in) {
  uint16_t v = 0;
  v |= static_cast<uint16_t>(in.get() & 0xFF);
  v |= static_cast<uint16_t>((in.get() & 0xFF) << 8);
  return v;
}

uint32_t read_u32(std::ifstream& in) {
  uint32_t v = 0;
  v |= static_cast<uint32_t>(in.get() & 0xFF);
  v |= static_cast<uint32_t>((in.get() & 0xFF) << 8);
  v |= static_cast<uint32_t>((in.get() & 0xFF) << 16);
  v |= static_cast<uint32_t>((in.get() & 0xFF) << 24);
  return v;
}

uint64_t read_u64(std::ifstream& in) {
  uint64_t v = 0;
  v |= static_cast<uint64_t>(in.get() & 0xFF);
  v |= static_cast<uint64_t>(in.get() & 0xFF) << 8;
  v |= static_cast<uint64_t>(in.get() & 0xFF) << 16;
  v |= static_cast<uint64_t>(in.get() & 0xFF) << 24;
  v |= static_cast<uint64_t>(in.get() & 0xFF) << 32;
  v |= static_cast<uint64_t>(in.get() & 0xFF) << 40;
  v |= static_cast<uint64_t>(in.get() & 0xFF) << 48;
  v |= static_cast<uint64_t>(in.get() & 0xFF) << 56;
  return v;
}
} // namespace

bool PackReader::load(const std::filesystem::path& path, std::string& error) {
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    error = "pack open failed";
    return false;
  }
  char magic[4] = {};
  in.read(magic, 4);
  if (std::string(magic, 4) != "RKGP") {
    error = "pack magic mismatch";
    return false;
  }
  const uint32_t version = read_u32(in);
  if (version != 1) {
    error = "unsupported pack version";
    return false;
  }
  const uint32_t count = read_u32(in);
  const uint64_t index_offset = read_u64(in);
  in.seekg(static_cast<std::streamoff>(index_offset), std::ios::beg);

  entries_.clear();
  index_by_path_.clear();
  for (uint32_t i = 0; i < count; ++i) {
    PackEntry entry;
    entry.id = read_u64(in);
    entry.type = read_u32(in);
    entry.offset = read_u64(in);
    entry.size = read_u64(in);
    const uint16_t path_len = read_u16(in);
    if (path_len > 0) {
      std::string p(path_len, '\0');
      in.read(p.data(), path_len);
      entry.path = p;
    }
    index_by_path_[entry.path] = entries_.size();
    entries_.push_back(std::move(entry));
  }
  path_ = path;
  return true;
}

const PackEntry* PackReader::find_by_path(const std::string& path) const {
  auto it = index_by_path_.find(path);
  if (it == index_by_path_.end()) return nullptr;
  return &entries_[it->second];
}

bool PackReader::read_entry_data(const PackEntry& entry, std::string& out) const {
  std::ifstream in(path_, std::ios::binary);
  if (!in) return false;
  in.seekg(static_cast<std::streamoff>(entry.offset), std::ios::beg);
  out.resize(static_cast<size_t>(entry.size));
  in.read(out.data(), static_cast<std::streamsize>(entry.size));
  return true;
}

} // namespace rkg::content
