#include "rkg/json_write.h"

#include <charconv>
#include <cstdint>
#include <iomanip>
#include <limits>

namespace rkg::json {

Writer::Writer(std::ostream& out) : out_(out) {}

void Writer::pre_value() {
  if (depth_ <= 0) return;
  auto& ctx = stack_[depth_ - 1];
  if (ctx.is_object) {
    if (ctx.expect_value) {
      ctx.expect_value = false;
      return;
    }
    // If a value is written without a key, treat it as a new entry.
    if (!ctx.first) {
      out_ << ",";
    }
    ctx.first = false;
  } else {
    if (!ctx.first) {
      out_ << ",";
    }
    ctx.first = false;
  }
}

void Writer::begin_object() {
  pre_value();
  out_ << "{";
  stack_[depth_++] = Context{true, true, false};
}

void Writer::end_object() {
  out_ << "}";
  if (depth_ > 0) {
    --depth_;
  }
}

void Writer::begin_array() {
  pre_value();
  out_ << "[";
  stack_[depth_++] = Context{true, false, false};
}

void Writer::end_array() {
  out_ << "]";
  if (depth_ > 0) {
    --depth_;
  }
}

void Writer::key(std::string_view name) {
  if (depth_ <= 0) return;
  auto& ctx = stack_[depth_ - 1];
  if (!ctx.is_object) return;
  if (!ctx.first) {
    out_ << ",";
  }
  ctx.first = false;
  write_string(name);
  out_ << ":";
  ctx.expect_value = true;
}

void Writer::write_string(std::string_view text) {
  out_ << "\"";
  for (unsigned char c : text) {
    switch (c) {
      case '\"': out_ << "\\\""; break;
      case '\\': out_ << "\\\\"; break;
      case '\b': out_ << "\\b"; break;
      case '\f': out_ << "\\f"; break;
      case '\n': out_ << "\\n"; break;
      case '\r': out_ << "\\r"; break;
      case '\t': out_ << "\\t"; break;
      default:
        if (c < 0x20) {
          static const char kHex[] = "0123456789ABCDEF";
          out_ << "\\u00" << kHex[(c >> 4) & 0xF] << kHex[c & 0xF];
        } else {
          out_ << static_cast<char>(c);
        }
        break;
    }
  }
  out_ << "\"";
}

void Writer::value(std::string_view text) {
  pre_value();
  write_string(text);
}

void Writer::value(double number) {
  pre_value();
  char buffer[64];
  auto result = std::to_chars(buffer, buffer + sizeof(buffer), number,
                              std::chars_format::general,
                              std::numeric_limits<double>::max_digits10);
  if (result.ec == std::errc()) {
    out_ << std::string_view(buffer, static_cast<size_t>(result.ptr - buffer));
  } else {
    out_ << std::setprecision(std::numeric_limits<double>::max_digits10) << number;
  }
}

void Writer::value(int64_t number) {
  pre_value();
  out_ << number;
}

void Writer::value(uint64_t number) {
  pre_value();
  out_ << number;
}

void Writer::value(bool boolean) {
  pre_value();
  out_ << (boolean ? "true" : "false");
}

void Writer::null() {
  pre_value();
  out_ << "null";
}

void Writer::finish() {
  out_ << "\n";
}

}  // namespace rkg::json
