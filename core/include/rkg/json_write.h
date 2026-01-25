#pragma once

#include <cstdint>
#include <ostream>
#include <string_view>

namespace rkg::json {

class Writer {
public:
  explicit Writer(std::ostream& out);

  void begin_object();
  void end_object();
  void begin_array();
  void end_array();

  void key(std::string_view name);

  void value(std::string_view text);
  void value(double number);
  void value(int64_t number);
  void value(uint64_t number);
  void value(bool boolean);
  void null();

  void finish();

private:
  struct Context {
    bool first = true;
    bool is_object = false;
    bool expect_value = false;
  };

  void pre_value();
  void write_string(std::string_view text);

  std::ostream& out_;
  Context stack_[64]{};
  int depth_ = 0;
};

}  // namespace rkg::json
