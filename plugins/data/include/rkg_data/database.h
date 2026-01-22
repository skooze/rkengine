#pragma once

#include <string>

namespace rkg::data {

struct sqlite3;

class IDatabase {
 public:
  virtual ~IDatabase() = default;
  virtual bool open(const std::string& path) = 0;
  virtual bool exec(const std::string& sql) = 0;
};

class SqliteDatabase final : public IDatabase {
 public:
  bool open(const std::string& path) override;
  bool exec(const std::string& sql) override;
  void close();
  ~SqliteDatabase() override;

 private:
  struct sqlite3* db_ = nullptr;
};

} // namespace rkg::data
