#pragma once

#include <string>

struct sqlite3;

namespace rkg::data {

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
  ::sqlite3* db_ = nullptr;
};

} // namespace rkg::data
