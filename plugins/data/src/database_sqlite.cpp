#include "rkg_data/database.h"

#include "rkg/log.h"

#if RKG_HAVE_SQLITE3
#include <sqlite3.h>
#endif

namespace rkg::data {

bool SqliteDatabase::open(const std::string& path) {
#if RKG_HAVE_SQLITE3
  if (sqlite3_open(path.c_str(), &db_) != SQLITE_OK) {
    rkg::log::warn("sqlite open failed");
    return false;
  }
  return true;
#else
  rkg::log::warn("sqlite not available");
  (void)path;
  return false;
#endif
}

bool SqliteDatabase::exec(const std::string& sql) {
#if RKG_HAVE_SQLITE3
  char* err_msg = nullptr;
  const int rc = sqlite3_exec(db_, sql.c_str(), nullptr, nullptr, &err_msg);
  if (rc != SQLITE_OK) {
    if (err_msg) {
      rkg::log::warn(std::string("sqlite exec error: ") + err_msg);
      sqlite3_free(err_msg);
    }
    return false;
  }
  return true;
#else
  (void)sql;
  return false;
#endif
}

void SqliteDatabase::close() {
#if RKG_HAVE_SQLITE3
  if (db_) {
    sqlite3_close(db_);
    db_ = nullptr;
  }
#endif
}

SqliteDatabase::~SqliteDatabase() {
  close();
}

} // namespace rkg::data
