#pragma once

#include <any>
#include <functional>
#include <typeindex>
#include <unordered_map>
#include <vector>

namespace rkg {

class EventBus {
 public:
  template <typename T>
  void subscribe(std::function<void(const T&)> handler) {
    auto& bucket = handlers_[std::type_index(typeid(T))];
    bucket.push_back([handler](const std::any& ev) {
      handler(std::any_cast<const T&>(ev));
    });
  }

  template <typename T>
  void emit(const T& event) {
    auto it = handlers_.find(std::type_index(typeid(T)));
    if (it == handlers_.end()) {
      return;
    }
    const std::any wrapped(event);
    for (auto& fn : it->second) {
      fn(wrapped);
    }
  }

 private:
  std::unordered_map<std::type_index, std::vector<std::function<void(const std::any&)>>> handlers_;
};

} // namespace rkg
