#include "rkg_ai/ai.h"

#include "rkg/log.h"

#include <fstream>
#include <sstream>

namespace rkg::ai {

namespace {
std::string read_file(const std::string& path) {
  std::ifstream in(path);
  if (!in) {
    rkg::log::warn(std::string("stub AI file not found: ") + path);
    return {};
  }
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}
} // namespace

LlmResponse StubLlmClient::submit(const LlmRequest& request) {
  (void)request;
  LlmResponse response;
  response.json = read_file(sample_path_);
  return response;
}

std::string StubAgentOrchestrator::generate_dev_task_graph(const std::string& prompt) {
  (void)prompt;
  return read_file(dev_plan_path_);
}

std::string StubAgentOrchestrator::generate_runtime_instructions(const std::string& prompt) {
  (void)prompt;
  return read_file(runtime_path_);
}

} // namespace rkg::ai
