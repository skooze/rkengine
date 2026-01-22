#pragma once

#include <string>

namespace rkg::ai {

struct LlmRequest {
  std::string system_prompt;
  std::string user_prompt;
};

struct LlmResponse {
  std::string json;
};

class ILLMClient {
 public:
  virtual ~ILLMClient() = default;
  virtual LlmResponse submit(const LlmRequest& request) = 0;
};

class IAgentOrchestrator {
 public:
  virtual ~IAgentOrchestrator() = default;
  virtual std::string generate_dev_task_graph(const std::string& prompt) = 0;
  virtual std::string generate_runtime_instructions(const std::string& prompt) = 0;
};

class StubLlmClient final : public ILLMClient {
 public:
  explicit StubLlmClient(std::string sample_path) : sample_path_(std::move(sample_path)) {}
  LlmResponse submit(const LlmRequest& request) override;

 private:
  std::string sample_path_;
};

class StubAgentOrchestrator final : public IAgentOrchestrator {
 public:
  explicit StubAgentOrchestrator(std::string dev_plan_path, std::string runtime_path)
      : dev_plan_path_(std::move(dev_plan_path)), runtime_path_(std::move(runtime_path)) {}

  std::string generate_dev_task_graph(const std::string& prompt) override;
  std::string generate_runtime_instructions(const std::string& prompt) override;

 private:
  std::string dev_plan_path_;
  std::string runtime_path_;
};

} // namespace rkg::ai
