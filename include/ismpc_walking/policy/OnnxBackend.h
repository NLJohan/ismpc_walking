#pragma once

#include "ismpc_walking/policy/PolicyBackend.h"

#include <memory>
#include <string>

namespace ismpc_walking::policy
{

/**
 * ONNX Runtime implementation (CPU, single thread, sequential, no spinning).
 * This header does not include any ONNX Runtime header. In a build without
 * ONNX Runtime, load() simply fails with an explanatory message.
 */
class OnnxBackend final : public PolicyBackend
{
public:
  /**
   * Loads the model, reads the "ismpc_contract" metadata and checks the tensor
   * signature (one float input [1, N], one float output [1, M]).
   * Session is created here and nowhere else. Never throws; returns nullptr and fills `err`.
   */
  static std::unique_ptr<OnnxBackend> load(const std::string & path, std::string & err) noexcept;

  ~OnnxBackend() override;

  int obsDim() const noexcept override;
  int actionDim() const noexcept override;
  const std::string & contractJson() const noexcept override;
  bool infer(const float * obs, float * action, std::string & err) noexcept override;

private:
  OnnxBackend();
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace ismpc_walking::policy
