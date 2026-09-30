#pragma once

#include <string>

namespace ismpc_walking::policy
{

/** Something that maps one observation vector to one raw action vector. */
class PolicyBackend
{
public:
  virtual ~PolicyBackend() = default;

  virtual int obsDim() const noexcept = 0;
  virtual int actionDim() const noexcept = 0;

  /** The "ismpc_contract" metadata string stored in the model file. */
  virtual const std::string & contractJson() const noexcept = 0;

  /**
   * obs: obsDim() floats, action: actionDim() floats. No allocation.
   * Never throws: returns false and fills `err` on failure.
   */
  virtual bool infer(const float * obs, float * action, std::string & err) noexcept = 0;
};

} // namespace ismpc_walking::policy
