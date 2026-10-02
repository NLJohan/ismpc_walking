#pragma once

#include "ismpc_walking/policy/PolicyContract.h"

#include <cstddef>
#include <vector>

namespace ismpc_walking::policy
{

/**
 * Diagnostics only: how many observation elements are far from the training distribution,
 * z = (x - mean) / (std + eps) with the contract's own normalizer statistics. This is the formula of rsl_rl's
 * EmpiricalNormalization (default eps 1e-2), i.e. what the network sees first.
 *
 * Pure: no logging, no controller. Preallocated at construction, update() does not allocate.
 */
class ZScoreMonitor
{
public:
  static constexpr double kWarn = 6.0; // |z| above this counts as an offender

  /** Copies what it needs: keeps no reference to the contract. */
  explicit ZScoreMonitor(const PolicyContract & contract);

  /** False when the contract has no normalizer: update() then does nothing. */
  bool available() const noexcept { return available_; }

  /** Looks at one observation (size obsDim, in doubles). Ignores a vector of the wrong size. */
  void update(const std::vector<double> & obs) noexcept;

  /** Number of elements of the LAST update() with |z| > kWarn. */
  size_t offenderCount() const noexcept { return offenders_; }

private:
  bool available_ = false;
  double eps_ = 0;
  std::vector<double> mean_, std_;
  size_t offenders_ = 0;
};

} // namespace ismpc_walking::policy
