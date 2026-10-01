#pragma once

#include "ismpc_walking/policy/ObservationBuilder.h"
#include "ismpc_walking/policy/PolicyContract.h"

#include <string>
#include <vector>

namespace ismpc_walking::policy
{

/**
 * Diagnostics only (TEMPORARY, removed in step 9 with the other debug code): how far each observation element is
 * from the training distribution, z = (x - mean) / (std + eps) with the contract's own normalizer statistics.
 * This is the formula of rsl_rl's EmpiricalNormalization (default eps 1e-2), i.e. what the network sees first.
 *
 * Pure: no logging, no controller. Preallocated at construction, update() does not allocate.
 */
class ZScoreMonitor
{
public:
  static constexpr double kWarn = 6.0; // |z| above this is reported as an offender

  struct Offender
  {
    double z = 0;    // |z|
    size_t index = 0; // position in the observation vector
  };

  struct TermMax
  {
    double z = 0; // max |z| over the term since the last resetRunning()
    int elem = 0; // element inside the term where it happened
  };

  /** Copies what it needs: keeps no reference to the contract or the builder. */
  ZScoreMonitor(const PolicyContract & contract, const std::vector<ObservationBuilder::Term> & terms);

  /** False when the contract has no normalizer: update() then does nothing. */
  bool available() const noexcept { return available_; }

  /** Looks at one observation (size obsDim, in doubles). Ignores a vector of the wrong size. */
  void update(const std::vector<double> & obs) noexcept;

  /** Elements of the LAST update() with |z| > kWarn, largest first. */
  const std::vector<Offender> & offenders() const noexcept { return offenders_; }

  /** Per term (builder order): max |z| over all update() calls since the last resetRunning(). */
  const std::vector<TermMax> & runningMax() const noexcept { return running_; }
  void resetRunning() noexcept;

  const std::vector<ObservationBuilder::Term> & terms() const noexcept { return terms_; }

  /** "joint_pos[RTMP]", "target_twist[1]", "ismpc_wants_stop" for an index of the observation vector. */
  std::string elementName(size_t index) const;
  /** The same for (term index, element inside the term). */
  std::string termElementName(size_t term, int elem) const;

private:
  bool available_ = false;
  double eps_ = 0;
  std::vector<double> mean_, std_;
  std::vector<ObservationBuilder::Term> terms_;
  std::vector<std::string> joints_;
  std::vector<Offender> offenders_;
  std::vector<TermMax> running_;
};

} // namespace ismpc_walking::policy
