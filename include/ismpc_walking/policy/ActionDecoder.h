#pragma once

#include "ismpc_walking/policy/PolicyContract.h"

#include <array>
#include <string>

namespace ismpc_walking::policy
{

/** Physical values decoded from one raw action (what the controller is given). */
struct DecodedAction
{
  double offset = 0;    // m
  double frequency = 0; // Hz
  double sin_amp = 0;   // m
  double cos_amp = 0;   // m
  bool walk = false;
  double ts = 0;                  // s
  std::array<double, 3> twist{};  // vx, vy (m/s), omega (rad/s)
};

/**
 * Raw 9 network outputs -> physical values, plus the latch counter.
 *
 * Pure math, mirrors IsmpcSineAction (_map_to_physical, _map_walk_gate, _map_step_timing, _map_twist and the
 * latch bookkeeping of _advance_sine_period). All constants come from the contract.
 *
 * Twist: raw clamp and scale only. There is NO rate limiter, by design; the contract's
 * twist_max_delta_per_latch (if it were ever non null) is deliberately ignored.
 *
 * Raw layout (fixed, enforced by PolicyContract::validate): sine [0,4), walk gate 4, Ts 5, twist [6,9).
 */
class ActionDecoder
{
public:
  using Raw = std::array<double, PolicyContract::kActionDim>;

  /** Copies what it needs from the contract; keeps no reference to it. */
  explicit ActionDecoder(const PolicyContract & contract);

  /** Episode/enable start: counter seeded to 1 (as IsmpcSineAction.reset), values back to the contract reset defaults. */
  void reset() noexcept;

  /**
   * Call once per controller step. Returns true when this step is a latch step: due when the counter is 0,
   * checked BEFORE the increment (IsmpcSineAction._advance_sine_period). With latch_ticks = N and the seed 1,
   * the first latch is the N-th call, then every N calls.
   */
  bool advance() noexcept;

  /**
   * Decodes `raw` and makes it the current command. On a non finite raw value returns false and leaves the
   * current command untouched (the caller treats that as a failure).
   */
  bool latch(const Raw & raw) noexcept;

  /** The current (latched) command; the reset defaults until the first latch. */
  const DecodedAction & current() const noexcept { return current_; }

  /** Pure raw -> physical mapping, no state. */
  DecodedAction decode(const Raw & raw) const noexcept;

private:
  ActionConstants k_;
  ResetDefaults defaults_;
  int latchTicks_ = 1;
  int ticks_ = 1;
  DecodedAction current_;
};

/**
 * TEMPORARY dev scaffolding (step 5 parity test, removed in step 9).
 * Reads `in_path` (CSV, one raw action of 9 values per line, one per latch), replays the decoder with the
 * latch counter and writes `out_path` (CSV: latch,tick,offset,frequency,sin_amp,cos_amp,walk,ts,vx,vy,omega).
 * Compared against the real Python action term by tools/decoder_parity.py.
 */
bool runDecoderParity(const PolicyContract & contract, const std::string & in_path, const std::string & out_path,
                      std::string & err);

} // namespace ismpc_walking::policy
