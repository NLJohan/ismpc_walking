#include "ismpc_walking/policy/ActionDecoder.h"

#include <algorithm>
#include <cmath>

namespace ismpc_walking::policy
{

namespace
{
constexpr size_t kOffset = 0, kFrequency = 1, kSinAmp = 2, kCosAmp = 3, kWalk = 4, kTs = 5, kTwist = 6;

inline double clampd(double x, double lo, double hi) noexcept { return std::min(std::max(x, lo), hi); }
} // namespace

ActionDecoder::ActionDecoder(const PolicyContract & contract)
: k_(contract.action), defaults_(contract.reset), latchTicks_(std::max(1, contract.latch_ticks))
{
  reset();
}

void ActionDecoder::reset() noexcept
{
  // Literal 1, not 1 % latchTicks_: this is what IsmpcSineAction.reset() does.
  ticks_ = 1;
  current_.offset = defaults_.offset;
  current_.frequency = defaults_.frequency;
  current_.sin_amp = defaults_.sin_amp;
  current_.cos_amp = defaults_.cos_amp;
  current_.walk = defaults_.walk;
  current_.ts = defaults_.ts;
  current_.twist = defaults_.twist;
}

bool ActionDecoder::advance() noexcept
{
  const bool due = (ticks_ == 0);
  ticks_ = (ticks_ + 1) % latchTicks_;
  return due;
}

DecodedAction ActionDecoder::decode(const Raw & raw) const noexcept
{
  DecodedAction d;

  d.offset = clampd(k_.offset_scale * raw[kOffset] + k_.offset_bias, k_.offset_min, k_.offset_max);

  d.frequency =
      clampd(std::exp(k_.frequency_scale * raw[kFrequency] + k_.frequency_bias), k_.frequency_min, k_.frequency_max);

  // Amplitudes: scale, then cap the radius so offset -/+ radius stays inside [offset_min, offset_max].
  const double sinRaw = raw[kSinAmp] * k_.amplitude_scale;
  const double cosRaw = raw[kCosAmp] * k_.amplitude_scale;
  const double radius = std::sqrt(sinRaw * sinRaw + cosRaw * cosRaw);
  const double maxRadius = std::max(std::min(d.offset - k_.offset_min, k_.offset_max - d.offset), 0.0);
  const double ratio = radius > maxRadius ? maxRadius / std::max(radius, 1e-8) : 1.0;
  d.sin_amp = sinRaw * ratio;
  d.cos_amp = cosRaw * ratio;

  d.walk = raw[kWalk] > k_.walk_gate_bias;

  d.ts = clampd(k_.ts_scale * raw[kTs] + k_.ts_bias, k_.ts_min, k_.ts_max);

  // Twist: clamp then scale. No rate limiter.
  for(size_t i = 0; i < 3; ++i)
  {
    d.twist[i] = clampd(raw[kTwist + i], k_.twist_raw_clamp[0], k_.twist_raw_clamp[1]) * k_.twist_scale[i];
  }
  return d;
}

bool ActionDecoder::latch(const Raw & raw) noexcept
{
  for(double v : raw)
  {
    if(!std::isfinite(v)) { return false; }
  }
  current_ = decode(raw);
  return true;
}

} // namespace ismpc_walking::policy
