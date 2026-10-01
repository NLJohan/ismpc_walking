#include "ismpc_walking/policy/PolicyContract.h"

#include <mc_rtc/Configuration.h>

#include <algorithm>
#include <cmath>
#include <set>
#include <sstream>
#include <stdexcept>

namespace ismpc_walking::policy
{

namespace
{

using mc_rtc::Configuration;

[[noreturn]] void fail(const std::string & path, const std::string & msg)
{
  throw std::runtime_error(path + ": " + msg);
}

Configuration child(const Configuration & c, const std::string & key, const std::string & path)
{
  if(!c.has(key)) { fail(path, "missing key '" + key + "'"); }
  return c(key);
}

// Every number is read as a double: mc_rtc's integer casts are strict about the stored type.
double num(const Configuration & c, const std::string & path)
{
  if(!c.isNumeric()) { fail(path, "expected a number"); }
  const double v = static_cast<double>(c);
  if(!std::isfinite(v)) { fail(path, "number is not finite"); }
  return v;
}

int integer(const Configuration & c, const std::string & path)
{
  const double v = num(c, path);
  if(v != std::floor(v) || std::fabs(v) > 1e9) { fail(path, "expected an integer"); }
  return static_cast<int>(v);
}

std::string str(const Configuration & c, const std::string & path)
{
  if(!c.isString()) { fail(path, "expected a string"); }
  return static_cast<std::string>(c);
}

Configuration array(const Configuration & c, const std::string & path, long expected = -1)
{
  if(!c.isArray()) { fail(path, "expected an array"); }
  if(expected >= 0 && static_cast<long>(c.size()) != expected)
  {
    fail(path, "expected an array of size " + std::to_string(expected) + ", got " + std::to_string(c.size()));
  }
  return c;
}

std::vector<double> numVector(const Configuration & c, const std::string & path, long expected = -1)
{
  array(c, path, expected);
  std::vector<double> out;
  out.reserve(c.size());
  for(size_t i = 0; i < c.size(); ++i) { out.push_back(num(c[i], path + "[" + std::to_string(i) + "]")); }
  return out;
}

template<size_t N>
std::array<double, N> numArray(const Configuration & c, const std::string & path)
{
  const auto v = numVector(c, path, static_cast<long>(N));
  std::array<double, N> out{};
  std::copy(v.begin(), v.end(), out.begin());
  return out;
}

double numKey(const Configuration & c, const std::string & key, const std::string & path)
{
  return num(child(c, key, path), path + "." + key);
}

bool finite(double v) { return std::isfinite(v); }

bool fmtErr(std::string & err, const std::string & msg)
{
  err = "policy contract: " + msg;
  return false;
}

} // namespace

int expectedObsTermDim(const std::string & name, int n_joints)
{
  // Keep in sync with the ObservationBuilder table (step 6) and the env cfg.
  if(name == "com_lin_vel") { return 3; }
  if(name == "base_ang_vel") { return 3; }
  if(name == "projected_gravity") { return 3; }
  if(name == "joint_pos") { return n_joints; }
  if(name == "joint_vel") { return n_joints; }
  if(name == "last_sine_params") { return 4; }
  if(name == "last_walk_action") { return 1; }
  if(name == "last_step_timing_action") { return 1; }
  if(name == "last_twist_action") { return 3; }
  if(name == "ismpc_wants_stop") { return 1; }
  if(name == "filt_perturbation") { return 1; }
  if(name == "filt_zmp_error") { return 1; }
  if(name == "filt_dcm_bias") { return 1; }
  if(name == "target_twist") { return 3; }
  return -1;
}

bool PolicyContract::parse(const std::string & json, PolicyContract & out, std::string & err)
{
  try
  {
    PolicyContract c;
    const Configuration root = Configuration::fromData(json);
    if(!root.isObject()) { fail("contract", "top level is not a JSON object"); }

    c.version = integer(child(root, "contract_version", "contract"), "contract_version");

    if(root.has("checkpoint") && root("checkpoint").isObject())
    {
      const auto ck = root("checkpoint");
      if(ck.has("stem") && ck("stem").isString()) { c.checkpoint_stem = str(ck("stem"), "checkpoint.stem"); }
      if(ck.has("iteration") && ck("iteration").isNumeric()) { c.iteration = integer(ck("iteration"), "checkpoint.iteration"); }
      if(ck.has("git") && ck("git").isObject() && ck("git").has("hash") && ck("git")("hash").isString())
      {
        c.git_hash = str(ck("git")("hash"), "checkpoint.git.hash");
      }
    }

    c.controller_dt = numKey(root, "controller_dt", "contract");
    c.latch_ticks = integer(child(root, "latch_ticks", "contract"), "latch_ticks");

    // --- obs
    const auto obs = child(root, "obs", "contract");
    c.obs_dim = integer(child(obs, "dim", "obs"), "obs.dim");
    const auto terms = array(child(obs, "terms", "obs"), "obs.terms");
    for(size_t i = 0; i < terms.size(); ++i)
    {
      const std::string p = "obs.terms[" + std::to_string(i) + "]";
      const auto t = terms[i];
      ObsTermSpec spec;
      spec.name = str(child(t, "name", p), p + ".name");
      spec.dim = integer(child(t, "dim", p), p + ".dim");
      c.terms.push_back(spec);
    }
    if(obs.has("normalizer") && obs("normalizer").isObject())
    {
      const auto n = obs("normalizer");
      ObsNormalizer norm;
      norm.mean = numVector(child(n, "mean", "obs.normalizer"), "obs.normalizer.mean");
      norm.std = numVector(child(n, "std", "obs.normalizer"), "obs.normalizer.std");
      norm.eps = numKey(n, "eps", "obs.normalizer");
      c.normalizer = std::move(norm);
    }

    // --- action
    const auto act = child(root, "action", "contract");
    c.action_dim = integer(child(act, "dim", "action"), "action.dim");
    const auto layout = child(act, "layout", "action");
    const auto sine = numVector(child(layout, "sine", "action.layout"), "action.layout.sine", 2);
    c.layout_sine = {static_cast<int>(sine[0]), static_cast<int>(sine[1])};
    c.layout_walk = integer(child(layout, "walk_gate", "action.layout"), "action.layout.walk_gate");
    c.layout_ts = integer(child(layout, "ts", "action.layout"), "action.layout.ts");
    const auto tw = numVector(child(layout, "twist", "action.layout"), "action.layout.twist", 2);
    c.layout_twist = {static_cast<int>(tw[0]), static_cast<int>(tw[1])};

    const auto k = child(act, "constants", "action");
    const std::string kp = "action.constants";
    auto & a = c.action;
    a.offset_scale = numKey(k, "offset_scale", kp);
    a.offset_bias = numKey(k, "offset_bias", kp);
    a.offset_min = numKey(k, "offset_min", kp);
    a.offset_max = numKey(k, "offset_max", kp);
    a.frequency_scale = numKey(k, "frequency_scale", kp);
    a.frequency_bias = numKey(k, "frequency_bias", kp);
    a.frequency_min = numKey(k, "frequency_min", kp);
    a.frequency_max = numKey(k, "frequency_max", kp);
    a.amplitude_scale = numKey(k, "amplitude_scale", kp);
    a.walk_gate_bias = numKey(k, "walk_gate_bias", kp);
    a.ts_scale = numKey(k, "ts_scale", kp);
    a.ts_bias = numKey(k, "ts_bias", kp);
    a.ts_min = numKey(k, "ts_min", kp);
    a.ts_max = numKey(k, "ts_max", kp);
    a.ts_default = numKey(k, "ts_default", kp);
    a.twist_scale = numArray<3>(child(k, "twist_scale", kp), kp + ".twist_scale");
    a.twist_raw_clamp = numArray<2>(child(k, "twist_raw_clamp", kp), kp + ".twist_raw_clamp");
    // null (or absent) = no rate limit; an array = per-latch limits.
    if(k.has("twist_max_delta_per_latch") && k("twist_max_delta_per_latch").isArray())
    {
      a.twist_max_delta_per_latch =
          numArray<3>(k("twist_max_delta_per_latch"), kp + ".twist_max_delta_per_latch");
    }

    const auto r = child(act, "reset_defaults", "action");
    const std::string rp = "action.reset_defaults";
    c.reset.offset = numKey(r, "offset", rp);
    c.reset.frequency = numKey(r, "frequency", rp);
    c.reset.sin_amp = numKey(r, "sin_amp", rp);
    c.reset.cos_amp = numKey(r, "cos_amp", rp);
    c.reset.ts = numKey(r, "ts", rp);
    c.reset.twist = numArray<3>(child(r, "twist", rp), rp + ".twist");
    c.reset.walk = false;
    if(r.has("walk")) { c.reset.walk = static_cast<bool>(r("walk")); }

    // --- joints
    const auto j = child(root, "joints", "contract");
    const auto names = array(child(j, "names", "joints"), "joints.names");
    for(size_t i = 0; i < names.size(); ++i)
    {
      c.joint_names.push_back(str(names[i], "joints.names[" + std::to_string(i) + "]"));
    }
    c.default_pos = numVector(child(j, "default_pos", "joints"), "joints.default_pos");

    // --- command
    const auto cmd = child(root, "command", "contract");
    const auto rng = child(cmd, "ranges", "command");
    c.command.lin_vel_x = numArray<2>(child(rng, "lin_vel_x", "command.ranges"), "command.ranges.lin_vel_x");
    c.command.lin_vel_y = numArray<2>(child(rng, "lin_vel_y", "command.ranges"), "command.ranges.lin_vel_y");
    c.command.ang_vel_z = numArray<2>(child(rng, "ang_vel_z", "command.ranges"), "command.ranges.ang_vel_z");

    // --- filters (required: the filt_* observation terms are defined by this cutoff)
    const auto flt = child(root, "filters", "contract");
    c.obs_filter_cutoff_T = numKey(flt, "obs_filter_cutoff_T", "filters");

    out = std::move(c);
    return true;
  }
  catch(const std::exception & e)
  {
    err = std::string("policy contract: cannot parse: ") + e.what();
    return false;
  }
  catch(...)
  {
    err = "policy contract: cannot parse: unknown error";
    return false;
  }
}

bool PolicyContract::validate(std::string & err) const
{
  if(version < 1 || version > kSupportedVersion)
  {
    return fmtErr(err, "contract_version " + std::to_string(version) + " is not supported (this build understands 1 to "
                           + std::to_string(kSupportedVersion) + ")");
  }
  if(!finite(controller_dt) || controller_dt <= 0) { return fmtErr(err, "controller_dt must be > 0"); }
  if(latch_ticks < 1 || latch_ticks > 100000)
  {
    return fmtErr(err, "latch_ticks " + std::to_string(latch_ticks) + " is invalid");
  }

  if(joint_names.empty()) { return fmtErr(err, "joints.names is empty"); }
  {
    std::set<std::string> seen;
    for(const auto & n : joint_names)
    {
      if(!seen.insert(n).second) { return fmtErr(err, "duplicate joint name '" + n + "'"); }
    }
  }
  if(default_pos.size() != joint_names.size())
  {
    return fmtErr(err, "joints.default_pos has " + std::to_string(default_pos.size()) + " values for "
                           + std::to_string(joint_names.size()) + " joints");
  }
  for(double v : default_pos)
  {
    if(!finite(v)) { return fmtErr(err, "joints.default_pos contains a non finite value"); }
  }

  if(obs_dim <= 0) { return fmtErr(err, "obs.dim must be > 0"); }
  if(terms.empty()) { return fmtErr(err, "obs.terms is empty"); }
  {
    int sum = 0;
    std::set<std::string> seen;
    for(const auto & t : terms)
    {
      if(!seen.insert(t.name).second) { return fmtErr(err, "observation term '" + t.name + "' appears twice"); }
      const int expected = expectedObsTermDim(t.name, static_cast<int>(joint_names.size()));
      if(expected < 0) { return fmtErr(err, "unknown observation term '" + t.name + "'"); }
      if(t.dim != expected)
      {
        return fmtErr(err, "observation term '" + t.name + "' has dim " + std::to_string(t.dim) + ", expected "
                               + std::to_string(expected));
      }
      sum += t.dim;
    }
    if(sum != obs_dim)
    {
      return fmtErr(err, "obs.dim is " + std::to_string(obs_dim) + " but the terms add up to " + std::to_string(sum));
    }
  }
  if(normalizer)
  {
    const auto & n = *normalizer;
    if(static_cast<int>(n.mean.size()) != obs_dim || static_cast<int>(n.std.size()) != obs_dim)
    {
      return fmtErr(err, "normalizer mean/std sizes (" + std::to_string(n.mean.size()) + "/"
                             + std::to_string(n.std.size()) + ") differ from obs.dim " + std::to_string(obs_dim));
    }
    if(!(n.eps > 0)) { return fmtErr(err, "normalizer eps must be > 0"); }
    for(size_t i = 0; i < n.std.size(); ++i)
    {
      if(!(n.std[i] >= 0)) { return fmtErr(err, "normalizer std[" + std::to_string(i) + "] is negative"); }
    }
  }

  if(action_dim != kActionDim)
  {
    return fmtErr(err, "action.dim is " + std::to_string(action_dim) + ", this build only handles " + std::to_string(kActionDim));
  }
  if(layout_sine != std::array<int, 2>{0, 4} || layout_walk != 4 || layout_ts != 5
     || layout_twist != std::array<int, 2>{6, 9})
  {
    return fmtErr(err, "unsupported action layout (expected sine [0,4), walk_gate 4, ts 5, twist [6,9))");
  }

  const auto & a = action;
  if(!(a.offset_min < a.offset_max)) { return fmtErr(err, "offset_min must be < offset_max"); }
  if(!(a.frequency_min > 0 && a.frequency_min < a.frequency_max))
  {
    return fmtErr(err, "need 0 < frequency_min < frequency_max");
  }
  if(!(a.ts_min > 0 && a.ts_min < a.ts_max)) { return fmtErr(err, "need 0 < ts_min < ts_max"); }
  if(!(a.twist_raw_clamp[0] < a.twist_raw_clamp[1])) { return fmtErr(err, "twist_raw_clamp must be increasing"); }
  if(a.twist_max_delta_per_latch)
  {
    for(double v : *a.twist_max_delta_per_latch)
    {
      if(!(v >= 0)) { return fmtErr(err, "twist_max_delta_per_latch must be >= 0"); }
    }
  }
  if(!finite(a.offset_scale) || !finite(a.offset_bias) || !finite(a.frequency_scale) || !finite(a.frequency_bias)
     || !finite(a.amplitude_scale) || !finite(a.walk_gate_bias) || !finite(a.ts_scale) || !finite(a.ts_bias)
     || !finite(a.ts_default))
  {
    return fmtErr(err, "a scale/bias constant is not finite");
  }
  if(!finite(reset.offset) || !finite(reset.frequency) || !finite(reset.sin_amp) || !finite(reset.cos_amp)
     || !finite(reset.ts))
  {
    return fmtErr(err, "a reset default is not finite");
  }

  for(const auto & r : {command.lin_vel_x, command.lin_vel_y, command.ang_vel_z})
  {
    if(!(r[0] <= r[1])) { return fmtErr(err, "command range with min > max"); }
  }
  if(!finite(obs_filter_cutoff_T) || !(obs_filter_cutoff_T > 0))
  {
    return fmtErr(err, "filters.obs_filter_cutoff_T must be > 0");
  }
  return true;
}

bool PolicyContract::validateAgainst(double controller_dt_running, const std::vector<std::string> & robot_joints,
                                     std::string & err) const
{
  if(std::fabs(controller_dt - controller_dt_running) > 1e-6)
  {
    std::ostringstream ss;
    ss << "controller step mismatch: the policy was trained at " << controller_dt << " s, the controller runs at "
       << controller_dt_running << " s";
    return fmtErr(err, ss.str());
  }
  const std::set<std::string> have(robot_joints.begin(), robot_joints.end());
  for(const auto & n : joint_names)
  {
    if(!have.count(n)) { return fmtErr(err, "joint '" + n + "' of the policy does not exist on the robot"); }
  }
  return true;
}

} // namespace ismpc_walking::policy
