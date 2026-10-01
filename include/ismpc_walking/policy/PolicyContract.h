#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace ismpc_walking::policy
{

struct ObsTermSpec
{
  std::string name;
  int dim = 0;
};

/** Training-time observation statistics. Diagnostics only (z-score monitor); the network has its own copy inside. */
struct ObsNormalizer
{
  std::vector<double> mean;
  std::vector<double> std;
  double eps = 0.0;
};

struct ActionConstants
{
  double offset_scale = 0, offset_bias = 0, offset_min = 0, offset_max = 0;
  double frequency_scale = 0, frequency_bias = 0, frequency_min = 0, frequency_max = 0;
  double amplitude_scale = 0;
  double walk_gate_bias = 0;
  double ts_scale = 0, ts_bias = 0, ts_min = 0, ts_max = 0, ts_default = 0;
  std::array<double, 3> twist_scale{};
  std::array<double, 2> twist_raw_clamp{};
  /** nullopt (JSON null) means: the twist is not rate limited. */
  std::optional<std::array<double, 3>> twist_max_delta_per_latch;
};

struct ResetDefaults
{
  double offset = 0, frequency = 0, sin_amp = 0, cos_amp = 0;
  bool walk = false;
  double ts = 0;
  std::array<double, 3> twist{};
};

struct CommandRanges
{
  std::array<double, 2> lin_vel_x{};
  std::array<double, 2> lin_vel_y{};
  std::array<double, 2> ang_vel_z{};
};

/**
 * The "ismpc_contract" JSON stored in the ONNX metadata, as a plain data structure.
 * Parsing and validation never throw: they return false and fill `err`.
 */
struct PolicyContract
{
  static constexpr int kSupportedVersion = 1;
  static constexpr int kActionDim = 9;

  int version = 0;
  std::string checkpoint_stem;
  long iteration = -1;
  std::string git_hash;

  double controller_dt = 0;
  int latch_ticks = 0;

  int obs_dim = 0;
  std::vector<ObsTermSpec> terms;
  std::optional<ObsNormalizer> normalizer;

  int action_dim = 0;
  // action.layout: sine [0,4), walk gate 4, Ts 5, twist [6,9)
  std::array<int, 2> layout_sine{};
  int layout_walk = 0;
  int layout_ts = 0;
  std::array<int, 2> layout_twist{};
  ActionConstants action;
  ResetDefaults reset;

  std::vector<std::string> joint_names;
  std::vector<double> default_pos;

  CommandRanges command;

  /**
   * filters.obs_filter_cutoff_T: cutoff period (s) of the controller-side low-pass filters behind the filt_* observation
   * terms, as in training. Required. The controller is forced to this value when the policy is activated.
   */
  double obs_filter_cutoff_T = 0;

  /** JSON text -> struct. Checks presence and types of every field, not their consistency. */
  static bool parse(const std::string & json, PolicyContract & out, std::string & err);

  /** Internal consistency (sizes, ranges, known terms, supported version/layout). */
  bool validate(std::string & err) const;

  /** Compatibility with the running controller: same control step, every joint exists on the robot. */
  bool validateAgainst(double controller_dt, const std::vector<std::string> & robot_joints, std::string & err) const;
};

/** Expected size of a known observation term, or -1 if the C++ side does not know the name. */
int expectedObsTermDim(const std::string & name, int n_joints);

} // namespace ismpc_walking::policy
