#include "ismpc_walking/policy/PolicyRunner.h"

#include "ismpc_walking/policy/ActionDecoder.h"
#include "ismpc_walking/policy/OnnxBackend.h"

#include <mc_rtc/gui.h>
#include <mc_rtc/logging.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

#ifdef ISMPC_WITH_POLICY
#  include <onnxruntime_c_api.h>
#endif

namespace ismpc_walking::policy
{

const char * toString(PolicyState s) noexcept
{
  switch(s)
  {
    case PolicyState::NoPolicy: return "NoPolicy";
    case PolicyState::Ready: return "Ready";
    case PolicyState::Active: return "Active";
    case PolicyState::Releasing: return "Releasing";
  }
  return "?";
}

bool PolicyRunner::builtWithOnnxRuntime() noexcept
{
#ifdef ISMPC_WITH_POLICY
  return true;
#else
  return false;
#endif
}

std::string PolicyRunner::onnxRuntimeVersion()
{
#ifdef ISMPC_WITH_POLICY
  // C API call: resolves libonnxruntime.so at run time but creates no
  // environment, session or thread.
  const OrtApiBase * base = OrtGetApiBase();
  return base ? std::string(base->GetVersionString()) : std::string();
#else
  return {};
#endif
}

PolicyRunner::PolicyRunner(Options options, std::unique_ptr<StateSource> source)
: options_(std::move(options)), library_(options_.dir), selected_(options_.file)
{
  source_ = std::move(source);
  if(builtWithOnnxRuntime())
  {
    mc_rtc::log::info("[ismpc_policy] state=NoPolicy, built with ONNX Runtime {}", onnxRuntimeVersion());
  }
  else
  {
    mc_rtc::log::info("[ismpc_policy] state=NoPolicy, built without ONNX Runtime (ISMPC_WITH_POLICY=OFF)");
  }
  if(!options_.file.empty()) { load(options_.file); }
}

PolicyRunner::~PolicyRunner() = default;

std::string PolicyRunner::resolve(const std::string & file) const
{
  if(file.empty() || file.front() == '/' || options_.dir.empty()) { return file; }
  std::string d = options_.dir;
  if(d.back() != '/') { d += '/'; }
  return d + file;
}

bool PolicyRunner::fail(const std::string & path, const std::string & msg)
{
  backend_.reset();
  builder_.reset();
  decoder_.reset();
  obs_.clear();
  obsF_.clear();
  contract_ = PolicyContract{};
  loadedFile_.clear();
  benchAvgUs_ = benchMaxUs_ = 0;
  benchRuns_ = 0;
  state_ = PolicyState::NoPolicy;
  lastError_ = msg;
  mc_rtc::log::error("[ismpc_policy] load failed ({}): {}", path, msg);
  return false;
}

bool PolicyRunner::unload()
{
  if(state_ == PolicyState::Active || state_ == PolicyState::Releasing)
  {
    mc_rtc::log::warning("[ismpc_policy] cannot unload while {}", toString(state_));
    return false;
  }
  backend_.reset();
  builder_.reset();
  decoder_.reset();
  obs_.clear();
  obsF_.clear();
  contract_ = PolicyContract{};
  loadedFile_.clear();
  lastError_.clear();
  benchAvgUs_ = benchMaxUs_ = 0;
  benchRuns_ = 0;
  state_ = PolicyState::NoPolicy;
  return true;
}

bool PolicyRunner::load(const std::string & file)
{
  if(state_ == PolicyState::Active || state_ == PolicyState::Releasing)
  {
    mc_rtc::log::warning("[ismpc_policy] cannot load a model while {}", toString(state_));
    return false;
  }
  const std::string path = resolve(file);
  // A failed load always ends in NoPolicy: never keep a half-replaced model.
  unload();
  if(path.empty()) { return fail(path, "no file given"); }

  std::string err;

  // 1. ONNX Runtime: file, metadata, tensor signature
  auto backend = OnnxBackend::load(path, err);
  if(!backend) { return fail(path, err); }

  // 2. contract
  PolicyContract contract;
  if(!PolicyContract::parse(backend->contractJson(), contract, err)) { return fail(path, err); }
  if(!contract.validate(err)) { return fail(path, err); }
  if(!contract.validateAgainst(options_.controller_dt, options_.joint_names, err)) { return fail(path, err); }
  if(backend->obsDim() != contract.obs_dim)
  {
    return fail(path, "the network takes " + std::to_string(backend->obsDim()) + " inputs but the contract says obs.dim = "
                          + std::to_string(contract.obs_dim));
  }
  if(backend->actionDim() != contract.action_dim)
  {
    return fail(path, "the network returns " + std::to_string(backend->actionDim())
                          + " outputs but the contract says action.dim = " + std::to_string(contract.action_dim));
  }

  // 2b. observation builder, and the robot binding (contract joint names -> robot joints)
  auto builder = ObservationBuilder::create(contract, err);
  if(!builder) { return fail(path, err); }
  if(source_ && !source_->bind(contract.joint_names, err)) { return fail(path, err); }

  // 3. smoke inference on a zero observation, plus a timing measurement
  std::vector<float> obs(static_cast<size_t>(contract.obs_dim), 0.f);
  std::vector<float> act(static_cast<size_t>(contract.action_dim), std::numeric_limits<float>::quiet_NaN());
  if(!backend->infer(obs.data(), act.data(), err)) { return fail(path, "smoke inference failed: " + err); }
  for(size_t i = 0; i < act.size(); ++i)
  {
    if(!std::isfinite(act[i])) { return fail(path, "smoke inference returned a non finite value at output " + std::to_string(i)); }
  }
  constexpr int kBenchRuns = 200;
  double sumUs = 0, maxUs = 0;
  for(int i = 0; i < kBenchRuns; ++i)
  {
    const auto t0 = std::chrono::steady_clock::now();
    if(!backend->infer(obs.data(), act.data(), err)) { return fail(path, "inference failed: " + err); }
    const double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
    sumUs += us;
    maxUs = std::max(maxUs, us);
  }

  // 4. commit
  backend_ = std::move(backend);
  builder_ = std::move(builder);
  obs_.assign(static_cast<size_t>(contract.obs_dim), 0.0);
  obsF_.assign(static_cast<size_t>(contract.obs_dim), 0.f);
  contract_ = std::move(contract);
  decoder_ = std::make_unique<ActionDecoder>(contract_);
  loadedFile_ = path;
  lastError_.clear();
  benchAvgUs_ = sumUs / kBenchRuns;
  benchMaxUs_ = maxUs;
  benchRuns_ = kBenchRuns;
  state_ = PolicyState::Ready;
  mc_rtc::log::info(
      "[ismpc_policy] state=Ready: {} | checkpoint {} iteration {} | obs {} -> action {} | latch {} x {} s | "
      "twist rate limit: {} | inference avg {:.0f} us, max {:.0f} us over {} runs",
      loadedFile_, contract_.checkpoint_stem, contract_.iteration, contract_.obs_dim, contract_.action_dim,
      contract_.latch_ticks, contract_.controller_dt,
      contract_.action.twist_max_delta_per_latch ? "yes" : "none", sumUs / kBenchRuns, maxUs, kBenchRuns);

  // TEMPORARY (step 5 parity test, removed in step 9): replay a raw action file through the ActionDecoder.
  if(const char * parityIn = std::getenv("ISMPC_DECODER_PARITY_IN"))
  {
    const char * parityOut = std::getenv("ISMPC_DECODER_PARITY_OUT");
    std::string perr;
    if(!parityOut) { perr = "ISMPC_DECODER_PARITY_OUT is not set"; }
    if(parityOut && runDecoderParity(contract_, parityIn, parityOut, perr))
    {
      mc_rtc::log::info("[ismpc_policy] decoder parity: wrote {}", parityOut);
    }
    else
    {
      mc_rtc::log::error("[ismpc_policy] decoder parity failed: {}", perr);
    }
  }
  return true;
}

void PolicyRunner::tick() noexcept
{
  if(guiRebuildPending_)
  {
    guiRebuildPending_ = false;
    try
    {
      if(gui_)
      {
        gui_->removeCategory(guiCategory_);
        buildGui();
      }
    }
    catch(const std::exception & e)
    {
      mc_rtc::log::error("[ismpc_policy] GUI rebuild failed: {}", e.what());
    }
    catch(...)
    {
      mc_rtc::log::error("[ismpc_policy] GUI rebuild failed");
    }
  }
  if(source_ && state_ != PolicyState::NoPolicy) { source_->sample(); } // every step: keeps the datastore-lag history
  if(state_ == PolicyState::Ready && options_.debug_no_apply) { debugTick(); }
  if(state_ != PolicyState::Active && state_ != PolicyState::Releasing) { return; }
}

namespace
{
std::string fmtVec3(const Vec3 & v)
{
  char buf[96];
  std::snprintf(buf, sizeof(buf), "[%+.3f %+.3f %+.3f]", v[0], v[1], v[2]);
  return buf;
}

std::string fmtN(const double * v, int n)
{
  std::string out = "[";
  char buf[32];
  for(int i = 0; i < n; ++i)
  {
    std::snprintf(buf, sizeof(buf), i ? " %+.3f" : "%+.3f", v[i]);
    out += buf;
  }
  return out + "]";
}
} // namespace

// TEMPORARY (step 6, removed in step 9): once per second while Ready with policy.debug_no_apply, read the robot
// state, build the state terms and log them. Reads only; nothing is written to the controller.
void PolicyRunner::debugTick() noexcept
{
  try
  {
    if(!source_ || !builder_) { return; }
    const long every = std::max<long>(1, std::lround(1.0 / options_.controller_dt));
    if(++debugTicks_ % every != 0) { return; }

    RobotState s;
    std::string err;
    if(!source_->read(s, err))
    {
      mc_rtc::log::warning("[ismpc_policy] debug obs: {}", err);
      return;
    }
    if(!builder_->build(s, decoder_->current(), obs_, err) || !ObservationBuilder::toFloat(obs_, obsF_, err))
    {
      mc_rtc::log::warning("[ismpc_policy] debug obs: {}", err);
      return;
    }

    auto worst = [&](const char * term, std::string & name) {
      const auto * t = builder_->find(term);
      double m = 0;
      name = "-";
      if(!t) { return m; }
      for(int i = 0; i < t->dim; ++i)
      {
        const double a = std::fabs(obs_[static_cast<size_t>(t->offset + i)]);
        if(a >= m)
        {
          m = a;
          name = contract_.joint_names[static_cast<size_t>(i)];
        }
      }
      return m;
    };
    std::string posName, velName;
    const double posMax = worst("joint_pos", posName);
    const double velMax = worst("joint_vel", velName);
    const std::string gyro = s.diag.has_gyro ? fmtVec3(s.diag.gyro) : std::string("n/a");
    mc_rtc::log::info(
        "[ismpc_policy] debug obs | com_lin_vel {} | base_ang_vel {} (velW) gyro {} | gravity {} | "
        "joint_pos max|.| {:.4f} ({}) | joint_vel max|.| {:.4f} ({}) from {} | encoder vel max {:.4f} alpha max {:.4f}",
        fmtVec3(s.com_lin_vel), fmtVec3(s.base_ang_vel), gyro, fmtVec3(s.projected_gravity), posMax, posName, velMax,
        velName, s.diag.joint_vel_source, s.diag.encoder_vel_max_abs, s.diag.alpha_max_abs);

    // Latched / command terms, as they sit in the observation vector.
    auto slice = [&](const char * term) {
      const auto * t = builder_->find(term);
      return t ? obs_.data() + t->offset : nullptr;
    };
    const double * sine = slice("last_sine_params");
    const double * walk = slice("last_walk_action");
    const double * ts = slice("last_step_timing_action");
    const double * twist = slice("last_twist_action");
    const double * stop = slice("ismpc_wants_stop");
    const double * target = slice("target_twist");
    if(sine && walk && ts && twist && stop && target)
    {
      mc_rtc::log::info(
          "[ismpc_policy] debug obs | last_sine {} walk {:.0f} ts {:.3f} last_twist {} | wants_stop {:.0f} | "
          "target_twist raw {} -> obs {} | float32 vector of {} values ok",
          fmtN(sine, 4), *walk, *ts, fmtN(twist, 3), *stop, fmtVec3(s.target_twist), fmtN(target, 3), obsF_.size());
    }
  }
  catch(...)
  {
  }
}

bool PolicyRunner::setActive(bool on)
{
  if(on) { mc_rtc::log::warning("[ismpc_policy] the policy cannot be activated yet (not implemented in this build step)"); }
  return false;
}

void PolicyRunner::addGui(mc_rtc::gui::StateBuilder & gui, const std::vector<std::string> & category)
{
  gui_ = &gui;
  guiCategory_ = category;
  buildGui();
}

void PolicyRunner::buildGui()
{
  auto busy = [this]() { return state_ == PolicyState::Active || state_ == PolicyState::Releasing; };
  gui_->addElement(
      guiCategory_,
      mc_rtc::gui::Checkbox(
          "Policy", [this]() { return state_ == PolicyState::Active; },
          [this]() { setActive(state_ != PolicyState::Active); }),
      mc_rtc::gui::ComboInput(
          "Policy file", library_.files(), [this]() { return selected_; },
          [this, busy](const std::string & f) {
            if(busy())
            {
              mc_rtc::log::warning("[ismpc_policy] cannot change the policy file while {}", toString(state_));
              return;
            }
            selected_ = f;
            load(f);
          }),
      mc_rtc::gui::Button("Refresh list",
                          [this]() {
                            library_.refresh();
                            guiRebuildPending_ = true; // dropdown values are fixed at creation: rebuilt in tick()
                          }),
      mc_rtc::gui::Button("Reload",
                          [this]() {
                            if(selected_.empty())
                            {
                              mc_rtc::log::warning("[ismpc_policy] Reload: no policy file selected");
                              return;
                            }
                            load(selected_);
                          }),
      mc_rtc::gui::Label("State", [this]() { return std::string(toString(state_)); }),
      mc_rtc::gui::Label("Last error", [this]() { return lastError_.empty() ? std::string("none") : lastError_; }),
      mc_rtc::gui::Label("Loaded file", [this]() { return loadedFile_.empty() ? std::string("-") : loadedFile_; }),
      mc_rtc::gui::Label("Observation size",
                         [this]() { return contract() ? std::to_string(contract_.obs_dim) : std::string("-"); }),
      mc_rtc::gui::Label("Action size",
                         [this]() { return contract() ? std::to_string(contract_.action_dim) : std::string("-"); }),
      mc_rtc::gui::Label("Latch period",
                         [this]() {
                           if(!contract()) { return std::string("-"); }
                           char buf[96];
                           std::snprintf(buf, sizeof(buf), "%d x %g s = %g ms", contract_.latch_ticks,
                                         contract_.controller_dt, 1000. * contract_.latch_ticks * contract_.controller_dt);
                           return std::string(buf);
                         }),
      mc_rtc::gui::Label("Checkpoint iteration",
                         [this]() {
                           return contract() ? contract_.checkpoint_stem + " / " + std::to_string(contract_.iteration)
                                             : std::string("-");
                         }),
      mc_rtc::gui::Label("Inference time avg / max (us, at load)", [this]() {
        if(benchRuns_ == 0) { return std::string("-"); }
        char buf[96];
        std::snprintf(buf, sizeof(buf), "%.0f / %.0f (%d runs)", benchAvgUs_, benchMaxUs_, benchRuns_);
        return std::string(buf);
      }));
}

} // namespace ismpc_walking::policy
