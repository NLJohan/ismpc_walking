#include "ismpc_walking/policy/PolicyRunner.h"

#include "ismpc_walking/policy/ActionDecoder.h"
#include "ismpc_walking/policy/OnnxBackend.h"

#include <mc_rtc/gui.h>
#include <mc_rtc/logging.h>

#include <algorithm>
#include <array>
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

PolicyRunner::PolicyRunner(Options options, std::unique_ptr<StateSource> source, ControlSink * sink)
: options_(std::move(options)), library_(options_.dir), selected_(options_.file)
{
  source_ = std::move(source);
  sink_ = sink;
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
  zmon_.reset();
  obs_.clear();
  obsF_.clear();
  latchCount_ = 0;
  inferSumUs_ = inferMaxUs_ = 0;
  shadowError_.clear();
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
  zmon_.reset();
  obs_.clear();
  obsF_.clear();
  latchCount_ = 0;
  inferSumUs_ = inferMaxUs_ = 0;
  shadowError_.clear();
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
  zmon_ = std::make_unique<ZScoreMonitor>(contract_, builder_->terms());
  debugTicks_ = 0;
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
  switch(request_.exchange(0))
  {
    case 1: activate(); break;
    case 2:
      if(state_ == PolicyState::Active) { release("the policy was switched off", false); }
      break;
    default: break;
  }
  if(state_ == PolicyState::Active) { activeTick(); }
  else if(state_ == PolicyState::Ready && options_.debug_no_apply) { debugTick(); }
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

// TEMPORARY (steps 6A to 6B, removed in step 9). While Ready with policy.debug_no_apply the policy runs in shadow:
//  - every controller step the latch counter advances (same seed as training);
//  - on a latch step the observation is built, the network is run, the action is decoded and latched (so the last_*
//    terms evolve as in a real run) and the z-score monitor looks at the observation the network saw;
//  - once per second everything is logged.
// It reads the robot only: nothing is written to the controller.
void PolicyRunner::debugTick() noexcept
{
  try
  {
    if(!source_ || !builder_ || !backend_ || !decoder_ || !zmon_) { return; }
    const long every = std::max<long>(1, std::lround(1.0 / options_.controller_dt));
    ++debugTicks_;
    const bool latchDue = decoder_->advance();
    const bool logDue = (debugTicks_ % every == 0);
    if(!latchDue && !logDue) { return; }

    // Failures are remembered and reported in the once-per-second line, so a persistent one cannot flood the log.
    auto failStep = [&](const std::string & msg) {
      shadowError_ = msg;
      if(logDue) { mc_rtc::log::warning("[ismpc_policy] debug obs: {}", msg); }
    };

    RobotState s;
    std::string err;
    if(!source_->read(s, err)) { return failStep(err); }
    if(!builder_->build(s, decoder_->current(), obs_, err) || !ObservationBuilder::toFloat(obs_, obsF_, err))
    {
      return failStep(err);
    }

    if(latchDue)
    {
      std::array<float, PolicyContract::kActionDim> act{};
      const auto t0 = std::chrono::steady_clock::now();
      const bool ran = backend_->infer(obsF_.data(), act.data(), err);
      const double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
      if(!ran) { return failStep("shadow inference failed: " + err); }
      ActionDecoder::Raw raw{};
      for(size_t i = 0; i < raw.size(); ++i) { raw[i] = static_cast<double>(act[i]); }
      if(!decoder_->latch(raw)) { return failStep("shadow inference returned a non finite value"); }
      lastRaw_ = raw;
      shadowError_.clear();
      ++latchCount_;
      inferSumUs_ += us;
      inferMaxUs_ = std::max(inferMaxUs_, us);
      zmon_->update(obs_); // obs_ is still the observation the network just saw
      if(latchCount_ == 1)
      {
        if(!zmon_->available())
        {
          mc_rtc::log::warning("[ismpc_policy] z-score monitor: the contract has no normalizer, no z-scores");
        }
        else
        {
          const auto & off = zmon_->offenders();
          std::string list;
          constexpr size_t kListed = 30;
          for(size_t i = 0; i < off.size() && i < kListed; ++i)
          {
            char buf[48];
            std::snprintf(buf, sizeof(buf), "=%.1f", off[i].z);
            list += (i ? ", " : "") + zmon_->elementName(off[i].index) + buf;
          }
          mc_rtc::log::info("[ismpc_policy] z-score first latch: {} of {} elements with |z| > {:.0f}{}{}", off.size(),
                            contract_.obs_dim, ZScoreMonitor::kWarn, off.empty() ? "" : ": ", list);
        }
      }
    }

    if(!logDue) { return; }

    auto worst = [&](const char * term, std::string & name) {
      const auto * t = builder_->find(term);
      double m = 0;
      name = "-";
      if(!t) { return m; }
      for(int i = 0; i < t->dim; ++i)
      {
        const double a = std::fabs(obs_[static_cast<size_t>(t->offset + i)]);
        if(a > m)
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

    // Latched / command terms, as they sit in the observation vector the network last saw.
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

    // Shadow policy: what the network says, and how long it took.
    if(latchCount_ > 0)
    {
      const auto & d = decoder_->current();
      mc_rtc::log::info(
          "[ismpc_policy] shadow | latches {} | raw {} | decoded offset {:.3f} freq {:.3f} sin {:.4f} cos {:.4f} walk {} "
          "ts {:.3f} twist {} | inference avg {:.1f} us max {:.1f} us{}{}",
          latchCount_, fmtN(lastRaw_.data(), static_cast<int>(lastRaw_.size())), d.offset, d.frequency, d.sin_amp,
          d.cos_amp, d.walk ? 1 : 0, d.ts, fmtN(d.twist.data(), 3), inferSumUs_ / static_cast<double>(latchCount_),
          inferMaxUs_, shadowError_.empty() ? "" : " | LAST ERROR: ", shadowError_);
    }
    else if(!shadowError_.empty())
    {
      mc_rtc::log::warning("[ismpc_policy] shadow: no latch yet, last error: {}", shadowError_);
    }

    // z-score monitor: max |z| per term over the latches since the last line.
    if(zmon_->available() && latchCount_ > 0)
    {
      std::string line;
      const auto & run = zmon_->runningMax();
      for(size_t ti = 0; ti < run.size(); ++ti)
      {
        char buf[32];
        std::snprintf(buf, sizeof(buf), " %.1f", run[ti].z);
        // name the element only where it helps: an offender inside a multi-element term
        const bool named = run[ti].z > ZScoreMonitor::kWarn && zmon_->terms()[ti].dim > 1;
        line += (ti ? " | " : "") + (named ? zmon_->termElementName(ti, run[ti].elem) : zmon_->terms()[ti].name) + buf;
      }
      mc_rtc::log::info("[ismpc_policy] z max since last line | {} | elements > {:.0f} at last latch: {}", line,
                        ZScoreMonitor::kWarn, zmon_->offenders().size());
      zmon_->resetRunning();
    }
  }
  catch(...)
  {
  }
}

void PolicyRunner::activate() noexcept
{
  try
  {
    auto refuse = [this](const std::string & why) {
      lastError_ = "cannot activate: " + why;
      mc_rtc::log::warning("[ismpc_policy] {}", lastError_);
    };
    if(state_ != PolicyState::Ready) { return refuse(std::string("the state is ") + toString(state_) + ", it must be Ready"); }
    if(!sink_ || !source_ || !builder_ || !backend_ || !decoder_ || !zmon_)
    {
      return refuse("the runner has no controller to drive or is not fully initialised");
    }
    std::string err;
    if(!sink_->canTakeOver(err)) { return refuse(err); }

    // Activation is training's episode reset: counter seeded to 1 (first latch on the 10th step), command back to
    // the reset defaults, so the first thing applied is offset 0.9, frequency 0, amplitudes 0, walk off, Ts 1.1, twist 0.
    decoder_->reset();
    zmon_->resetRunning();
    lastRaw_ = ActionDecoder::Raw{};
    latchCount_ = 0;
    inferSumUs_ = inferMaxUs_ = 0;
    shadowError_.clear();
    activeTicks_ = 0;
    lastError_.clear();
    sink_->takeOwnership();
    state_ = PolicyState::Active;
    mc_rtc::log::info("[ismpc_policy] state=Active: the policy owns walking, Ts and the velocity source");
  }
  catch(...)
  {
  }
}

void PolicyRunner::release(const std::string & reason, bool failure) noexcept
{
  try
  {
    if(state_ != PolicyState::Active) { return; }
    if(sink_)
    {
      if(failure) { sink_->applyWalkGate(false); } // stop walking
      sink_->releaseOwnership();
    }
    state_ = PolicyState::Ready;
    if(failure)
    {
      lastError_ = reason;
      mc_rtc::log::error("[ismpc_policy] FAILURE, back to Ready (Stop set): {}", reason);
    }
    else
    {
      mc_rtc::log::info("[ismpc_policy] state=Ready: {}", reason);
    }
  }
  catch(...)
  {
  }
}

// The Active loop (step 7B: the whole command, i.e. sine parameters, walk gate, Ts and twist). Per controller step:
//  1. apply the command that was latched on a PREVIOUS step (training hands a latched action to the controller one
//     control period late, see B.10 item 5);
//  2. advance the latch counter; on a latch step read the robot, build the observation, run the network, latch.
// Any failure releases the policy at once (release(..., true)).
void PolicyRunner::activeTick() noexcept
{
  try
  {
    if(!source_ || !sink_ || !builder_ || !backend_ || !decoder_ || !zmon_)
    {
      release("internal error: the runner is not fully initialised", true);
      return;
    }
    const long every = std::max<long>(1, std::lround(1.0 / options_.controller_dt));
    ++activeTicks_;

    sink_->applyCommand(decoder_->current());

    if(decoder_->advance())
    {
      RobotState s;
      std::string err;
      if(!source_->read(s, err))
      {
        release(err, true);
        return;
      }
      if(!builder_->build(s, decoder_->current(), obs_, err) || !ObservationBuilder::toFloat(obs_, obsF_, err))
      {
        release(err, true);
        return;
      }
      std::array<float, PolicyContract::kActionDim> act{};
      const auto t0 = std::chrono::steady_clock::now();
      const bool ran = backend_->infer(obsF_.data(), act.data(), err);
      const double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
      if(!ran)
      {
        release("inference failed: " + err, true);
        return;
      }
      ActionDecoder::Raw raw{};
      for(size_t i = 0; i < raw.size(); ++i) { raw[i] = static_cast<double>(act[i]); }
      if(!decoder_->latch(raw))
      {
        release("the network returned a non finite value", true);
        return;
      }
      lastRaw_ = raw;
      ++latchCount_;
      inferSumUs_ += us;
      inferMaxUs_ = std::max(inferMaxUs_, us);
      zmon_->update(obs_); // obs_ is still the observation the network just saw
    }

    if(activeTicks_ % every == 0)
    {
      const auto & d = decoder_->current();
      const size_t nOff = zmon_->available() ? zmon_->offenders().size() : size_t(0);
      mc_rtc::log::info(
          "[ismpc_policy] active | latches {} | applied: walk {} offset {:.3f} freq {:.3f} sin {:.4f} cos {:.4f} "
          "ts {:.3f} twist {} | elements |z| > {:.0f}: {} | inference avg {:.1f} us max {:.1f} us",
          latchCount_, d.walk ? 1 : 0, d.offset, d.frequency, d.sin_amp, d.cos_amp, d.ts, fmtN(d.twist.data(), 3),
          ZScoreMonitor::kWarn, nOff,
          latchCount_ > 0 ? inferSumUs_ / static_cast<double>(latchCount_) : 0.0, inferMaxUs_);
    }
  }
  catch(const std::exception & e)
  {
    release(std::string("exception in the Active loop: ") + e.what(), true);
  }
  catch(...)
  {
    release("unknown exception in the Active loop", true);
  }
}

bool PolicyRunner::setActive(bool on)
{
  if(on)
  {
    if(state_ != PolicyState::Ready)
    {
      mc_rtc::log::warning("[ismpc_policy] cannot activate: the state is {}, it must be Ready", toString(state_));
      return false;
    }
    if(!sink_)
    {
      mc_rtc::log::warning("[ismpc_policy] cannot activate: no controller to drive");
      return false;
    }
    request_ = 1;
    return true;
  }
  if(state_ != PolicyState::Active) { return false; }
  request_ = 2;
  return true;
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
