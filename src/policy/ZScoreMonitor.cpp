#include "ismpc_walking/policy/ZScoreMonitor.h"

#include <algorithm>
#include <cmath>

namespace ismpc_walking::policy
{

ZScoreMonitor::ZScoreMonitor(const PolicyContract & contract, const std::vector<ObservationBuilder::Term> & terms)
: terms_(terms), joints_(contract.joint_names)
{
  running_.assign(terms_.size(), TermMax{});
  offenders_.reserve(static_cast<size_t>(std::max(contract.obs_dim, 0)));
  if(contract.normalizer && static_cast<int>(contract.normalizer->mean.size()) == contract.obs_dim
     && static_cast<int>(contract.normalizer->std.size()) == contract.obs_dim)
  {
    mean_ = contract.normalizer->mean;
    std_ = contract.normalizer->std;
    eps_ = contract.normalizer->eps;
    available_ = true;
  }
}

void ZScoreMonitor::update(const std::vector<double> & obs) noexcept
{
  offenders_.clear();
  if(!available_ || obs.size() != mean_.size()) { return; }
  for(size_t ti = 0; ti < terms_.size(); ++ti)
  {
    const auto & t = terms_[ti];
    for(int k = 0; k < t.dim; ++k)
    {
      const size_t i = static_cast<size_t>(t.offset + k);
      const double z = std::fabs((obs[i] - mean_[i]) / (std_[i] + eps_));
      if(z > running_[ti].z)
      {
        running_[ti].z = z;
        running_[ti].elem = k;
      }
      if(z > kWarn) { offenders_.push_back({z, i}); }
    }
  }
  std::sort(offenders_.begin(), offenders_.end(), [](const Offender & a, const Offender & b) { return a.z > b.z; });
}

void ZScoreMonitor::resetRunning() noexcept { running_.assign(terms_.size(), TermMax{}); }

std::string ZScoreMonitor::termElementName(size_t term, int elem) const
{
  if(term >= terms_.size()) { return "?"; }
  const auto & t = terms_[term];
  if(t.id == ObservationBuilder::TermId::JointPos || t.id == ObservationBuilder::TermId::JointVel)
  {
    return t.name + "[" + (static_cast<size_t>(elem) < joints_.size() ? joints_[static_cast<size_t>(elem)] : "?") + "]";
  }
  return t.dim > 1 ? t.name + "[" + std::to_string(elem) + "]" : t.name;
}

std::string ZScoreMonitor::elementName(size_t index) const
{
  for(size_t ti = 0; ti < terms_.size(); ++ti)
  {
    const auto & t = terms_[ti];
    if(index >= static_cast<size_t>(t.offset) && index < static_cast<size_t>(t.offset + t.dim))
    {
      return termElementName(ti, static_cast<int>(index) - t.offset);
    }
  }
  return "?";
}

} // namespace ismpc_walking::policy
