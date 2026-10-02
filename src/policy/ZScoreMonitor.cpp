#include "ismpc_walking/policy/ZScoreMonitor.h"

#include <cmath>

namespace ismpc_walking::policy
{

ZScoreMonitor::ZScoreMonitor(const PolicyContract & contract)
{
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
  offenders_ = 0;
  if(!available_ || obs.size() != mean_.size()) { return; }
  for(size_t i = 0; i < obs.size(); ++i)
  {
    const double z = std::fabs((obs[i] - mean_[i]) / (std_[i] + eps_));
    if(z > kWarn) { ++offenders_; }
  }
}

} // namespace ismpc_walking::policy
