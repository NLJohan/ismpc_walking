#include "ismpc_walking/policy/PolicyLibrary.h"

#include <mc_rtc/logging.h>

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <system_error>

namespace ismpc_walking::policy
{

namespace
{

/** "model_9.onnx" < "model_50.onnx" < "model_100.onnx": digit runs compare as numbers. */
bool naturalLess(const std::string & a, const std::string & b)
{
  size_t i = 0, j = 0;
  while(i < a.size() && j < b.size())
  {
    const bool da = std::isdigit(static_cast<unsigned char>(a[i])) != 0;
    const bool db = std::isdigit(static_cast<unsigned char>(b[j])) != 0;
    if(da && db)
    {
      size_t ei = i, ej = j;
      while(ei < a.size() && std::isdigit(static_cast<unsigned char>(a[ei]))) { ++ei; }
      while(ej < b.size() && std::isdigit(static_cast<unsigned char>(b[ej]))) { ++ej; }
      // strip leading zeros, then shorter run = smaller number
      size_t si = i, sj = j;
      while(si + 1 < ei && a[si] == '0') { ++si; }
      while(sj + 1 < ej && b[sj] == '0') { ++sj; }
      const size_t li = ei - si, lj = ej - sj;
      if(li != lj) { return li < lj; }
      const int c = a.compare(si, li, b, sj, lj);
      if(c != 0) { return c < 0; }
      i = ei;
      j = ej;
    }
    else
    {
      if(a[i] != b[j]) { return a[i] < b[j]; }
      ++i;
      ++j;
    }
  }
  return (a.size() - i) < (b.size() - j);
}

} // namespace

PolicyLibrary::PolicyLibrary(std::string dir) : dir_(std::move(dir)) { refresh(); }

void PolicyLibrary::refresh()
{
  files_.clear();
  if(dir_.empty()) { return; }
  namespace fs = std::filesystem;
  std::error_code ec;
  fs::directory_iterator it(dir_, ec);
  if(ec)
  {
    mc_rtc::log::warning("[ismpc_policy] cannot list policies directory '{}': {}", dir_, ec.message());
    return;
  }
  const fs::directory_iterator end;
  for(; it != end; it.increment(ec))
  {
    if(ec) { break; }
    std::error_code ec2;
    if(!it->is_regular_file(ec2) || ec2) { continue; }
    if(it->path().extension() == ".onnx") { files_.push_back(it->path().filename().string()); }
  }
  std::sort(files_.begin(), files_.end(), naturalLess);
}

} // namespace ismpc_walking::policy
