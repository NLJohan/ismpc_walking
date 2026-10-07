#pragma once
// DEBUG (temporary): brute-force state dump to a plain text file, usable when mc_rtc logs are not visible.
// One file per process: /tmp/ismpc_full_dump_<pid>.txt (truncated at first use).
// Layout: sections delimited by "=== <tag> | count=<n> | t_ms=<ms> | self=<ptr> ===" ... "=== end <tag> ===".
// Thread-safe (recursive mutex held for the lifetime of a Section). No dependency on fmt or Eigen headers:
// the formatter replaces each "{}" in order with the streamed argument. Remove together with the call sites.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <ostream>
#include <sstream>
#include <string>
#include <unistd.h>

namespace ismpc_dump
{

inline std::recursive_mutex & mtx()
{
  static std::recursive_mutex m;
  return m;
}

inline std::ofstream & out()
{
  static std::ofstream f("/tmp/ismpc_full_dump_" + std::to_string(::getpid()) + ".txt",
                         std::ios::out | std::ios::trunc);
  return f;
}

inline double nowMs()
{
  static const auto t0 = std::chrono::steady_clock::now();
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

namespace detail
{
inline void fmtImpl(std::ostream & os, const char * f)
{
  os << f;
}

template<class T, class... R>
void fmtImpl(std::ostream & os, const char * f, const T & v, const R &... r)
{
  while(*f)
  {
    if(f[0] == '{' && f[1] == '}')
    {
      os << v;
      fmtImpl(os, f + 2, r...);
      return;
    }
    os << *f++;
  }
}
} // namespace detail

// Any Eigen-like dense object (needs rows(), cols(), operator()(i,j)): all coefficients on one line.
template<class T>
std::string flat(const T & m)
{
  std::ostringstream o;
  o << std::setprecision(17);
  for(long i = 0; i < static_cast<long>(m.rows()); ++i)
  {
    for(long j = 0; j < static_cast<long>(m.cols()); ++j) { o << m(i, j) << ' '; }
  }
  return o.str();
}

template<class T>
long nonFinite(const T & m)
{
  long bad = 0;
  for(long i = 0; i < static_cast<long>(m.rows()); ++i)
  {
    for(long j = 0; j < static_cast<long>(m.cols()); ++j)
    {
      if(!std::isfinite(static_cast<double>(m(i, j)))) { ++bad; }
    }
  }
  return bad;
}

// std::vector of scalars: size, non-finite count, first maxHead values, last value.
template<class V>
std::string vecs(const V & v, std::size_t maxHead = 8)
{
  std::ostringstream o;
  o << std::setprecision(17);
  const std::size_t n = v.size();
  std::size_t bad = 0;
  for(const auto & x : v)
  {
    if(!std::isfinite(static_cast<double>(x))) { ++bad; }
  }
  o << "n=" << n << " nonfinite=" << bad << " [";
  for(std::size_t i = 0; i < std::min(n, maxHead); ++i) { o << v[i] << ' '; }
  if(n > maxHead) { o << "... " << v[n - 1]; }
  o << "]";
  return o.str();
}

// std::vector of Eigen-like vectors: size, non-finite count, first maxHead elements, last element.
template<class V>
std::string vecE(const V & v, std::size_t maxHead = 4)
{
  std::ostringstream o;
  const std::size_t n = v.size();
  long bad = 0;
  for(const auto & x : v) { bad += nonFinite(x); }
  o << "n=" << n << " nonfinite=" << bad << " [";
  for(std::size_t i = 0; i < std::min(n, maxHead); ++i) { o << "(" << flat(v[i]) << ") "; }
  if(n > maxHead) { o << "... (" << flat(v[n - 1]) << ")"; }
  o << "]";
  return o.str();
}

// sva::PTransformd-like: translation then rotation matrix.
template<class T>
std::string pose(const T & X)
{
  return "t(" + flat(X.translation()) + ") R(" + flat(X.rotation()) + ")";
}

class Section
{
public:
  Section(const std::string & tag, int count, const void * self = nullptr) : lk_(mtx()), tag_(tag)
  {
    auto & o = out();
    o << std::setprecision(17) << "=== " << tag_ << " | count=" << count << " | t_ms=" << nowMs()
      << " | self=" << self << " ===\n";
  }

  ~Section()
  {
    auto & o = out();
    o << "=== end " << tag_ << " ===\n";
    o.flush();
  }

  Section(const Section &) = delete;
  Section & operator=(const Section &) = delete;

  template<class T>
  void kv(const char * k, const T & v)
  {
    out() << k << " = " << v << '\n';
  }

  template<class... A>
  void line(const char * f, const A &... a)
  {
    detail::fmtImpl(out(), f, a...);
    out() << '\n';
  }

private:
  std::lock_guard<std::recursive_mutex> lk_;
  std::string tag_;
};

} // namespace ismpc_dump
