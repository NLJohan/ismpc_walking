// TEMPORARY dev scaffolding for the step 5 parity test. Removed in step 9.
#include "ismpc_walking/policy/ActionDecoder.h"

#include <cstdio>
#include <fstream>
#include <sstream>
#include <vector>

namespace ismpc_walking::policy
{

bool runDecoderParity(const PolicyContract & contract, const std::string & in_path, const std::string & out_path,
                      std::string & err)
{
  std::ifstream in(in_path);
  if(!in)
  {
    err = "cannot open " + in_path;
    return false;
  }
  std::vector<ActionDecoder::Raw> rows;
  std::string line;
  while(std::getline(in, line))
  {
    if(line.empty() || line[0] == '#') { continue; }
    std::stringstream ss(line);
    std::string cell;
    ActionDecoder::Raw raw{};
    size_t i = 0;
    while(std::getline(ss, cell, ',') && i < raw.size())
    {
      try
      {
        raw[i++] = std::stod(cell);
      }
      catch(...)
      {
        err = "bad number '" + cell + "' in " + in_path;
        return false;
      }
    }
    if(i != raw.size())
    {
      err = "expected " + std::to_string(raw.size()) + " values per line in " + in_path;
      return false;
    }
    rows.push_back(raw);
  }

  std::ofstream out(out_path);
  if(!out)
  {
    err = "cannot write " + out_path;
    return false;
  }
  out << "latch,tick,offset,frequency,sin_amp,cos_amp,walk,ts,vx,vy,omega\n";

  ActionDecoder decoder(contract);
  decoder.reset();
  long tick = 0;
  char buf[512];
  for(size_t n = 0; n < rows.size(); ++n)
  {
    do
    {
      ++tick;
    } while(!decoder.advance());
    if(!decoder.latch(rows[n]))
    {
      err = "non finite raw value in row " + std::to_string(n);
      return false;
    }
    const DecodedAction & d = decoder.current();
    std::snprintf(buf, sizeof(buf), "%zu,%ld,%.17g,%.17g,%.17g,%.17g,%d,%.17g,%.17g,%.17g,%.17g\n", n, tick, d.offset,
                  d.frequency, d.sin_amp, d.cos_amp, d.walk ? 1 : 0, d.ts, d.twist[0], d.twist[1], d.twist[2]);
    out << buf;
  }
  return true;
}

} // namespace ismpc_walking::policy
