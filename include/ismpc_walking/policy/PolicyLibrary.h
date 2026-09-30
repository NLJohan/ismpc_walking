#pragma once

#include <string>
#include <vector>

namespace ismpc_walking::policy
{

/**
 * Lists the *.onnx files of the policies directory (file names only, natural order:
 * model_9 before model_50 before model_100). Used for the GUI dropdown.
 * Never throws: an unreadable directory just gives an empty list.
 */
class PolicyLibrary
{
public:
  explicit PolicyLibrary(std::string dir);

  /** Re-scans the directory. */
  void refresh();

  const std::string & dir() const noexcept { return dir_; }
  const std::vector<std::string> & files() const noexcept { return files_; }

private:
  std::string dir_;
  std::vector<std::string> files_;
};

} // namespace ismpc_walking::policy
