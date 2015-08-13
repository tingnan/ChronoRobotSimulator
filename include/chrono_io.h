#ifndef INCLUDE_CHRONOIO_H_
#define INCLUDE_CHRONOIO_H_

#include <vector>
#include <string>
#include <fstream>

#include "include/rft.h"

namespace chrono {
class ChSystem;
} // namespace chrono

class IOManager {
public:
  IOManager(chrono::ChSystem *ch_system, std::string fname_prefix)
      : ch_system_(ch_system) {
    std::string mov_fname = fname_prefix + ".mov";
    mov_file_.open(mov_fname.c_str());
    std::string jnt_fname = fname_prefix + ".jnt";
    jnt_file_.open(jnt_fname.c_str());
  }

  void DumpNodInfo();
  void DumpJntInfo();
  void DumpContact();

private:
  chrono::ChSystem *ch_system_;
  std::ofstream mov_file_;
  std::ofstream rft_file_;
  std::ofstream jnt_file_;
};

void DumpRFTInfo(std::vector<RFTBody> &rft_body_list, std::ofstream &rft_file);

#endif // INCLUDE_CHRONOIO_H_
