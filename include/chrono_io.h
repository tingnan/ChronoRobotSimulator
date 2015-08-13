#ifndef INCLUDE_CHRONOIO_H_
#define INCLUDE_CHRONOIO_H_

#include <vector>

namespace chrono {
class ChSystem;
} // namespace chrono

class RFTBody;

class ChronoIOManager {
public:
  ChronoIOManager(chrono::ChSystem *pSys, std::vector<RFTBody> *pbdlist)
      : ch_system_(pSys), body_list_(pbdlist) {}

  void DumpNodInfo();
  void DumpJntInfo();
  void DumpContact();
  void DumpRFTInfo();

private:
  chrono::ChSystem *ch_system_;
  std::vector<RFTBody> *body_list_;
};

#endif // INCLUDE_CHRONOIO_H_
