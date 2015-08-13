#pragma once
#include <vector>
namespace chrono
{
    class ChSystem;
}
class RFTBody;

class ChronoIOManager
{
    chrono::ChSystem *mChSys;
    std::vector<RFTBody> *mRFTBodyList;
public:
    ChronoIOManager(chrono::ChSystem *pSys, std::vector<RFTBody> *pbdlist) : mChSys(pSys), mRFTBodyList(pbdlist) {}
    
    void DumpNodInfo();
    void DumpJntInfo();
    void DumpContact();
    void DumpRFTInfo();
};

