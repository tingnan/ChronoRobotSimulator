# pragma once

namespace chrono
{
    class ChLinkEngine;
}

#include <vector>
class RobotController
{
protected:
    class SnakeControlSet *mSnakeParams;
    std::vector<chrono::ChLinkEngine*> mEngines;
public:
    RobotController();
    ~RobotController() {}
    void AddEngine(chrono::ChLinkEngine* p) { mEngines.push_back(p); }
    void SetControlSet(class SnakeControlSet *p) { mSnakeParams = p; }
    void PositionControl();
    void ActiveLifting();
    void TorqueControl();
};
