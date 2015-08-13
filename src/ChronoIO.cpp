#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <string>
#include <map>
#include <vector>
#include "include/VectorUtility.h"
#include "include/ChronoIO.h"
#include "include/RFT.h"
#include "physics/ChSystem.h"

using namespace chrono;

// dump pos vel acc, rot, ome, and rot_acc for the body
std::ofstream nodinfofile("snake.mov");
void ChronoIOManager::DumpNodInfo() {
  nodinfofile << std::setprecision(8);
  nodinfofile << std::scientific;
  const size_t nnodes = mChSys->Get_bodylist()->size();
  for (unsigned int i = 0; i < nnodes; ++i) {
    ChBody *curbody = (*mChSys->Get_bodylist())[i];
    if (curbody->GetIdentifier() == -1)
      continue;
    nodinfofile << curbody->GetIdentifier() << " ";
    nodinfofile << curbody->GetPos() << " " << curbody->GetPos_dt() << " ";
    ChQuaternion<> rotquat = curbody->GetRot();
    // needs to conjugate to satisfy the matlab convention
    rotquat.Conjugate();
    nodinfofile << rotquat << " ";
    // now output the angular velocity
    nodinfofile << curbody->GetWvel_par() << "\n";
  }
  nodinfofile.flush();
}

// dump pos vel acc, rot, ome, and rot_acc for the link
std::ofstream jntinfofile("snake.jnt");
void ChronoIOManager::DumpJntInfo() {
  jntinfofile << std::setprecision(8);
  jntinfofile << std::scientific;
  std::vector<ChLink *>::iterator itr;
  for (itr = mChSys->Get_linklist()->begin();
       itr != mChSys->Get_linklist()->end(); ++itr) {
    ChVector<> localforce = (*itr)->Get_react_force();
    double localtorque = ((chrono::ChLinkEngine *)*itr)->Get_mot_torque();
    jntinfofile << (*itr)->GetIdentifier() << " ";
    jntinfofile << localforce << " " << localtorque << "\n";
    // jntinfofile << (*itr)->GetLinkRelativeCoords().rot.Rotate(localforce) <<
    // " " << (*itr)->GetLinkRelativeCoords().rot.Rotate(localtorque) << "\n";
  }
  jntinfofile.flush();
}

std::ofstream cotinfofile("snake.cot");
void ChronoIOManager::DumpContact() {
  /*
  // collect all the contact forces on each body
  std::map<ChBody *, ChVector<> > forcemap;
  std::map<ChBody *, ChVector<> > torquemap;
  ChContactContainerBase *syscot =
      (ChContactContainerBase *)(mChSys->GetContactContainer());
  std::list<ChContact *> contactlist = syscot->GetContactList();
  std::list<ChContact *>::const_iterator itr = contactlist.begin();
  for (; itr != contactlist.end(); ++itr) {
    collision::ChModelBulletBody *modelA =
        (collision::ChModelBulletBody *)((*itr)->GetModelA());
    collision::ChModelBulletBody *modelB =
        (collision::ChModelBulletBody *)((*itr)->GetModelB());
    ChBody *bodyA = modelA->GetBody();
    ChBody *bodyB = modelB->GetBody();
    ChVector<> curforce =
        (*itr)->GetContactCoords().TransformDirectionLocalToParent(
            (*itr)->GetContactForce());
    forcemap[bodyA] += curforce;
    forcemap[bodyB] -= curforce;
    ChVector<> pointA = (*itr)->GetContactP1();
    ChVector<> pointB = (*itr)->GetContactP2();
    // torque in global frame;
    torquemap[bodyA] += (pointA - bodyA->GetPos()) % curforce;
    torquemap[bodyB] -= (pointB - bodyB->GetPos()) % curforce;
  }

  // now let us save the contact force
  cotinfofile << std::setprecision(8);
  cotinfofile << std::scientific;
  std::map<ChBody *, ChVector<> >::const_iterator citr = forcemap.begin();
  for (; citr != forcemap.end(); ++citr) {
    cotinfofile << citr->first->GetIdentifier() << " ";
    cotinfofile << citr->second << "\n";
  }
  */
}

std::ofstream rftinfofile("snake.rft");
void ChronoIOManager::DumpRFTInfo() {
  const size_t nnodes = mRFTBodyList->size();
  for (int i = 0; i < nnodes; ++i) {
    rftinfofile << (*mRFTBodyList)[i].GetChBody()->GetIdentifier() << " ";
    rftinfofile << (*mRFTBodyList)[i].flist_ << "\n";
  }
}
