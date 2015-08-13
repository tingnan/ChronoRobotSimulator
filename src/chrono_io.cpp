#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <string>
#include <map>
#include <vector>

#include <physics/ChSystem.h>

#include "include/vector_utility.h"
#include "include/chrono_io.h"

using namespace chrono;

// dump pos vel acc, rot, ome, and rot_acc for the body
void IOManager::DumpNodInfo() {
  mov_file_ << std::setprecision(8);
  mov_file_ << std::scientific;
  const size_t nnodes = ch_system_->Get_bodylist()->size();
  for (unsigned int i = 0; i < nnodes; ++i) {
    ChBody *curbody = (*ch_system_->Get_bodylist())[i];
    if (curbody->GetIdentifier() == -1)
      continue;
    mov_file_ << curbody->GetIdentifier() << " ";
    mov_file_ << curbody->GetPos() << " " << curbody->GetPos_dt() << " ";
    ChQuaternion<> rotquat = curbody->GetRot();
    // needs to conjugate to satisfy the matlab convention
    rotquat.Conjugate();
    mov_file_ << rotquat << " ";
    // now output the angular velocity
    mov_file_ << curbody->GetWvel_par() << "\n";
  }
  mov_file_.flush();
}

// dump pos vel acc, rot, ome, and rot_acc for the link
void IOManager::DumpJntInfo() {
  jnt_file_ << std::setprecision(8);
  jnt_file_ << std::scientific;
  std::vector<ChLink *>::iterator itr;
  for (itr = ch_system_->Get_linklist()->begin();
       itr != ch_system_->Get_linklist()->end(); ++itr) {
    ChVector<> localforce = (*itr)->Get_react_force();
    double localtorque = ((chrono::ChLinkEngine *)*itr)->Get_mot_torque();
    jnt_file_ << (*itr)->GetIdentifier() << " ";
    jnt_file_ << localforce << " " << localtorque << "\n";
  }
  jnt_file_.flush();
}

void IOManager::DumpContact() {
  /*
  // collect all the contact forces on each body
  std::map<ChBody *, ChVector<> > forcemap;
  std::map<ChBody *, ChVector<> > torquemap;
  ChContactContainerBase *syscot =
      (ChContactContainerBase *)(ch_system_->GetContactContainer());
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

void DumpRFTInfo(std::vector<RFTBody> &rft_body_list, std::ofstream &rft_file) {
  const size_t nnodes = rft_body_list.size();
  for (int i = 0; i < nnodes; ++i) {
    rft_file << rft_body_list[i].GetChBody()->GetIdentifier() << " ";
    rft_file << rft_body_list[i].flist_ << "\n";
  }
  rft_file.flush();
}
