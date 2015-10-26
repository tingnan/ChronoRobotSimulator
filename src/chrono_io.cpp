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

void SerializeBodies(std::vector<ChBody *> &body_list,
                     std::ofstream &mov_file) {
  mov_file << std::setprecision(8);
  mov_file << std::scientific;
  for (size_t i = 0; i < body_list.size(); ++i) {
    auto cur_body = body_list[i];
    mov_file << cur_body->GetIdentifier() << " ";
    mov_file << cur_body->GetPos() << " " << cur_body->GetPos_dt() << " ";
    auto rotquat = cur_body->GetRot();
    // needs to conjugate to satisfy the matlab convention
    rotquat.Conjugate();
    mov_file << rotquat << " ";
    // now output the angular velocity
    mov_file << cur_body->GetWvel_par() << "\n";
  }
  mov_file.flush();
}

void SerializeEngines(std::vector<chrono::ChLinkEngine *> &engine_list,
                      std::ofstream &jnt_file) {
  jnt_file << std::setprecision(8);
  jnt_file << std::scientific;
  for (size_t i = 0; i < engine_list.size(); ++i) {
    auto link_coordsys = engine_list[i]->GetLinkRelativeCoords();
    jnt_file << engine_list[i]->GetIdentifier() << " ";
    auto localforce = link_coordsys.TransformDirectionLocalToParent(
        engine_list[i]->Get_react_force());
    auto localtorque = link_coordsys.TransformDirectionLocalToParent(
        engine_list[i]->Get_mot_torque());
    jnt_file << localforce << " " << localtorque << "\n";
  }
  jnt_file.flush();
}

void SerializeConstraints(std::vector<chrono::ChLink *> &constraint_list,
                          std::ofstream &cst_file) {
  cst_file << std::setprecision(8);
  cst_file << std::scientific;
  for (size_t i = 0; i < constraint_list.size(); ++i) {
    cst_file << constraint_list[i]->GetIdentifier() << " ";
    auto link_coordsys = constraint_list[i]->GetLinkRelativeCoords();
    auto localforce = link_coordsys.TransformDirectionLocalToParent(
        constraint_list[i]->Get_react_force());
    auto localtorque = link_coordsys.TransformDirectionLocalToParent(
        constraint_list[i]->Get_react_torque());
    cst_file << localforce << " " << localtorque << "\n";
  }
  cst_file.flush();
}

void SerializeRFTForce(std::vector<RFTBody> &rft_body_list,
                       std::ofstream &rft_file) {
  const size_t nnodes = rft_body_list.size();
  for (int i = 0; i < nnodes; ++i) {
    rft_file << rft_body_list[i].chbody->GetIdentifier() << " ";
    ChVector<> total_force;
    for (size_t j = rft_body_list[i].forces.size(); j != 0; --j) {
      total_force += rft_body_list[i].forces[j - 1];
    }
    rft_file << total_force << std::endl;
  }
  rft_file.flush();
}
