#ifndef INCLUDE_CHRONOIO_H_
#define INCLUDE_CHRONOIO_H_

#include <vector>
#include <string>
#include <fstream>

#include "include/rft.h"

namespace chrono {
class ChSystem;
class ChBody;
class ChLinkEngine;
} // namespace chrono

void SerializeBodies(std::vector<chrono::ChBody *> &body_list,
                     std::ofstream &mov_file);
void SerializeEngines(std::vector<chrono::ChLinkEngine *> &engine_list,
                      std::ofstream &jnt_file);
void SerializeRFTForce(std::vector<RFTBody> &rft_body_list,
                       std::ofstream &rft_file);

#endif // INCLUDE_CHRONOIO_H_
