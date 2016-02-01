#ifndef INCLUDE_WORLD_H_
#define INCLUDE_WORLD_H_

#include "json/json.h"

void BuildWorld(chrono::ChSystem *ch_system, const Json::Value &params);

#endif // INCLUDE_WORLD_H_
