#pragma once

#include <string>

#include "include/graph.h"
#include "include/edge.h"

class ExtractData {
  public:
    Graph extract(std::string filename);
};