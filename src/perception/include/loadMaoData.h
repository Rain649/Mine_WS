#pragma once

// std
#include <iostream>
#include <stdio.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <string>
#include <omp.h>

// STL
#include <unordered_map>
#include <vector>

// topoMap Class
#include "topoMap.h"

// now the extraction operators for these types
void operator>>(const YAML::Node &node, perception::LinkedInfo &lI){};

void operator>>(const YAML::Node &node, perception::Vertex &v){};

void operator>>(const YAML::Node &node, perception::Edge &e){};
// end

TopoMap loadMap(const std::string &vertexFilePath, const std::string &edgeFilePath){};