#ifndef MAPPING_UTILS_H
#define MAPPING_UTILS_H

#include <vector>
#include <utility>
#include <cstdlib>


#define SWAP(a, b)  {a ^= b; b ^= a; a ^= b;}


std::vector<std::pair<int, int>> bresenhamLine(int x0, int y0, int x1, int y1);

#endif