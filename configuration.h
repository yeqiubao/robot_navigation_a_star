#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

#include <iostream>

// six movement direction
enum Movement {
    FL = 0,
    FR,
    FS,
    BL,
    BR,
    BS
};

// map size
constexpr int N  = 7;
constexpr int M = 14;
// number of direction
constexpr int Z = 4;
// number of movement direction
constexpr int k = 6;

#endif  // CONFIGURATION_H_
