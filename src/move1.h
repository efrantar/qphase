#ifndef __MOVE__
#define __MOVE__

#include <string>
#include "cubie.h"

namespace move {

  struct info {
    int ax;

    int f1;
    int f2;
    int pow1;
    int pow2;

    bool axial;
    bool tilt;

    std::string name;
    cubie::cube cube;
  };

}

#endif
