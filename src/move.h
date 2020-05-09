#ifndef __MOVE__
#define __MOVE__

#include <string>
#include "cubie.h"
#include "face.h"

namespace move {

  using namespace face::color;

  using mask = uint64_t;

  const int COUNT_CUBE = 45;
  const int COUNT_TILT = 2;
  const int COUNT_GRIP = 1;
  const int COUNT = COUNT_CUBE + COUNT_TILT + COUNT_GRIP;

  const int U1 = 0;
  const int U2 = 1;
  const int U3 = 2;
  const int D1 = 3;
  const int D2 = 4;
  const int D3 = 5;
  const int U1D1 = 6;
  const int U1D2 = 7;
  const int U1D3 = 8;
  const int U2D1 = 9;
  const int U2D2 = 10;
  const int U2D3 = 11;
  const int U3D1 = 12;
  const int U3D2 = 13;
  const int U3D3 = 14;
  const int R1 = 15;
  const int R2 = 16;
  const int R3 = 17;
  const int L1 = 18;
  const int L2 = 19;
  const int L3 = 20;
  const int R1L1 = 21;
  const int R1L2 = 22;
  const int R1L3 = 23;
  const int R2L1 = 24;
  const int R2L2 = 25;
  const int R2L3 = 26;
  const int R3L1 = 27;
  const int R3L2 = 28;
  const int R3L3 = 29;
  const int F1 = 30;
  const int F2 = 31;
  const int F3 = 32;
  const int B1 = 33;
  const int B2 = 34;
  const int B3 = 35;
  const int F1B1 = 36;
  const int F1B2 = 37;
  const int F1B3 = 38;
  const int F2B1 = 39;
  const int F2B2 = 40;
  const int F2B3 = 41;
  const int F3B1 = 42;
  const int F3B2 = 43;
  const int F3B3 = 44;
  const int TRL = 45;
  const int TFB = 46;
  const int G = 47;

  extern cubie::cube cubes[COUNT_CUBE];

  const std::string names[] = {
    "U", "U2", "U'", "D", "D2", "D'",
    "(U D)", "(U D2)", "(U D')", "(U2 D)", "(U2 D2)", "(U2 D')", "(U' D)", "(U' D2)", "(U' D')",
    "R", "R2", "R'", "L", "L2", "L'",
    "(R L)", "(R L2)", "(R L')", "(R2 L)", "(R2 L2)", "(R2 L')", "(R' L)", "(R' L2)", "(R' L')",
    "F", "F2", "F'", "B", "B2", "B'",
    "(F B)", "(F B2)", "(F B')", "(F2 B)", "(F2 B2)", "(F2 B')", "(F' B)", "(F' B2)", "(F' B')",
    "tRL", "tFB", "g"
  };

  const int inv[] = {
    U3, U2, U1, D3, D2, D1, U3D3, U3D2, U3D1, U2D3, U2D2, U2D1, U1D3, U1D2, U1D1,
    R3, R2, R1, L3, L2, L1, R3L3, R3L2, R3L1, R2L3, R2L2, R2L1, R1L3, R1L2, R1L1,
    F3, F2, F1, B3, B2, B1, F3B3, F3B2, F3B1, F2B3, F2B2, F2B1, F1B3, F1B2, F1B1,
    -1, -1, -1 // we don't need inverses for rotations
  };

  extern mask next[COUNT]; // successor moves that should be explored

  extern mask p1mask; // phase 1 moves
  extern mask p2mask; // phase 2 moves

  int translate(int m, const int fperm[6]);

  inline mask bit(int m) {
    return mask(1) << m;
  }
  inline bool in(int m, mask mm) {
    return mm & bit(m);
  }

  void init();

}

#endif
