#include "move1.h"

#include <vector>
#include "face.h"

namespace move {

  const int COUNT = 48;

  // TODO: renumber
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
  const int RUD = 45;
  const int RRL = 46;
  const int RFB = 47;

  const int moves1[] = {
    U1, U2, U3, D1, D2, D3, U1D1, U1D2, U1D3, U2D1, U2D2, U2D3, U3D1, U3D2, U3D3,
    R1, R2, R3, L1, L2, L3, R1L1, R1L2, R1L3, R2L1, R2L2, R2L3, R3L1, R3L2, R3L3,
    F1, F2, F3, B1, B2, B3, F1B1, F1B2, F1B3, F2B1, F2B2, F2B3, F3B1, F3B2, F3B3,
    RUD, RRL, RFB
  };
  const int moves2[] = {
    U1, U2, U3, D1, D2, D3, U1D1, U1D2, U1D3, U2D1, U2D2, U2D3, U3D1, U3D2, U3D3,
    R2, L2, R2L2, F2, B2, F2B2,
    RUD, RRL, RFB
  };

  const int inv[] = {
    U3, U2, U1, D3, D2, D1, U3D3, U3D2, U3D1, U2D3, U2D2, U2D1, U1D3, U1D2, U1D1,
    R3, R2, R1, L3, L2, L1, R3L3, R3L2, R3L1, R2L3, R2L2, R2L1, R1L3, R1L2, R1L1,
    F3, F2, F1, B3, B2, B1, F3B3, F3B2, F3B1, F2B3, F2B2, F2B1, F1B3, F1B2, F1B1,
    -1, -1, -1
  };

  const std::vector<int> blocked[] = {
    {U1, U2, U3, D1, D2, D3, U1D1, U1D2, U1D3, U2D1, U2D2, U2D3, U3D1, U3D2, U3D3},
    {U1, U2, U3, D1, D2, D3, U1D1, U1D2, U1D3, U2D1, U2D2, U2D3, U3D1, U3D2, U3D3},
    {U1, U2, U3, D1, D2, D3, U1D1, U1D2, U1D3, U2D1, U2D2, U2D3, U3D1, U3D2, U3D3},
    {U1, U2, U3, D1, D2, D3, U1D1, U1D2, U1D3, U2D1, U2D2, U2D3, U3D1, U3D2, U3D3},
    {U1, U2, U3, D1, D2, D3, U1D1, U1D2, U1D3, U2D1, U2D2, U2D3, U3D1, U3D2, U3D3},
    {U1, U2, U3, D1, D2, D3, U1D1, U1D2, U1D3, U2D1, U2D2, U2D3, U3D1, U3D2, U3D3},
    {U1, U2, U3, D1, D2, D3, U1D1, U1D2, U1D3, U2D1, U2D2, U2D3, U3D1, U3D2, U3D3},
    {U1, U2, U3, D1, D2, D3, U1D1, U1D2, U1D3, U2D1, U2D2, U2D3, U3D1, U3D2, U3D3},
    {U1, U2, U3, D1, D2, D3, U1D1, U1D2, U1D3, U2D1, U2D2, U2D3, U3D1, U3D2, U3D3},
    {U1, U2, U3, D1, D2, D3, U1D1, U1D2, U1D3, U2D1, U2D2, U2D3, U3D1, U3D2, U3D3},
    {U1, U2, U3, D1, D2, D3, U1D1, U1D2, U1D3, U2D1, U2D2, U2D3, U3D1, U3D2, U3D3},
    {U1, U2, U3, D1, D2, D3, U1D1, U1D2, U1D3, U2D1, U2D2, U2D3, U3D1, U3D2, U3D3},
    {U1, U2, U3, D1, D2, D3, U1D1, U1D2, U1D3, U2D1, U2D2, U2D3, U3D1, U3D2, U3D3},
    {U1, U2, U3, D1, D2, D3, U1D1, U1D2, U1D3, U2D1, U2D2, U2D3, U3D1, U3D2, U3D3},
    {U1, U2, U3, D1, D2, D3, U1D1, U1D2, U1D3, U2D1, U2D2, U2D3, U3D1, U3D2, U3D3},
    {R1, R2, R3, L1, L2, L3, R1L1, R1L2, R1L3, R2L1, R2L2, R2L3, R3L1, R3L2, R3L3},
    {R1, R2, R3, L1, L2, L3, R1L1, R1L2, R1L3, R2L1, R2L2, R2L3, R3L1, R3L2, R3L3},
    {R1, R2, R3, L1, L2, L3, R1L1, R1L2, R1L3, R2L1, R2L2, R2L3, R3L1, R3L2, R3L3},
    {R1, R2, R3, L1, L2, L3, R1L1, R1L2, R1L3, R2L1, R2L2, R2L3, R3L1, R3L2, R3L3},
    {R1, R2, R3, L1, L2, L3, R1L1, R1L2, R1L3, R2L1, R2L2, R2L3, R3L1, R3L2, R3L3},
    {R1, R2, R3, L1, L2, L3, R1L1, R1L2, R1L3, R2L1, R2L2, R2L3, R3L1, R3L2, R3L3},
    {R1, R2, R3, L1, L2, L3, R1L1, R1L2, R1L3, R2L1, R2L2, R2L3, R3L1, R3L2, R3L3},
    {R1, R2, R3, L1, L2, L3, R1L1, R1L2, R1L3, R2L1, R2L2, R2L3, R3L1, R3L2, R3L3, U2D2},
    {R1, R2, R3, L1, L2, L3, R1L1, R1L2, R1L3, R2L1, R2L2, R2L3, R3L1, R3L2, R3L3},
    {R1, R2, R3, L1, L2, L3, R1L1, R1L2, R1L3, R2L1, R2L2, R2L3, R3L1, R3L2, R3L3},
    {R1, R2, R3, L1, L2, L3, R1L1, R1L2, R1L3, R2L1, R2L2, R2L3, R3L1, R3L2, R3L3},
    {R1, R2, R3, L1, L2, L3, R1L1, R1L2, R1L3, R2L1, R2L2, R2L3, R3L1, R3L2, R3L3},
    {R1, R2, R3, L1, L2, L3, R1L1, R1L2, R1L3, R2L1, R2L2, R2L3, R3L1, R3L2, R3L3},
    {R1, R2, R3, L1, L2, L3, R1L1, R1L2, R1L3, R2L1, R2L2, R2L3, R3L1, R3L2, R3L3},
    {R1, R2, R3, L1, L2, L3, R1L1, R1L2, R1L3, R2L1, R2L2, R2L3, R3L1, R3L2, R3L3},
    {F1, F2, F3, B1, B2, B3, F1B1, F1B2, F1B3, F2B1, F2B2, F2B3, F3B1, F3B2, F3B3},
    {F1, F2, F3, B1, B2, B3, F1B1, F1B2, F1B3, F2B1, F2B2, F2B3, F3B1, F3B2, F3B3},
    {F1, F2, F3, B1, B2, B3, F1B1, F1B2, F1B3, F2B1, F2B2, F2B3, F3B1, F3B2, F3B3},
    {F1, F2, F3, B1, B2, B3, F1B1, F1B2, F1B3, F2B1, F2B2, F2B3, F3B1, F3B2, F3B3},
    {F1, F2, F3, B1, B2, B3, F1B1, F1B2, F1B3, F2B1, F2B2, F2B3, F3B1, F3B2, F3B3},
    {F1, F2, F3, B1, B2, B3, F1B1, F1B2, F1B3, F2B1, F2B2, F2B3, F3B1, F3B2, F3B3},
    {F1, F2, F3, B1, B2, B3, F1B1, F1B2, F1B3, F2B1, F2B2, F2B3, F3B1, F3B2, F3B3},
    {F1, F2, F3, B1, B2, B3, F1B1, F1B2, F1B3, F2B1, F2B2, F2B3, F3B1, F3B2, F3B3, U2D2, R2L2},
    {F1, F2, F3, B1, B2, B3, F1B1, F1B2, F1B3, F2B1, F2B2, F2B3, F3B1, F3B2, F3B3},
    {F1, F2, F3, B1, B2, B3, F1B1, F1B2, F1B3, F2B1, F2B2, F2B3, F3B1, F3B2, F3B3},
    {F1, F2, F3, B1, B2, B3, F1B1, F1B2, F1B3, F2B1, F2B2, F2B3, F3B1, F3B2, F3B3},
    {F1, F2, F3, B1, B2, B3, F1B1, F1B2, F1B3, F2B1, F2B2, F2B3, F3B1, F3B2, F3B3},
    {F1, F2, F3, B1, B2, B3, F1B1, F1B2, F1B3, F2B1, F2B2, F2B3, F3B1, F3B2, F3B3},
    {F1, F2, F3, B1, B2, B3, F1B1, F1B2, F1B3, F2B1, F2B2, F2B3, F3B1, F3B2, F3B3},
    {F1, F2, F3, B1, B2, B3, F1B1, F1B2, F1B3, F2B1, F2B2, F2B3, F3B1, F3B2, F3B3},
    {RUD, RRL, RFB},
    {RUD, RRL, RFB},
    {RUD, RRL, RFB},
  };

}
