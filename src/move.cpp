#include "move.h"

#include <vector>
#include "face.h"

namespace move {

  using namespace cubie::corner;
  using namespace cubie::edge;

  mask next[COUNT];

  mask p1mask;
  mask p2mask;

  cubie::cube cubes[COUNT_CUBE];

  const int moves1[] = {
    U1, U2, U3, D1, D2, D3, U1D1, U1D2, U1D3, U2D1, U2D2, U2D3, U3D1, U3D2, U3D3,
    R1, R2, R3, L1, L2, L3, R1L1, R1L2, R1L3, R2L1, R2L2, R2L3, R3L1, R3L2, R3L3,
    F1, F2, F3, B1, B2, B3, F1B1, F1B2, F1B3, F2B1, F2B2, F2B3, F3B1, F3B2, F3B3,
    TRL, TFB
  };
  const int moves2[] = {
    U1, U2, U3, D1, D2, D3, U1D1, U1D2, U1D3, U2D1, U2D2, U2D3, U3D1, U3D2, U3D3,
    R2, L2, R2L2, F2, B2, F2B2,
    TRL, TFB
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
    {R1, R2, R3, L1, L2, L3, R1L1, R1L2, R1L3, R2L1, R2L2, R2L3, R3L1, R3L2, R3L3},
    {R1, R2, R3, L1, L2, L3, R1L1, R1L2, R1L3, R2L1, R2L2, R2L3, R3L1, R3L2, R3L3},
    {R1, R2, R3, L1, L2, L3, R1L1, R1L2, R1L3, R2L1, R2L2, R2L3, R3L1, R3L2, R3L3},
    {R1, R2, R3, L1, L2, L3, R1L1, R1L2, R1L3, R2L1, R2L2, R2L3, R3L1, R3L2, R3L3, U2D2},
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
    {F1, F2, F3, B1, B2, B3, F1B1, F1B2, F1B3, F2B1, F2B2, F2B3, F3B1, F3B2, F3B3},
    {F1, F2, F3, B1, B2, B3, F1B1, F1B2, F1B3, F2B1, F2B2, F2B3, F3B1, F3B2, F3B3},
    {F1, F2, F3, B1, B2, B3, F1B1, F1B2, F1B3, F2B1, F2B2, F2B3, F3B1, F3B2, F3B3},
    {F1, F2, F3, B1, B2, B3, F1B1, F1B2, F1B3, F2B1, F2B2, F2B3, F3B1, F3B2, F3B3, U2D2, R2L2},
    {F1, F2, F3, B1, B2, B3, F1B1, F1B2, F1B3, F2B1, F2B2, F2B3, F3B1, F3B2, F3B3},
    {F1, F2, F3, B1, B2, B3, F1B1, F1B2, F1B3, F2B1, F2B2, F2B3, F3B1, F3B2, F3B3},
    {F1, F2, F3, B1, B2, B3, F1B1, F1B2, F1B3, F2B1, F2B2, F2B3, F3B1, F3B2, F3B3},
    {F1, F2, F3, B1, B2, B3, F1B1, F1B2, F1B3, F2B1, F2B2, F2B3, F3B1, F3B2, F3B3},
    {R1, R2, R3, L1, L2, L3, R1L1, R1L2, R1L3, R2L1, R2L2, R2L3, R3L1, R3L2, R3L3, TRL, TFB},
    {F1, F2, F3, B1, B2, B3, F1B1, F1B2, F1B3, F2B1, F2B2, F2B3, F3B1, F3B2, F3B3, TRL, TFB}
  };

  void init() {
    cubie::cube fcubes[] = {
      { // U
        {UBR, URF, UFL, ULB, DFR, DLF, DBL, DRB},
        {UB, UR, UF, UL, DR, DF, DL, DB, FR, FL, BL, BR},
        {}, {}
      },
      { // D
        {URF, UFL, ULB, UBR, DLF, DBL, DRB, DFR},
        {UR, UF, UL, UB, DF, DL, DB, DR, FR, FL, BL, BR},
        {}, {}
      },
      { // R
        {DFR, UFL, ULB, URF, DRB, DLF, DBL, UBR},
        {FR, UF, UL, UB, BR, DF, DL, DB, DR, FL, BL, UR},
        {2, 0, 0, 1, 1, 0, 0, 2}, {}
      },
      { // L
        {URF, ULB, DBL, UBR, DFR, UFL, DLF, DRB},
        {UR, UF, BL, UB, DR, DF, FL, DB, FR, UL, DL, BR},
        {0, 1, 2, 0, 0, 2, 1, 0}, {}
      },
      { // F
        {UFL, DLF, ULB, UBR, URF, DFR, DBL, DRB},
        {UR, FL, UL, UB, DR, FR, DL, DB, UF, DF, BL, BR},
        {1, 2, 0, 0, 2, 1, 0, 0},
        {0, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0}
      },
      { // B
        {URF, UFL, UBR, DRB, DFR, DLF, ULB, DBL},
        {UR, UF, UL, BR, DR, DF, DL, BL, FR, FL, UB, DB},
        {0, 0, 1, 2, 0, 0, 2, 1},
        {0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 1}
      }
    };

    int i = 0;
    for (int ax = 0; ax < 3; ax++) {
      int f1 = 2 * ax;
      int f2 = f1 + 1;
      int off1 = 15 * ax;
      int off2 = off1 + 3;

      cubes[i] = fcubes[f1];
      cubie::mul(cubes[i], fcubes[f1], cubes[i + 1]);
      cubie::mul(cubes[i + 1], fcubes[f1], cubes[i + 2]);
      i += 3;
      cubes[i] = fcubes[f2];
      cubie::mul(cubes[i], fcubes[f2], cubes[i + 1]);
      cubie::mul(cubes[i + 1], fcubes[f2], cubes[i + 2]);
      i += 3;

      for (int cnt1 = 0; cnt1 < 3; cnt1++) {
        for (int cnt2 = 0; cnt2 < 3; cnt2++)
          cubie::mul(cubes[off1 + cnt1], cubes[off2 + cnt2], cubes[i++]);
      }
    }

    for (int m : moves1)
      p1mask |= bit(m);
    for (int m : moves2)
      p2mask |= bit(m);

    for (int m = 0; m < COUNT; m++) {
      for (int m1 : blocked[m])
        next[m] |= bit(m1);
      next[m] = ~next[m];
    }
  }

  int translate(int m, const int fperm[6]) {
    if (m > move::COUNT_CUBE) // no translation of tilt-moves
      return m;

    int ax = m / 15;
    int i = m % 15;

    if (i < 6) { // simple move
      int f = 2 * ax + i / 3;
      int cnt = i % 3;
      int f1 = fperm[f];
      int ax1 = f / 2;
      return 15 * ax1 + (f1 - 2 * ax1) + cnt;
    } else { // axial move
      int f1 = 2 * ax;
      int f2 = f1 + 1;
      int cnt1 = (i - 6) / 3;
      int cnt2 = (i - 6) % 3;
      int f11 = fperm[f1];
      int f21 = fperm[f2];
      if (f11 > f21) {
        std::swap(f11, f21);
        std::swap(cnt1, cnt2);
      }
      int ax1 = f11 / 2;
      return (15 * ax1 + 6) + (3 * cnt1 + cnt2);
    }
  }

}
