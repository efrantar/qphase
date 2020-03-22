#include "state.h"

#include <algorithm>
#include <vector>
#include "move.h"

namespace state {

  using namespace move;

  int coord_cls[N_COORD];
  int coord_rep[N_COORD_SYM];
  int cored_coord[N_COORD][sym::COUNT_SUB];

  int move_coord[N_COORD][move::COUNT];
  move::mask moves[N_COORD_SYM];

  const int N_AXPERM = 6;

  const int FPERMS[][6] = {
    {U, R, F, D, L, B},
    {U, B, R, D, F, L},
    {U, L, B, D, R, F},
    {U, F, L, D, B, R},
    {R, U, B, L, D, F},
    {F, U, R, B, D, L},
    {L, U, F, R, D, B},
    {B, U, L, F, D, R},
    {R, F, U, L, B, D},
    {B, R, U, F, L, D},
    {L, B, U, R, F, D},
    {F, L, U, B, R, D},
    {D, R, B, U, L, F},
    {D, F, R, U, B, L},
    {D, L, F, U, R, B},
    {D, B, L, U, F, R},
    {R, D, F, L, U, B},
    {B, D, R, F, U, L},
    {L, D, B, R, U, F},
    {F, D, L, B, U, R},
    {R, B, D, L, F, U},
    {F, R, D, B, L, U},
    {L, F, D, R, B, U},
    {B, L, D, F, R, U}
  };

  std::vector<int> blocked[N_COORD_SYM] = {
    {U1, U2, U3, D1, D2, D3, U1D1, U1D2, U1D3, U2D1, U2D2, U2D3, U3D1, U3D2, U3D3},
    {R1, R2, R3, L1, L2, L3, R1L1, R1L2, R1L3, R2L1, R2L2, R2L3, R3L1, R3L2, R3L3},
    {F1, F2, F3, B1, B2, B3, F1B1, F1B2, F1B3, F2B1, F2B2, F2B3, F3B1, F3B2, F3B3}
  };

  int axperm_enc[27];
  int axperm_dec[N_AXPERM];

  cube sym_cubes[sym::COUNT];

  void init() {
    int perm[] = {0, 1, 2};
    for (int i = 0; i < N_AXPERM; i++) {
      int tmp = 9 * perm[0] + 3 * perm[1] + perm[2];
      axperm_enc[tmp] = i;
      axperm_dec[i] = tmp;
      std::next_permutation(perm, perm + 3);
    }

    cube c = ID_CUBE;
    cube tmp;

    cube u4 = {
      {U, B, R, D, F, L}
    };
    cube urf3 = {
      {F, U, R, B, D, L}
    };

    for (int i = 0; i < sym::COUNT; i++) {
      sym_cubes[i] = c;

      if (i % 4 == 3) {
        mul(c, u4, tmp);
        std::swap(tmp, c);
      }
      if (i % 16 == 15) {
        mul(c, urf3, tmp);
        std::swap(tmp, c);
      }
    }

    std::fill(coord_cls, coord_cls + N_COORD, -1);
    cube c1;
    int cls = 0;

    for (int coord = 0; coord < N_COORD; coord++) {
      set_coord(c, coord);

      if (coord_cls[coord] != -1)
        continue;
      coord_cls[coord] = cls;
      coord_rep[cls] = coord;

      for (int s = 1; s < sym::COUNT_SUB; s++) {
        // Note that we want to reduce with respect to full cube rotations, not w.r.t. symmetry of the cube itself
        mul(c, sym_cubes[s], c1);
        int coord1 = get_coord(c1);
        if (coord_cls[coord1] == -1)
          coord_cls[coord1] = cls;
      }
      cls++;
    }

    for (int coord = 0; coord < N_COORD; coord++) {
      set_coord(c, coord);
      cored_coord[coord][0] = coord_cls[coord];
      for (int s = 1; s < sym::COUNT_SUB; s++) {
        // Here we want to conjugate with respect to an actual cube symmetry
        mul(sym_cubes[s], c, tmp);
        mul(tmp, sym_cubes[sym::inv[s]], c1);
        cored_coord[coord][s] = coord_cls[get_coord(c1)];
      }
    }

    for (int coord = 0; coord < N_COORD; coord++) {
      set_coord(c, coord);
      for (int m = 0; m < move::COUNT_CUBE; m++)
        move_coord[coord][m] = coord;
      for (int m = move::COUNT_CUBE; m < move::COUNT; m++) {
        mul(c, MOVES[m - move::COUNT_CUBE], c1);
        move_coord[coord][m] = get_coord(c1);
      }
    }

    for (int scoord = 0; scoord < N_COORD_SYM; scoord++) {
      for (int m : blocked[scoord])
        moves[scoord] |= move::bit(m);
      moves[scoord] = ~moves[scoord];
    }
  }

  void mul(const cube& c1, const cube& c2, cube& into) {
    for (int i = 0; i < face::color::COUNT; i++)
      into.fperm[i] = c1.fperm[c2.fperm[i]];
  }

  void inv(const cube& c, cube& into) {
    for (int face = 0; face < face::color::COUNT; face++)
      into.fperm[c.fperm[face]] = face;
  }

  bool operator==(const cube& c1, const cube& c2) {
    return std::equal(c1.fperm, c1.fperm + face::color::COUNT, c2.fperm);
  }

  int get_coord(const cube& c) {
    return axperm_enc[9 * (c.fperm[0] % 3) + 3 * (c.fperm[1] % 3) + (c.fperm[2] % 3)];
  }

  void set_coord(cube& c, int coord) {
    int axperm = axperm_dec[coord];
    for (int i = 2; i >= 0; i--) {
      c.fperm[i] = axperm % 3;
      c.fperm[i + 3] = c.fperm[i] + 3;
      axperm /= 3;
    }
  }

  int get_tilt(const cube& c) {
    for (int i = 0; i < N_TILT; i++) {
      if (std::equal(c.fperm, c.fperm + face::color::COUNT, FPERMS[i]))
        return i;
    }
    return -1;
  }

  void set_tilt(cube& c, int tilt) {
    std::copy(FPERMS[tilt], FPERMS[tilt] + face::color::COUNT, c.fperm);
  }

}
