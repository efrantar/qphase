#include "state.h"

#include <algorithm>
#include <vector>
#include "move.h"

namespace state {

  using namespace move;

  int summ_cls[N_SUMM];
  int summ_rep[N_SUMM_SYM];
  int cored_summ[N_SUMM][sym::COUNT_SUB];

  int move_summ[N_SUMM][move::COUNT];
  move::mask moves[N_SUMM_SYM];

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

  std::vector<int> blocked[N_SUMM_SYM] = {
    {U1, U2, U3, D1, D2, D3, U1D1, U1D2, U1D3, U2D1, U2D2, U2D3, U3D1, U3D2, U3D3, RUD},
    {R1, R2, R3, L1, L2, L3, R1L1, R1L2, R1L3, R2L1, R2L2, R2L3, R3L1, R3L2, R3L3, RRL},
    {F1, F2, F3, B1, B2, B3, F1B1, F1B2, F1B3, F2B1, F2B2, F2B3, F3B1, F3B2, F3B3, RFB}
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

    std::fill(summ_cls, summ_cls + N_SUMM, -1);
    cube c1;
    int cls = 0;

    for (int summ = 0; summ < N_SUMM; summ++) {
      set_summ(c, summ);

      if (summ_cls[summ] != -1)
        continue;
      summ_cls[summ] = cls;
      summ_rep[cls] = summ;

      for (int s = 1; s < sym::COUNT_SUB; s++) {
        // Note that we want to reduce with respect to full cube rotations, not w.r.t. symmetry of the cube itself
        mul(c, sym_cubes[s], c1);
        int summ1 = get_summ(c1);
        if (summ_cls[summ1] == -1)
          summ_cls[summ1] = cls;
      }
      cls++;
    }

    for (int summ = 0; summ < N_SUMM; summ++) {
      set_summ(c, summ);
      cored_summ[summ][0] = summ_cls[summ];
      for (int s = 1; s < sym::COUNT_SUB; s++) {
        // Here we want to conjugate with respect to an actual cube symmetry
        mul(sym_cubes[s], c, tmp);
        mul(tmp, sym_cubes[sym::inv[s]], c1);
        cored_summ[summ][s] = summ_cls[get_summ(c1)];
      }
    }

    for (int summ = 0; summ < N_SUMM; summ++) {
      set_summ(c, summ);
      for (int m = 0; m < move::COUNT_CUBE; m++)
        move_summ[summ][m] = summ;
      for (int m = move::COUNT_CUBE; m < move::COUNT; m++) {
        mul(c, MOVES[m - move::COUNT_CUBE], c1);
        move_summ[summ][m] = get_summ(c1);
      }
    }

    for (int ssumm = 0; ssumm < N_SUMM_SYM; ssumm++) {
      for (int m : blocked[ssumm])
        moves[ssumm] |= move::bit(m);
      moves[ssumm] = ~moves[ssumm];
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

  int get_summ(const cube& c) {
    return axperm_enc[9 * (c.fperm[0] % 3) + 3 * (c.fperm[1] % 3) + (c.fperm[2] % 3)];
  }

  void set_summ(cube& c, int summ) {
    int axperm = axperm_dec[summ];
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
