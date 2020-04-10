#ifndef __STATE__
#define __STATE__

#include "face.h"
#include "move.h"
#include "sym.h"

namespace state {

  using namespace face::color;

  const int N_TILT = 24; // tilt, i.e. face permutation
  const int N_GRIP = 5; // gripper state
  const int N_COORD = N_TILT * N_GRIP; // overall state coordinate
  const int N_COORD_SYM = 15; // symmetry reduced overall state coordinate

  // Gripper states
  const int G_NEUTRAL = 0;
  const int G_PARTIAL_RL = 1;
  const int G_PARTIAL_FB = 2;
  const int G_BLOCKED_RL = 3;
  const int G_BLOCKED_FB = 4;

  struct cube {
    int fperm[face::color::COUNT]; // face permutation
    int grip; // gripper state
  };

  const cube ID_CUBE = {
    {U, R, F, D, L, B},
    G_NEUTRAL
  };

  const cube MOVES[] = {
    {{F, R, D, B, L, U}, G_BLOCKED_RL}, // RL tilt
    {{R, D, F, L, U, B}, G_BLOCKED_FB}, // FB tilt
    {{U, R, F, D, L, B}, G_PARTIAL_RL}, // RL regrip
    {{U, R, F, D, L, B}, G_PARTIAL_FB}  // FB regrip
  };

  extern int coord_cls[N_COORD];
  extern int coord_rep[N_COORD_SYM];
  extern int cored_coord[N_COORD][sym::COUNT_SUB]; // conj w.r.t. to cube-sym + reduction w.r.t. robot-sym

  extern move::mask moves[N_COORD];
  extern int conj_move[move::COUNT_STATE][sym::COUNT];
  extern int eff_mperm[sym::COUNT_SUB][move::COUNT_STATE];
  extern int move_coord[N_COORD][move::COUNT];

  void mul(const cube& c1, const cube& c2, cube& into);
  bool operator==(const cube& c1, const cube& c2);

  int get_coord(const cube& c);
  void set_coord(cube& c, int coord);

  void init();

}

#endif
