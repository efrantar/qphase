#ifndef __STATE__
#define __STATE__

#include "face.h"
#include "move.h"
#include "sym.h"

namespace state {

  using namespace face::color;

  const int N_TILT = 24; // tilt, i.e. face permutation
  const int N_COORD = N_TILT; // overall state coordinate
  const int N_COORD_SYM = 3; // symmetry reduced overall state coordinate

  struct cube {
    int fperm[face::color::COUNT]; // face permutation
  };

  const cube ID_CUBE = {{U, R, F, D, L, B}};

  const cube MOVES[] = {
    {{F, R, D, B, L, U}}, // RL tilt
    {{R, D, F, L, U, B}}, // FB tilt
  };

  extern int coord_cls[N_COORD];
  extern int coord_rep[N_COORD_SYM];
  extern int cored_coord[N_COORD][sym::COUNT_SUB]; // conj w.r.t. to cube-sym + reduction w.r.t. robot-sym

  extern move::mask moves[N_COORD];
  extern int conj_move[move::COUNT_STATE][sym::COUNT];
  extern int move_coord[N_COORD][move::COUNT];

  void mul(const cube& c1, const cube& c2, cube& into);
  bool operator==(const cube& c1, const cube& c2);

  int get_coord(const cube& c);
  void set_coord(cube& c, int coord);

  void init();

}

#endif
