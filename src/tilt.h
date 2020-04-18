#ifndef __STATE__
#define __STATE__

#include "face.h"
#include "move.h"
#include "sym.h"

namespace tilt {

  using namespace face::color;

  const int N_COORD = 24; // 4!, i.e. number of face permutations
  const int N_COORD_SYM = 3;

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
  extern int move_coord[N_COORD][move::COUNT];

  void mul(const cube& c1, const cube& c2, cube& into);
  bool operator==(const cube& c1, const cube& c2);

  int get_coord(const cube& c);
  void set_coord(cube& c, int coord);

  inline int cored_cls(int cored) { return cored >> 1; }
  inline bool cored_flip(int cored) { return cored & 1; } // whether conj + red flips the tilt moves

  void init();

}

#endif
