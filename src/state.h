#ifndef __STATE__
#define __STATE__

#include "face.h"
#include "move.h"
#include "sym.h"

namespace state {

  using namespace face::color;

  const int N_COORD = 6;
  const int N_COORD_SYM = 3;
  const int N_TILT = 24;

  struct cube {
    int fperm[face::color::COUNT];
  };

  const cube ID_CUBE = {
    {U, R, F, D, L, B}
  };

  const cube MOVES[] = {
    {{F, R, D, B, L, U}},
    {{R, D, F, L, U, B}}
  };

  extern int coord_cls[N_COORD];
  extern int coord_rep[N_COORD_SYM];
  extern int cored_coord[N_COORD][sym::COUNT_SUB];

  extern int move_coord[N_COORD][move::COUNT];
  extern move::mask moves[N_COORD_SYM];

  void mul(const cube& c1, const cube& c2, cube& into);
  void inv(const cube& c, cube& into);
  bool operator==(const cube& c1, const cube& c2);

  int get_coord(const cube& c);
  int get_tilt(const cube& c);
  void set_coord(cube& c, int coord);
  void set_tilt(cube& c, int tilt);

  void init();

}

#endif
