#ifndef __STATE__
#define __STATE__

#include "face.h"
#include "sym.h"

namespace state {

  using namespace face::color;

  const int N_SUMM = 6;
  const int N_SUMM_SYM = 3;
  const int N_TILT = 24;

  struct cube {
    int fperm[face::COUNT];
  };

  const cube ID_CUBE = {
    {U, R, F, D, L, B}
  };

  int cored_summ[N_SUMM][sym::COUNT_SUB];

  void mul(const cube& c1, const cube& c2, cube& into);
  void inv(const cube& c, cube& into);
  bool operator==(const cube& c1, const cube& c2);

  int get_summ(const cube& c);
  int get_tilt(const cube& c);
  void set_summ(cube& c, int summ);
  void set_tilt(cube& c, int tilt);

}

#endif
