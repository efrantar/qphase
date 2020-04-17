/**
 * Pruning table generation and lookup.
 */

#ifndef __PRUN__
#define __PRUN__

#include <cstdint>

#include "coord.h"
#include "tilt.h"
#include "sym.h"

namespace prun {

  const int N_FS1TWIST = sym::N_FSLICE1 * coord::N_TWIST * tilt::N_COORD_SYM;
  const int N_CORNUD2 = sym::N_CORNERS * coord::N_UDEDGES2 * tilt::N_COORD_SYM;
  const int N_CSLICE2 = coord::N_CORNERS * coord::N_SLICE2 * tilt::N_COORD_SYM;

  extern uint64_t *phase1;
  extern uint8_t *phase2;
  extern uint8_t *precheck;

  int get_phase1(int flip, int slice, int twist, int tilt, int togo, move::mask& next);
  int get_phase1(int flip, int slice, int twist, int state);
  int get_phase2(int corners, int udedges2, int state);
  int get_precheck(int corners, int slice, int state);

  bool init(bool file = true);

}

#endif
