/**
 * Handling of regrips; primarily stateset transition tables and eventual regrip reconstruction.
 */

#ifndef __GRIP__
#define __GRIP__

#include <vector>
#include "move.h"

namespace grip {

  // Gripper states; faces mentioned in the name are not blocked
  namespace state {
    const int COUNT = 7;

    const int RLFB = 0;
    const int RFB = 1;
    const int LFB = 2;
    const int FB = 3;
    const int RLB = 4;
    const int RLF = 5;
    const int RL = 6;

    inline bool in(int stateset, int state) { return stateset & (1 << state); }
  }
  using namespace state;

  // Regrip options; mentioned axis are affected (A is moved axis, B the orthogonal one, 1 lower and 2 higher face)
  namespace regrip {
    const int COUNT = 8;

    const int _      = 0; // no regripping whatsoever
    const int A2     = 1; // regrip parallel face
    const int B1     = 2; // regrip lower indexed orthogonal face
    const int B2     = 3; // regrip higher indexed orthogonal face
    const int B1B2   = 4; // regrip both orthogonal faces
    const int A2B1   = 5; // regrip parallel and lower orthogonal
    const int A2B2   = 6; // regrip parallel and higher orthogonal
    const int A2B1B2 = 7; // triple regrip of all but the current face (which must be in neutral)
  }
  using namespace regrip;

  // Datastructure for tracking the cube state
  struct cube {
    int blocked[4]; // int because we use more values than just 0 and 1
  };

  const int N_MOVES = move::COUNT + 4; // + 4 regrips anchored on each face respectively
  const int N_STATESETS = 1 << state::COUNT;
  const int DEFAULTSET = 1;
  const int ALLSTATES = N_STATESETS - 1; // all bits on

  // Cube moves + regrips, UD-moves do not exist here (reindexing would just make things more complicated)
  extern cube move_cubes[N_MOVES][regrip::COUNT];
  extern std::string move_names[N_MOVES][regrip::COUNT];

  extern int nextset[N_STATESETS][N_MOVES];
  extern int nextiset[N_STATESETS][N_MOVES]; // for inverse searches
  extern int nextstate[state::COUNT][N_MOVES][regrip::COUNT];

  bool valid(const cube& c);
  void mul(const cube& c1, const cube& c2, cube& into);
  int get_state(const cube& c);
  void set_state(cube& c, int state);

  /*
   * Find the best actual regrip solution. `parg` contains the parallel regrips, `blog` any blocked regrips and the
   * final solution score is returned. Currently this is the number of "safe" (i.e. with at least double support) moves.
   */
  int optim(const std::vector<int>& sol, std::vector<int>& parg, std::vector<int>& blog);

  bool operator==(const cube& c1, const cube& c2);
  bool operator!=(const cube& c1, const cube& c2);

  void init();

}

#endif
