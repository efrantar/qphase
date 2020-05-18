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

    const int DEFAULTSET = 1;
    inline bool in(int stateset, int state) { return stateset & (1 << state); }
  }
  using namespace state;

  // Regrip options
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

  const int N_STATESETS = 1 << state::COUNT;
  const int N_MOVES = move::COUNT + 4; // + 4 regrips anchored on each face respectively

  extern std::string move_names[N_MOVES][regrip::COUNT];
  extern int nextset[N_STATESETS][N_MOVES];

  int optim(const std::vector<int>& sol, std::vector<int>& parg, std::vector<int>& blog);
  void init();

}

#endif
