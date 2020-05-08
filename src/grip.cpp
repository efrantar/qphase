#include "grip.h"

#include <iostream>
#include <queue>
#include "move.h"
#include "tilt.h"

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


  // Datastructure for tracking the cube state
  struct cube {
    int blocked[4];
  };

  const cube INVALID = {-1, -1, -1, -1}; // this is why we defined `blocked` as an integer

  // Compress/uncompress GRIP coordinate
  const int enc_state[16] = {
    RLFB, RLF, RLB, RL, RFB, -1, -1, -1, LFB, -1, -1, -1, FB, -1, -1, -1
  };
  const int dec_state[state::COUNT] = {
    0, 4, 8, 12, 2, 1, 3
  };

  // Cube moves + regrips
  cube move_cubes[N_MOVES][regrip::COUNT];

  // Regrips for RL-axis only
  const cube MOVE_CUBES[15][regrip::COUNT] = {
    {{1, 0, 0, 0}, {1, 1, 0, 0}, {1, 0, 1, 0}, {1, 0, 0, 1}, INVALID,      {0, 1, 1, 0}, {0, 1, 0, 1}, INVALID     }, // R
    {{0, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}, INVALID,      {0, 1, 1, 0}, {0, 1, 0, 1}, INVALID     }, // R2
    {{1, 0, 0, 0}, {1, 1, 0, 0}, {1, 0, 1, 0}, {1, 0, 0, 1}, INVALID,      {0, 1, 1, 0}, {0, 1, 0, 1}, INVALID     }, // R'
    {{1, 0, 0, 0}, {1, 1, 0, 0}, {1, 0, 1, 0}, {1, 0, 0, 1}, INVALID,      {1, 0, 1, 0}, {1, 0, 0, 1}, INVALID     }, // L
    {{0, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}, INVALID,      {1, 0, 1, 0}, {1, 0, 0, 1}, INVALID     }, // L2
    {{1, 0, 0, 0}, {1, 1, 1, 1}, {1, 1, 1, 0}, {1, 1, 0, 1}, INVALID,      {1, 0, 1, 0}, {1, 0, 0, 1}, INVALID     }, // L'
    {{1, 1, 0, 0}, {1, 1, 1, 1}, {1, 1, 1, 0}, {1, 1, 0, 1}, INVALID,      INVALID,      INVALID,      INVALID     }, // (R L)
    {{1, 0, 0, 0}, {1, 0, 0, 0}, {1, 0, 1, 0}, {1, 0, 0, 1}, INVALID,      INVALID,      INVALID,      INVALID     }, // (R L2)
    {{1, 1, 0, 0}, {1, 1, 1, 1}, {1, 1, 1, 0}, {1, 1, 0, 1}, INVALID,      INVALID,      INVALID,      INVALID     }, // (R L')
    {{0, 1, 0, 0}, {0, 1, 0, 0}, {0, 1, 1, 0}, {0, 1, 0, 1}, INVALID,      INVALID,      INVALID,      INVALID     }, // (R2 L)
    {{0, 0, 0, 0}, {1, 1, 1, 1}, {0, 0, 1, 0}, {0, 0, 0, 1}, INVALID,      INVALID,      INVALID,      INVALID     }, // (R2 L2)
    {{0, 1, 0, 0}, {0, 1, 0, 0}, {0, 1, 1, 0}, {0, 1, 0, 1}, INVALID,      INVALID,      INVALID,      INVALID     }, // (R2 L')
    {{1, 1, 0, 0}, {1, 1, 1, 1}, {1, 1, 1, 0}, {1, 1, 0, 1}, INVALID,      INVALID,      INVALID,      INVALID     }, // (R' L)
    {{1, 0, 0, 0}, {1, 0, 0, 0}, {1, 0, 1, 0}, {1, 0, 0, 1}, INVALID,      INVALID,      INVALID,      INVALID     }, // (R' L2)
    {{1, 1, 0, 0}, {1, 1, 1, 1}, {1, 1, 1, 0}, {1, 1, 0, 1}, INVALID,      INVALID,      INVALID,      INVALID     }, // (R' L')
  };

  // Tilt moves + regrips
  const cube TILT_CUBES[move::COUNT_TILT][regrip::COUNT] = {
    {{1, 1, 0, 0}, {1, 1, 1, 1}, {1, 1, 1, 0}, {1, 1, 0, 1}, {1, 1, 1, 1}, INVALID     , INVALID     , INVALID     }, // tRL
    {{0, 0, 1, 1}, {1, 1, 1, 1}, {1, 0, 1, 1}, {0, 1, 1, 1}, {1, 1, 1, 1}, INVALID     , INVALID     , INVALID     }  // tFB
  };

  // Hard regrips
  const cube REGRIP_CUBES[4][regrip::COUNT] = {
    {{0, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}, {0, 0, 1, 1}, {0, 1, 1, 0}, {0, 1, 0, 1}, {0, 1, 1, 1}}, // rR
    {{0, 0, 0, 0}, {1, 0, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}, {0, 0, 1, 1}, {1, 0, 1, 0}, {1, 0, 0, 1}, {1, 0, 1, 1}}, // rL
    {{0, 0, 0, 0}, {0, 0, 0, 1}, {1, 0, 0, 0}, {0, 1, 0, 0}, {1, 1, 0, 0}, {1, 0, 0, 1}, {0, 1, 0, 1}, {1, 1, 0, 1}}, // rF
    {{0, 0, 0, 0}, {0, 0, 1, 0}, {1, 0, 0, 0}, {0, 1, 0, 0}, {1, 1, 0, 0}, {1, 0, 1, 0}, {0, 1, 1, 0}, {1, 1, 1, 0}}  // rB
  };

  int nextset[N_STATESETS][move::COUNT];
  move::mask setmoves[N_STATESETS];

  bool valid(const cube& c) {
    return !((c.blocked[0] || c.blocked[1]) && (c.blocked[2] || c.blocked[3]));
  }

  void mul(const cube& c1, const cube& c2, cube& into) {
    for (int i = 0; i < 4; i++)
      into.blocked[i] = c1.blocked[i] ^ c2.blocked[i];
  }

  int get_state(const cube& c) {
    int state = 0;
    for (int i = 0; i < 4; i++)
      state = (state << 1) | c.blocked[i];
    return enc_state[state];
  }

  void set_state(cube& c, int state) {
    state = dec_state[state];
    for (int i = 3; i >= 0; i--) {
      c.blocked[i] = state & 1;
      state >>= 1;
    }
  }

  bool operator==(const cube& c1, const cube& c2) {
    return std::equal(c1.blocked, c1.blocked + 4, c2.blocked);
  }

  void init() {
    for (int m = 15; m < move::COUNT + 4; m++) {
      for (int r = 0; r < regrip::COUNT; r++) {
        if (m < move::COUNT_CUBE) {
          move_cubes[m][r] = MOVE_CUBES[m % 15][r];
          if (m >= 30) {
            std::swap(move_cubes[m][r].blocked[0], move_cubes[m][r].blocked[2]);
            std::swap(move_cubes[m][r].blocked[1], move_cubes[m][r].blocked[3]);
          }
        } else if (m < move::COUNT)
          move_cubes[m][r] = TILT_CUBES[m - move::COUNT_CUBE][r];
        else
          move_cubes[m][r] = REGRIP_CUBES[m - move::COUNT_CUBE][r];
      }
    }

    cube c;
    cube tmp;

    for (int stateset = 0; stateset < N_STATESETS; stateset++) {
      for (int state = 0; state < state::COUNT; state++) {
        if ((stateset & (1 << state)) == 0)
          continue;
        set_state(c, state);

        for (int m = 15; m < move::COUNT; m++) {
          if (m < move::COUNT_CUBE) {
            int ax = m / 15 - 1;
            if (c.blocked[2 * ax] == 1 && c.blocked[2 * ax + 1] == 1)
              continue;
          } else if (m >= move::COUNT) {
            int f = m - move::COUNT;
            if (c.blocked[f])
              continue;
          }

          bool possible = false;
          for (int r = 0; r < regrip::COUNT; r++) {
            if (move_cubes[m][r] == INVALID)
              continue;
            mul(c, move_cubes[m][r], tmp);
            if (valid(tmp)) {
              nextset[stateset][m] |= 1 << get_state(tmp);
              possible = true;
            }
          }
          if (possible)
            setmoves[stateset] |= move::bit(m);
        }
      }
    }
  }

}
