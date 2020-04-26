#include "move.h"

namespace grip {

  const int N_STATE = 7;

  // Faces mentioned in the name are not blocked
  const int S_RLFB = 0;
  const int S_RFB = 1;
  const int S_LFB = 2;
  const int S_FB = 3;
  const int S_RLB = 4;
  const int S_RLF = 5;
  const int S_RL = 6;

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

  struct cube {
    bool blocked[4];
  };

  const cube INVALID = {1, 0, 1, 0};

  const int enc_state[16] = {
    S_RLFB, S_RLF, S_RLB, S_RL, S_RFB, -1, -1, -1, S_LFB, -1, -1, -1, S_FB, -1, -1, -1
  };
  const int dec_state[N_STATE] = {
    0, 4, 8, 12, 2, 1, 3
  };

  cube MOVE_CUBES[move::COUNT_CUBE][regrip::COUNT] = {

  };

  const cube MOVE_CUBES1[15][regrip::COUNT] = {
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

  const cube TILT_CUBES[move::COUNT_TILT][regrip::COUNT] = {
    {{1, 1, 0, 0}, {1, 1, 1, 1}, {1, 1, 1, 0}, {1, 1, 0, 1}, {1, 1, 1, 1}, INVALID     , INVALID     , INVALID     }, // tRL
    {{0, 0, 1, 1}, {1, 1, 1, 1}, {1, 0, 1, 1}, {0, 1, 1, 1}, {1, 1, 1, 1}, INVALID     , INVALID     , INVALID     }  // tFB
  };

  const cube REGRIP_CUBES[4][regrip::COUNT] = {

  };

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

}
