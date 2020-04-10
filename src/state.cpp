#include "state.h"

#include <algorithm>
#include <vector>
#include "move.h"

namespace state {

  using namespace move;

  int coord_cls[N_COORD];
  int coord_rep[N_COORD_SYM];
  int cored_coord[N_COORD][sym::COUNT_SUB];

  move::mask moves[N_COORD];
  int conj_move[move::COUNT_STATE][sym::COUNT];
  int eff_mperm[sym::COUNT_SUB][move::COUNT_STATE];
  int move_coord[N_COORD][move::COUNT];

  // Multiply gripper states
  const int G_MUL[6][6] = {
    {G_NEUTRAL,    G_PARTIAL_RL, G_PARTIAL_FB, G_BLOCKED_RL, G_BLOCKED_FB},
    {G_PARTIAL_RL, G_PARTIAL_RL, G_PARTIAL_FB, G_PARTIAL_RL, G_BLOCKED_FB},
    {G_PARTIAL_FB, G_PARTIAL_RL, G_PARTIAL_FB, G_BLOCKED_RL, G_PARTIAL_FB},
    {G_BLOCKED_RL, G_PARTIAL_RL, -1,           G_NEUTRAL,    -1          },
    {G_BLOCKED_FB, -1,           G_PARTIAL_FB, -1,           G_NEUTRAL   }
  };

  // All legal face permutations
  const int FPERMS[][6] = {
    {U, R, F, D, L, B},
    {U, B, R, D, F, L},
    {U, L, B, D, R, F},
    {U, F, L, D, B, R},
    {R, U, B, L, D, F},
    {F, U, R, B, D, L},
    {L, U, F, R, D, B},
    {B, U, L, F, D, R},
    {R, F, U, L, B, D},
    {B, R, U, F, L, D},
    {L, B, U, R, F, D},
    {F, L, U, B, R, D},
    {D, R, B, U, L, F},
    {D, F, R, U, B, L},
    {D, L, F, U, R, B},
    {D, B, L, U, F, R},
    {R, D, F, L, U, B},
    {B, D, R, F, U, L},
    {L, D, B, R, U, F},
    {F, D, L, B, U, R},
    {R, B, D, L, F, U},
    {F, R, D, B, L, U},
    {L, F, D, R, B, U},
    {B, L, D, F, R, U}
  };

  // Flip grip axis
  const int G_FLIP[] = {G_NEUTRAL, G_PARTIAL_FB, G_PARTIAL_RL, G_BLOCKED_FB, G_BLOCKED_RL};

  // Cubes representing the face permutation of cube symmetries
  cube sym_cubes[sym::COUNT];

  // Effect of moves on grip (for RL-axis); note that a half-turn can always be executed in partial manner
  const int GMOVES[] = {
    G_PARTIAL_RL, G_PARTIAL_RL, G_PARTIAL_RL, G_PARTIAL_RL, G_PARTIAL_RL, G_PARTIAL_RL,
    G_BLOCKED_RL, G_PARTIAL_RL, G_BLOCKED_RL, G_PARTIAL_RL, G_NEUTRAL, G_PARTIAL_RL, G_BLOCKED_RL, G_PARTIAL_RL, G_BLOCKED_RL
  };

  void init() {
    cube c = ID_CUBE;
    cube c1;
    cube tmp;

    // The symmetry class w.r.t. the robot is defined by the axis in the UD-slot and the gripper state if the other
    // two axes are in order or the axis-flipped gripper state if they are not.
    for (int coord = N_COORD - 1; coord >= 0; coord--) { // the rep should always be the smallest value
      set_coord(c, coord);
      coord_cls[coord] = N_GRIP * (c.fperm[0] % 3) + ((c.fperm[1] % 3 > c.fperm[2] % 3) ? G_FLIP[c.grip] : c.grip);
      coord_rep[coord_cls[coord]] = coord;
    }

    // Note that the following conjugation w.r.t. cube symmetries is only correct in terms of the axes permutation
    // (but not the full face ordering); this is however sufficient here.

    // The cube symmetries which affect the permutation of the axes
    cube u4 = {
      {U, F, L, D, B, R}, G_NEUTRAL
    };
    cube urf3 = {
      {F, U, R, B, D, L}, G_NEUTRAL
    };

    c = ID_CUBE;
    for (int i = 0; i < sym::COUNT; i++) {
      sym_cubes[i] = c;

      if (i % 4 == 3) {
        mul(c, u4, tmp);
        std::swap(tmp, c);
      }
      if (i % 16 == 15) {
        mul(c, urf3, tmp);
        std::swap(tmp, c);
      }
    }

    // Here we want to conjugate w.r.t. to the cube symmetry, i.e. permute the axes in the reference frame of the
    // robot; note that such a conjugation should not affect the grip
    for (int coord = 0; coord < N_COORD; coord++) {
      set_coord(c, coord);
      cored_coord[coord][0] = coord_cls[coord];
      for (int s = 1; s < sym::COUNT_SUB; s++) {
        for (int i = 0; i < face::color::COUNT; i++) {
          for (int j = 0; j < face::color::COUNT; j++) {
            if (c.fperm[j] == i)
              c1.fperm[j] = sym_cubes[s].fperm[i];
          }
        }
        c1.grip = c.grip;

        cored_coord[coord][s] = coord_cls[get_coord(c1)];
      }
    }

    std::fill(moves, moves + N_COORD, 0);
    for (int coord = 0; coord < N_COORD; coord++) {
      set_coord(c, coord);
      if (c.grip == G_BLOCKED_RL) // if we are blocked we can only move that axis
        moves[coord] = (move::mask(0x7fff) << 15 * (c.fperm[1] % 3)) | (move::mask(0b0101) << move::COUNT_CUBE);
      else if (c.grip == G_BLOCKED_FB)
        moves[coord] = (move::mask(0x7fff) << 15 * (c.fperm[2] % 3)) | (move::mask(0b1010) << move::COUNT_CUBE);
      else {
        moves[coord] |= move::mask(0x7fff) << 15 * (c.fperm[1] % 3);
        moves[coord] |= move::mask(0x7fff) << 15 * (c.fperm[2] % 3);
        moves[coord] |= move::mask(0xf) << move::COUNT_CUBE;
      }
    }

    for (int coord = 0; coord < N_COORD; coord++) {
      set_coord(c, coord);

      c1 = c;
      for (int m = 0; m < move::COUNT_CUBE; m++) {
        if (move::in(m, moves[coord])) {
          for (int ax = 1; ax < 3; ax++) {
            if (c.fperm[ax] % 3 == m / 15) { // figure out where axis is located
              c1.grip = G_MUL[c.grip][GMOVES[m % 15] != G_NEUTRAL ? GMOVES[m % 15] + (ax - 1) : G_NEUTRAL];
              move_coord[coord][m] = get_coord(c1);
            }
          }
        } else
          move_coord[coord][m] = -1;
      }
      for (int m = move::COUNT_CUBE; m < move::COUNT; m++) {
        mul(c, MOVES[m - move::COUNT_CUBE], c1);
        move_coord[coord][m] = get_coord(c1);
      }
    }
  }

  void mul(const cube& c1, const cube& c2, cube& into) {
    for (int i = 0; i < face::color::COUNT; i++)
      into.fperm[i] = c1.fperm[c2.fperm[i]];
    into.grip = G_MUL[c1.grip][c2.grip];
  }

  bool operator==(const cube& c1, const cube& c2) {
    return std::equal(c1.fperm, c1.fperm + face::color::COUNT, c2.fperm);
  }

  int get_coord(const cube& c) {
    for (int tilt = 0; tilt < N_TILT; tilt++) {
      if (std::equal(c.fperm, c.fperm + face::color::COUNT, FPERMS[tilt]))
        return N_GRIP * tilt + c.grip;
    }
    return -1;
  }

  void set_coord(cube& c, int coord) {
    std::copy(FPERMS[coord / N_GRIP], FPERMS[coord / N_GRIP] + face::color::COUNT, c.fperm);
    c.grip = coord % N_GRIP;
  }

}
