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
      coord_cls[coord] = N_GRIP * (c.fperm[0] % 3) + ((c.fperm[1] % 3 > c.fperm[2] % 3) ? G_FLIP[c.gstate] : c.gstate);
      coord_rep[coord_cls[coord]] = coord;
    }

    // The cube symmetries which affect the permutation of the axes, the grip is handled explicitly during conjugation
    cube u4 = {
      {U, B, R, D, F, L}, G_NEUTRAL
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

    for (int coord = 0; coord < N_COORD; coord++) {
      set_coord(c, coord);
      cored_coord[coord][0] = coord_cls[coord];
      for (int s = 1; s < sym::COUNT_SUB; s++) {
        // Here we want to conjugate w.r.t. to the cube symmetry, i.e. inversely permute the axes in the reference
        // frame of the robot (and then correct the grip accordingly)
        mul(sym_cubes[s], c, tmp);
        mul(tmp, sym_cubes[sym::inv[s]], c1);

        // Only symmetries 4 - 7 and 12 - 15 flip the grip exactly if the UD-axis is aligned
        if (c1.fperm[0] % 3 == 0 && s % 8 >= 4)
          c1.gstate = G_FLIP[c1.gstate];

        cored_coord[coord][s] = coord_cls[get_coord(c1)];
      }
    }

    std::fill(moves, moves + N_COORD, 0);
    for (int coord = 0; coord < N_COORD; coord++) {
      set_coord(c, coord);
      if (c.gstate == G_BLOCKED_RL) // if we are blocked we can only move that axis
        moves[coord] = (move::mask(0x7fff) << 15 * (c.fperm[1] % 3)) | (move::mask(0b0101) << move::COUNT_CUBE);
      else if (c.gstate == G_BLOCKED_FB)
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
        if (move::in(m, moves[coord_cls[coord]])) {
          for (int ax = 1; ax < 3; ax++) {
            if (c.fperm[ax] % 3 == m / 15) { // figure out where axis is located
              c1.gstate = G_MUL[c.gstate][GMOVES[m % 15] != G_NEUTRAL ? GMOVES[m % 15] + (ax - 1) : G_NEUTRAL];
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

  /*
  void init1() {
    int perm[] = {0, 1, 2};
    for (int i = 0; i < N_TILT; i++) {
      int tmp = 9 * perm[0] + 3 * perm[1] + perm[2];
      axperm_enc[tmp] = i;
      axperm_dec[i] = tmp;
      std::next_permutation(perm, perm + 3);
    }

    cube c = ID_CUBE;
    cube tmp;

    cube u4 = {
      {U, B, R, D, F, L}, G_FLIP
    };
    cube urf3 = {
      {F, U, R, B, D, L}, G_NEUTRAL // this symmetry is only legal for neutral grip anyways
    };
    // The other symmetries do not affect the face permutation

    for (int i = 0; i < sym::COUNT; i++) {
      sym_cubes[i] = c;

      if (i % 4 == 3) {
        mul(c, u4, tmp);
        tmp.gstate = tmp.gstate == G_FLIP ? G_NEUTRAL : G_FLIP; // normal mul defined for symmetry reduction
        std::swap(tmp, c);
      }
      if (i % 16 == 15) {
        mul(c, urf3, tmp);
        std::swap(tmp, c);
      }
    }

    std::fill(coord_cls, coord_cls + N_COORD, -1);
    cube c1;
    int cls = 0;

    for (int coord = 0; coord < N_COORD; coord++) {
      set_coord(c, coord);

      if (coord_cls[coord] != -1)
        continue;
      coord_cls[coord] = cls;
      coord_rep[cls] = coord;

      for (int s = 1; s < sym::COUNT_SUB; s++) {
        // Note that we want to reduce with respect to full cube rotations, not w.r.t. symmetry of the cube itself
        mul(c, sym_cubes[s], c1);
        int coord1 = get_coord(c1);
        if (coord_cls[coord1] == -1)
          coord_cls[coord1] = cls;
      }
      cls++;
    }

    for (int coord = 0; coord < N_COORD; coord++) {
      set_coord(c, coord);
      cored_coord[coord][0] = coord_cls[coord];
      for (int s = 1; s < sym::COUNT_SUB; s++) {
        // Here we want to conjugate with respect to an actual cube symmetry
        mul(sym_cubes[s], c, tmp);
        mul(tmp, sym_cubes[sym::inv[s]], c1);

        // A conjugation can only flip the grip if the UD-axis is aligned
        if (c1.fperm[0] % 3 == 0)
          c1.gstate = G_MUL[c1.gstate][sym_cubes[s].gstate];

        cored_coord[coord][s] = coord_cls[get_coord(c1)];
      }
    }

    for (int coord = 0; coord < N_COORD; coord++) {
      set_coord(c, coord);

      c1 = c;
      for (int m = 0; m < move::COUNT_CUBE; m++) {
        for (int i = 0; i < 3; i++) {
          if (c.fperm[i] % 3 == m / 15) { // figure out where axis is located
            if (i > 0) {
              c1.gstate = G_MUL[c.gstate][GMOVES[m % 15] + (i - 1)];
              move_coord[coord][m] = get_coord(c1);
            }
            break;
          }
        }
      }
      for (int m = move::COUNT_CUBE; m < move::COUNT; m++) {
        mul(c, MOVES[m - move::COUNT_CUBE], c1);
        move_coord[coord][m] = get_coord(c1);
      }
    }

    std::fill(moves, moves + N_COORD_SYM, 0);
    for (int scoord = 0; scoord < N_COORD_SYM; scoord++) {
      set_coord(c, coord_rep[scoord]);
      if (c.gstate == G_BLOCKED_RL) // if we are blocked we can only move this axis
        moves[scoord] = (move::mask(0x7fff) << 15 * (c.fperm[1] % 3)) | (move::mask(0b0101) << move::COUNT_CUBE);
      else if (c.gstate == G_BLOCKED_FB)
        moves[scoord] = (move::mask(0x7fff) << 15 * (c.fperm[2] % 3)) | (move::mask(0b1010) << move::COUNT_CUBE);
      else {
        moves[scoord] |= move::mask(0x7fff) << 15 * (c.fperm[1] % 3);
        moves[scoord] |= move::mask(0x7fff) << 15 * (c.fperm[2] % 3);
        moves[scoord] |= move::mask(0xf) << move::COUNT_CUBE;
      }
    }

    for (int m = 0; m < move::COUNT_STATE; m++) {
      for (int s = 0; s < sym::COUNT; s++) {
        mul(sym_cubes[s], MOVES[m], tmp);
        mul(tmp, sym_cubes[sym::inv[s]], c);
        for (int conj = 0; conj < move::COUNT_STATE; conj++) {
          if (c == MOVES[conj])
            conj_move[m][s] = conj;
        }
      }
    }

    for (int s = 0; s < sym::COUNT_SUB; s++) {
      for (int m = 0; m < move::COUNT_STATE; m++)
        eff_mperm[s][m] = conj_move[m][sym::inv[s]];
    }
  }
  */

  void mul(const cube& c1, const cube& c2, cube& into) {
    for (int i = 0; i < face::color::COUNT; i++)
      into.fperm[i] = c1.fperm[c2.fperm[i]];
    into.gstate = G_MUL[c1.gstate][c2.gstate];
  }

  bool operator==(const cube& c1, const cube& c2) {
    return std::equal(c1.fperm, c1.fperm + face::color::COUNT, c2.fperm);
  }

  int get_coord(const cube& c) {
    for (int tilt = 0; tilt < N_TILT; tilt++) {
      if (std::equal(c.fperm, c.fperm + face::color::COUNT, FPERMS[tilt]))
        return N_GRIP * tilt + c.gstate;
    }
    return -1;
  }

  void set_coord(cube& c, int coord) {
    std::copy(FPERMS[coord / N_GRIP], FPERMS[coord / N_GRIP] + face::color::COUNT, c.fperm);
    c.gstate = coord % N_GRIP;
  }

}
