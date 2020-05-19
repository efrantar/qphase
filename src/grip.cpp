#include "grip.h"

#include <iostream>

#include <numeric>
#include "move.h"

namespace grip {

  // Datastructure for tracking the cube state
  struct cube {
    int blocked[4]; // int because we use more values than just 0 and 1
  };

  const cube INVALID = {-1, -1, -1, -1};

  // Compress/uncompress GRIP coordinate
  const int enc_state[16] = {
    RLFB, RLF, RLB, RL, RFB, -1, -1, -1, LFB, -1, -1, -1, FB, -1, -1, -1
  };
  const int dec_state[state::COUNT] = {
    0, 4, 8, 12, 2, 1, 3
  };

  // Cube moves + regrips
  cube move_cubes[N_MOVES][regrip::COUNT];
  std::string move_names[N_MOVES][regrip::COUNT];

  // Regrips for RL-axis only
  const cube MOVE_CUBES[15][regrip::COUNT] = {
    {{1, 0, 0, 0}, {1, 1, 0, 0}, {1, 0, 1, 0}, {1, 0, 0, 1}, INVALID,      {1, 1, 1, 0}, {1, 1, 0, 1}, INVALID     }, // R
    {{2, 0, 0, 0}, {2, 1, 0, 0}, {2, 0, 1, 0}, {2, 0, 0, 1}, INVALID,      {2, 1, 1, 0}, {2, 1, 0, 1}, INVALID     }, // R2
    {{1, 0, 0, 0}, {1, 1, 0, 0}, {1, 0, 1, 0}, {1, 0, 0, 1}, INVALID,      {1, 1, 1, 0}, {1, 1, 0, 1}, INVALID     }, // R'
    {{0, 1, 0, 0}, {1, 1, 0, 0}, {0, 1, 1, 0}, {0, 1, 0, 1}, INVALID,      {1, 1, 1, 0}, {1, 1, 0, 1}, INVALID     }, // L
    {{0, 2, 0, 0}, {1, 2, 0, 0}, {0, 2, 1, 0}, {0, 2, 0, 1}, INVALID,      {1, 2, 1, 0}, {1, 2, 0, 1}, INVALID     }, // L2
    {{0, 1, 0, 0}, {1, 1, 0, 0}, {0, 1, 1, 0}, {0, 1, 0, 1}, INVALID,      {1, 1, 1, 0}, {1, 1, 0, 1}, INVALID     }, // L'
    {{1, 1, 0, 0}, INVALID,      {1, 1, 1, 0}, {1, 1, 0, 1}, INVALID,      INVALID,      INVALID,      INVALID     }, // (R L)
    {{1, 2, 0, 0}, INVALID,      {1, 2, 1, 0}, {1, 2, 0, 1}, INVALID,      INVALID,      INVALID,      INVALID     }, // (R L2)
    {{1, 1, 0, 0}, INVALID,      {1, 1, 1, 0}, {1, 1, 0, 1}, INVALID,      INVALID,      INVALID,      INVALID     }, // (R L')
    {{2, 1, 0, 0}, INVALID,      {2, 1, 1, 0}, {2, 1, 0, 1}, INVALID,      INVALID,      INVALID,      INVALID     }, // (R2 L)
    {{2, 2, 0, 0}, INVALID,      {2, 2, 1, 0}, {2, 2, 0, 1}, INVALID,      INVALID,      INVALID,      INVALID     }, // (R2 L2)
    {{2, 1, 0, 0}, INVALID,      {2, 1, 1, 0}, {2, 1, 0, 1}, INVALID,      INVALID,      INVALID,      INVALID     }, // (R2 L')
    {{1, 1, 0, 0}, INVALID,      {1, 1, 1, 0}, {1, 1, 0, 1}, INVALID,      INVALID,      INVALID,      INVALID     }, // (R' L)
    {{1, 2, 0, 0}, INVALID,      {1, 2, 1, 0}, {1, 2, 0, 1}, INVALID,      INVALID,      INVALID,      INVALID     }, // (R' L2)
    {{1, 1, 0, 0}, INVALID,      {1, 1, 1, 0}, {1, 1, 0, 1}, INVALID,      INVALID,      INVALID,      INVALID     }, // (R' L')
  };

  // Tilt moves + regrips
  const cube TILT_CUBES[move::COUNT_TILT][regrip::COUNT] = {
    {{1, 1, 0, 0}, INVALID,      {1, 1, 1, 0}, {1, 1, 0, 1}, {1, 1, 1, 1}, INVALID     , INVALID     , INVALID     }, // tRL
    {{0, 0, 1, 1}, INVALID,      {1, 0, 1, 1}, {0, 1, 1, 1}, {1, 1, 1, 1}, INVALID     , INVALID     , INVALID     }  // tFB
  };

  // Hard regrips
  const cube REGRIP_CUBES[5][regrip::COUNT] = {
    {INVALID,      INVALID,      INVALID,      INVALID,      INVALID,      INVALID,      INVALID,      INVALID     }, // joint dummy regrip
    {{0, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}, {0, 0, 1, 1}, {0, 1, 1, 0}, {0, 1, 0, 1}, {0, 1, 1, 1}}, // rR
    {{0, 0, 0, 0}, {1, 0, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}, {0, 0, 1, 1}, {1, 0, 1, 0}, {1, 0, 0, 1}, {1, 0, 1, 1}}, // rL
    {{0, 0, 0, 0}, {0, 0, 0, 1}, {1, 0, 0, 0}, {0, 1, 0, 0}, {1, 1, 0, 0}, {1, 0, 0, 1}, {0, 1, 0, 1}, {1, 1, 0, 1}}, // rF
    {{0, 0, 0, 0}, {0, 0, 1, 0}, {1, 0, 0, 0}, {0, 1, 0, 0}, {1, 1, 0, 0}, {1, 0, 1, 0}, {0, 1, 1, 0}, {1, 1, 1, 0}}  // rB
  };

  int nextset[N_STATESETS][N_MOVES];
  int nextstate[state::COUNT][N_MOVES][regrip::COUNT];
  int score[state::COUNT][N_MOVES][regrip::COUNT];

  bool valid(const cube& c) {
    return !((c.blocked[0] || c.blocked[1]) && (c.blocked[2] || c.blocked[3]));
  }

  void mul(const cube& c1, const cube& c2, cube& into) {
    for (int i = 0; i < 4; i++)
      into.blocked[i] = (c1.blocked[i] + c2.blocked[i]) & 1;
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

  bool operator!=(const cube& c1, const cube& c2) {
    return !(c1 == c2);
  }

  int optim(const std::vector<int>& sol, std::vector<int>& parg, std::vector<int>& blog) {
    if (!sol.size()) // best avoid any trouble
      return 0;

    for (int m : sol)
      std::cout << move::names[m] << " ";
    std::cout << "\n";

    int len = sol.size();
    parg.resize(len);
    blog.resize(len);

    int dp[len][state::COUNT];
    int pd[len][state::COUNT];
    int pd1[len][state::COUNT];
    int pd2[len][state::COUNT];
    std::fill(dp[0], dp[0] + len * regrip::COUNT, -1);
    dp[0][0] = 0;

    int stateset = state::DEFAULTSET;
    for (int i = 0; i < sol.size() - 1; i++) {
      int m = sol[i];

      for (int state = 0; state < state::COUNT; state++) {
        if (!in(stateset, state))
          continue;

        for (int r = 0; r < regrip::COUNT; r++) {
          if (m == move::G) {
            for (int m1 = move::COUNT; m1 < N_MOVES; m1++) {
              int state1 = nextstate[state][m1][r];
              if (state1 == -1)
                continue;
              if (dp[i][state] + score[state][sol[i]][r] > dp[i + 1][state1]) {
                dp[i + 1][state1] = dp[i][state] + score[state][sol[i]][r];
                pd[i + 1][state1] = state;
                pd1[i + 1][state1] = r;
                pd2[i + 1][state1] = m1;
              }
            }
            continue;
          }

          int state1 = nextstate[state][m][r];
          if (state1 == -1)
            continue;
          if (dp[i][state] + score[state][m][r] > dp[i + 1][state1]) {
            dp[i + 1][state1] = dp[i][state] + score[state][m][r];
            pd[i + 1][state1] = state;
            pd1[i + 1][state1] = r;
          }
        }
      }

      stateset = nextset[stateset][m];
    }

    int best_s = 0;
    for (int s = 1; s < state::COUNT; s++) {
      if (dp[s] > dp[best_s])
        best_s = s;
    }
    int s = best_s;
    for (int i = len - 1; i > 0; i--) {
      parg[i - 1] = pd1[i][s];
      if (sol[i - 1] == move::G)
        blog[i - 1] = pd2[i][s];
      s = pd[i][s];
    }

    return dp[len - 1][best_s];
  }

  void init() {
    for (int m = 15; m < N_MOVES; m++) {
      for (int r = 0; r < regrip::COUNT; r++) {
        if (m < move::COUNT_CUBE) {
          move_cubes[m][r] = MOVE_CUBES[m % 15][r];
          if (m >= 30) {
            std::swap(move_cubes[m][r].blocked[0], move_cubes[m][r].blocked[2]);
            std::swap(move_cubes[m][r].blocked[1], move_cubes[m][r].blocked[3]);
          }
        } else if (m < move::COUNT - move::COUNT_GRIP)
          move_cubes[m][r] = TILT_CUBES[m - move::COUNT_CUBE][r];
        else
          move_cubes[m][r] = REGRIP_CUBES[m - (move::COUNT - move::COUNT_GRIP)][r];
      }
    }

    cube c;
    cube tmp;

    for (int stateset = 0; stateset < N_STATESETS; stateset++) {
      for (int state = 0; state < state::COUNT; state++) {
        if ((stateset & (1 << state)) == 0)
          continue;
        set_state(c, state);

        for (int m = 15; m < N_MOVES; m++) {
          if (m < move::COUNT_CUBE) {
            int other_ax = !(m / 15 - 1);
            if (c.blocked[2 * other_ax] && c.blocked[2 * other_ax + 1])
              continue;
          } else if (m >= move::COUNT - move::COUNT_GRIP) {
            int f = m - (move::COUNT - move::COUNT_GRIP);
            if (c.blocked[f])
              continue;
          }

          for (int r = 0; r < regrip::COUNT; r++) {
            if (move_cubes[m][r] == INVALID)
              continue;
            mul(c, move_cubes[m][r], tmp);
            if (valid(tmp))
              nextset[stateset][m] |= 1 << get_state(tmp);
          }
        }
      }

      for (int m = move::COUNT; m < N_MOVES; m++)
        nextset[stateset][move::G] |= nextset[stateset][m];
    }

    std::string face_names[] = {"rR", "rL", "rF", "rB"};
    for (int m = 0; m < N_MOVES; m++) {
      for (int r = 0; r < regrip::COUNT; r++) {
        move_names[m][r] = "[";
        for (int i = 0; i < 4; i++) {
          if (move_cubes[m][r].blocked[i] && !move_cubes[m][0].blocked[i])
            move_names[m][r] += face_names[i] + " ";
        }
        if (move_names[m][r].size() > 1)
          move_names[m][r][move_names[m][r].size() - 1] = ']';
        else
          move_names[m][r] += "]";
      }
    }

    std::fill(nextstate[0][0], nextstate[0][0] + state::COUNT * N_MOVES * regrip::COUNT, - 1);
    for (int state = 0; state < state::COUNT; state++) {
      set_state(c, state);

      for (int m = 15; m < N_MOVES; m++) {
        if (m < move::COUNT_CUBE) {
          int other_ax = !(m / 15 - 1);
          if (c.blocked[2 * other_ax] && c.blocked[2 * other_ax + 1])
            continue;
        } else if (m >= move::COUNT - move::COUNT_GRIP) {
          int f = m - (move::COUNT - move::COUNT_GRIP);
          if (c.blocked[f])
            continue;
        }

        for (int r = 0; r < regrip::COUNT; r++) {
          if (move_cubes[m][r] != INVALID) {
            mul(c, move_cubes[m][r], tmp);
            if (valid(tmp))
              nextstate[state][m][r] = get_state(tmp);
          }
        }
      }
    }

    for (int state = 0; state < state::COUNT; state++) {
      set_state(c, state);
      for (int m = 15; m < N_MOVES; m++) {
        if (m < move::COUNT - move::COUNT_GRIP) { // tilts are always safe
          std::fill(score[state][m], score[state][m] + regrip::COUNT, 1);
          continue;
        }
        for (int r = 0; r < regrip::COUNT; r++) {
          if (move_cubes[m][r] != INVALID) {
            int count = 0;
            for (int i = 0; i < 4; i++) {
              if (!move_cubes[m][0].blocked[i])
                count += !move_cubes[m][r].blocked[i] && !c.blocked[i];
              // Note that it is sufficient to count the non-moved non-blocked grippers to determine whether or
              // not a move is safe
            }
            score[state][m][r] = count >= 2;
          }
        }
      }
    }
  }

}
