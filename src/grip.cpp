#include "grip.h"

#include <numeric>
#include "move.h"

namespace grip {

  const cube INVALID = {-1, -1, -1, -1}; // we need to distinguish invalid temporary move cubes and truly invalid states

  // Compress/uncompress GRIP coordinate
  const int enc_state[16] = {
    RLFB, RLF, RLB, RL, RFB, -1, -1, -1, LFB, -1, -1, -1, FB, -1, -1, -1
  };
  const int dec_state[state::COUNT] = {
    0, 4, 8, 12, 2, 1, 3
  };

  cube move_cubes[N_MOVES][regrip::COUNT];
  std::string move_names[N_MOVES][regrip::COUNT];

  // Regrips for RL-axis only (we use 2s to detect the moved faces)
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

  // TODO
  // In some situations we can
  const move::mask CUTOPTIONS = move::bit(move::R1L2) | move::bit(move::R2L1) | move::bit(move::F1B2) | move::bit(move::F2B1);

  int nextset[N_STATESETS][N_MOVES];
  int nextiset[N_STATESETS][N_MOVES];
  int nextstate[state::COUNT][N_MOVES][regrip::COUNT];
  int score[state::COUNT][N_MOVES][regrip::COUNT];

  // Opposite axes are not both blocked
  bool valid(const cube& c) {
    return !((c.blocked[0] || c.blocked[1]) && (c.blocked[2] || c.blocked[3]));
  }

  void mul(const cube& c1, const cube& c2, cube& into) {
    for (int i = 0; i < 4; i++)
      into.blocked[i] = (c1.blocked[i] + c2.blocked[i]) & 1; // to handle 2s
  }

  /* Simply binary encoding plus compression. */

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

  /*
   * - One extra DP state for every gripper state indicating a cutable scenario
   * - 4 extra states indicating that a certain face did not move in a blocking state
   *  - Note that we only have forced hard regrips and thus 4 states should be sufficient
   */

  /*
   * Always make the choice that
   * Always make the choice that does a half-turn in vertical position
   */

  /*
   * - Tilts + regrips never cutable
   * - We can always cut two consecutive move without any regrips
   * - Out cut: no regrip or half turn + regrip
   * - In cut: no regrip
   * - Out cut: half-turn + regrip that starts in horizontal, half-turn + quarter-turn
   * - In cut: half turn + regrip that starts in vertical
   * - tFB F [rB] R is cutable if not [rF] in next move (potentially ignore this case if too hard)
   */
  
  // dp[len + 1][state::COUNT][regrip::COUNT]
  // pd[len + 1][state::COUNT][regrip::COUNT]

  const move::mask TILTMASK = move::bit(move::TRL) | move::bit(move::TFB);
  const move::mask HALFMASK =
    move::bit(move::R2) | move::bit(move::L2) | move::bit(move::R2L1) |
    move::bit(move::R2L2) | move::bit(move::R2L3) | move::bit(move::R1L2) || move::bit(move::R3L2)
  ;

  // Transition guaranteed to be possible
  const bool cut(int m1, int par1, int blog1, int state, int m2, int par2, int blog2) {
    // There are no cuts when tilts are involved
    if ((move::bit(m1) & TILTMASK) || (move::bit(m2) & TILTMASK))
      return false;

    cube c;
    set_state(c, state);

    if (m1 == move::G)
      return false; // we cannot cut out of any regrip
    if (m2 == move::G) { // TODO:

    }
    // Neither `m1` nor `m2` is a full regrip

    int ax1 = m1 / 15 - 1;
    int ax2 = m2 / 15 - 1;
    bool half1 = HALFMASK & (m1 % 15);
    bool half2 = HALFMASK & (m2 % 15);

    if (half1) {
      int otherax = 2 * !ax1;
      if (c.blocked[otherax] || c.blocked[otherax + 1])
        return false; // cannot cut when regripping into HT-face
    }

    if (!half2) {
      if (par2 != regrip::_)
        return false; // cannot cut into a non-HT with any kind of regrip
    } else {
      if (!half1) {
        int otherax = 2 * !ax2;

        // Cannot cut when we have to open a gripper before the HT
        if (c.blocked[otherax] && MOVE_CUBES[m1][regrip::_].blocked[otherax])
          return false;
        if (c.blocked[otherax + 1] && MOVE_CUBES[m1][regrip::_].blocked[otherax + 1])
          return false;

        // Cannot cut when current pivot was regripped in previous move
        if (par1 == regrip::A2) {
          bool piv1 = MOVE_CUBES[m2][par2].blocked[otherax] == 0;
          bool piv2 = MOVE_CUBES[m2][par2].blocked[otherax = 1] == 0;
          bool reg1 = !MOVE_CUBES[m1][regrip::_].blocked[otherax] && MOVE_CUBES[m1][par1].blocked[otherax];
          bool reg2 = !MOVE_CUBES[m1][regrip::_].blocked[otherax + 1] && MOVE_CUBES[m1][par1].blocked[otherax + 1];
          if (piv1 != piv2) { // no problem if we have 2 pivots
            if (piv1 && reg1 || piv2 && reg2)
              return false;
          }
        }
      }
    }

  }

  // Only supposed to be called with actually executable solutions (i.e. with correct stateset transitions)
  int optim(const std::vector<int>& sol, std::vector<int>& parg, std::vector<int>& blog) {
    if (!sol.size()) // best avoid any trouble
      return 0;

    int len = sol.size();
    parg.resize(len);
    blog.resize(len);
    int n_states = 2 * state::COUNT;

    int dp[len + 1][state::COUNT];
    int pd[len + 1][state::COUNT];
    int pd1[len + 1][state::COUNT];
    int pd2[len + 1][state::COUNT];
    std::fill(dp[0], dp[0] + (len + 1) * state::COUNT, -1);
    dp[0][0] = 0;

    int stateset = DEFAULTSET;
    for (int i = 0; i < sol.size(); i++) {
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
              if (dp[i][state] + score[state][m1][r] > dp[i + 1][state1]) {
                dp[i + 1][state1] = dp[i][state] + score[state][m1][r];
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
      if (dp[len][s] > dp[len][best_s])
        best_s = s;
    }
    int s = best_s;
    for (int i = len; i > 0; i--) {
      parg[i - 1] = pd1[i][s];
      if (sol[i - 1] == move::G)
        blog[i - 1] = pd2[i][s];
      s = pd[i][s];
    }

    return dp[len][best_s];
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
        // G move remains uninitialized
      }
    }

    cube c;
    cube tmp;

    for (int state = 0; state < state::COUNT; state++) {
      set_state(c, state);

      for (int m = 15; m < N_MOVES; m++) {
        int other_ax;
        if (m < move::COUNT_CUBE) { // there needs to be at least on support on the orthogonal axis
          other_ax = !(m / 15 - 1);
          if (c.blocked[2 * other_ax] && c.blocked[2 * other_ax + 1])
            continue;
        } else if (m >= move::COUNT - move::COUNT_GRIP) { // anchored face needs to be unblocked
          if (m == move::G)
            continue;
          int f = m - move::COUNT;
          if (c.blocked[f])
            continue;
        }

        for (int r = 0; r < regrip::COUNT; r++) {
          if (move_cubes[m][r] == INVALID)
            continue;
          mul(c, move_cubes[m][r], tmp);
          if (m < move::COUNT_CUBE) {
            if (tmp.blocked[2 * other_ax] && tmp.blocked[2 * other_ax + 1]) // stuff like "[rR] F2 [rL]" is impossible
              continue;
          }
          if (valid(tmp)) {
            int state1 = get_state(tmp);
            for (int stateset = 0; stateset < N_STATESETS; stateset++) {
              if (state::in(stateset, state))
                nextset[stateset][m] |= 1 << state1;
              if (state::in(stateset, state1))
                nextiset[stateset][m] |= 1 << state;
            }
          }
        }
      }
    }
    // Summarize different full regrips into a single G-move
    for (int stateset = 0; stateset < N_STATESETS; stateset++) {
      for (int m = move::COUNT; m < N_MOVES; m++) {
        nextset[stateset][move::G] |= nextset[stateset][m];
        nextiset[stateset][move::G] |= nextiset[stateset][m];
      }
    }

    std::string face_names[] = {"rR", "rL", "rF", "rB"};
    for (int m = 0; m < N_MOVES; m++) {
      for (int r = 0; r < regrip::COUNT; r++) {
        move_names[m][r] = "[";
        for (int i = 0; i < 4; i++) {
          if (move_cubes[m][r].blocked[i] && !move_cubes[m][0].blocked[i]) // blocked but not moved faces
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
        int other_ax;
        if (m < move::COUNT_CUBE) {
          other_ax = !(m / 15 - 1);
          if (c.blocked[2 * other_ax] && c.blocked[2 * other_ax + 1])
            continue;
        } else if (m >= move::COUNT - move::COUNT_GRIP) {
          if (m == move::G)
            continue;
          int f = m - move::COUNT;
          if (c.blocked[f])
            continue;
        }

        for (int r = 0; r < regrip::COUNT; r++) {
          if (move_cubes[m][r] != INVALID) {
            mul(c, move_cubes[m][r], tmp);
            if (m < move::COUNT_CUBE) {
              if (tmp.blocked[2 * other_ax] && tmp.blocked[2 * other_ax + 1])
                continue;
            }
            if (valid(tmp))
              nextstate[state][m][r] = get_state(tmp);
          }
        }
      }
    }

    for (int state = 0; state < state::COUNT; state++) {
      set_state(c, state);
      for (int m = 15; m < N_MOVES; m++) {
        if (m >= move::COUNT_CUBE && m < move::COUNT - move::COUNT_GRIP) { // tilts are always safe
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
