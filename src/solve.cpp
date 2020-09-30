#include "solve.h"

#include <algorithm>
#include <bitset>
#include <strings.h>
#include <thread>

#include "grip.h"
#include "prun.h"
#include "sym.h"

namespace solve {

  const int DIR_TILTS[] = {
    0, 0, 8, 8, 5, 5
  }; // inversion does not change axis permutation

  // If an inverse search ends in a different face permutation (but same symmetry class) as the default direction's,
  // we have to flip the tilt axes on conversion
  const bool FLIP_TILTS[] = {
      false, // {U, R, F, D, L, B},
      true,  // {U, B, R, D, F, L},
      false, // {U, L, B, D, R, F},
      true,  // {U, F, L, D, B, R},
      true,  // {R, U, B, L, D, F},
      false, // {F, U, R, B, D, L},
      true,  // {L, U, F, R, D, B},
      false, // {B, U, L, F, D, R},
      false, // {R, F, U, L, B, D},
      true,  // {B, R, U, F, L, D},
      false, // {L, B, U, R, F, D},
      true,  // {F, L, U, B, R, D},
      false, // {D, R, B, U, L, F},
      true,  // {D, F, R, U, B, L},
      false, // {D, L, F, U, R, B},
      true,  // {D, B, L, U, F, R},
      true,  // {R, D, F, L, U, B},
      false, // {B, D, R, F, U, L},
      true,  // {L, D, B, R, U, F},
      false, // {F, D, L, B, U, R},
      false, // {R, B, D, L, F, U},
      true,  // {F, R, D, B, L, U},
      false, // {L, F, D, R, B, U},
      true   // {B, L, D, F, R, U}
  };

  class Search {

    int dir; // ID of search direction
    const coordc& cube; // starting position
    int p1depth; // phase 1 search depth
    move::mask d0moves; // mask for initial moves to consider
    bool& done; // when to terminate the search
    int& lenlim; // only find strictly shorter solutions
    Engine& solver; // report solutions to

    /* Keep track of reconstructed edges that remain valid in the current search path */
    int uedges[50];
    int dedges[50];
    int edges_depth;

    int moves[50]; // current (partial) solution

    bool inv;
    int dirtilt;

  private:
    void phase1(
      int depth, int togo, int flip, int slice, int twist, int corners, int tilt, move::mask next, int stateset
    ); // phase 1 search; iterates through all solution with exactly `togo` moves
    bool phase2(
      int depth, int togo, int slice, int udedges2, int corners, int tilt, move::mask next, int stateset
    ); // phase 2 search; returns once any solution is found

  public:
    Search(
      int dir,
      const coordc& cube,
      int p1depth, move::mask d0moves,
      bool& done, int& lenlim, Engine& solver
    ) : dir(dir), cube(cube), p1depth(p1depth), d0moves(d0moves), done(done), lenlim(lenlim), solver(solver) {};
    void run(); // perform the search

  };

  void Search::run() {
    uedges[0] = cube.uedges;
    dedges[0] = cube.dedges;
    inv = dir & 1;
    dirtilt = tilt::coord_cls[cube.tilt];

    if (inv) {
      p1depth--; // since we +1ed to avoid biasing the search towards inverse directions
      for (int stilt = 0; stilt < 3; stilt++) {
        int tilt = tilt::coord_rep[stilt];
        move::mask next;
        if (prun::get_phase1(cube.flip, cube.slice, cube.twist, tilt, p1depth, next) <= p1depth) {
          next &= move::p1mask & tilt::moves[tilt] & d0moves; // select current search split
          edges_depth = 0;
          // We may start with any legal gripper state
          phase1(0, p1depth, cube.flip, cube.slice, cube.twist, cube.corners, tilt, next, grip::N_STATESETS - 1);
        }
      }
    } else {
      move::mask next;
      prun::get_phase1(cube.flip, cube.slice, cube.twist, cube.tilt, p1depth, next);
      next &= move::p1mask & tilt::moves[cube.tilt] & d0moves; // select current search split
      edges_depth = 0;
      phase1(0, p1depth, cube.flip, cube.slice, cube.twist, cube.corners, cube.tilt, next, 1);
    }
  }

  void Search::phase1(
    int depth, int togo, int flip, int slice, int twist, int corners, int tilt, move::mask next, int stateset
  ) {
    if (done)
      return;
    if (togo == 0) {
      int tmp = prun::get_precheck(corners, slice, tilt, inv, dirtilt);
      if (tmp >= lenlim - depth) // phase 2 precheck, only reconstruct edges if successful
        return;

      for (int i = edges_depth + 1; i <= depth; i++) {
        uedges[i] = coord::move_edges4[uedges[i - 1]][moves[i - 1]];
        dedges[i] = coord::move_edges4[dedges[i - 1]][moves[i - 1]];
      }
      edges_depth = depth - 1;
      int udedges2 = coord::merge_udedges2(uedges[depth], dedges[depth]);

      for (int togo1 = std::max(prun::get_phase2(corners, udedges2, tilt, inv, dirtilt), tmp); togo1 < lenlim - depth; togo1++) {
        if (phase2(depth, togo1, slice, udedges2, corners, tilt, move::p2mask & tilt::moves[tilt] & move::next[moves[depth - 1]], stateset))
          return; // once we have found a phase 2 solution, there cannot be any shorter ones -> quit
      }
      return;
    }

    bool regrip = true;
    depth++;
    togo--;

    while (next) {
      int m = ffsll(next) -  1; // get rightmost move index (`ffsll()` uses 1-based indexing)
      next &= next - 1;

      int stateset1;
      if (inv)
        stateset1 = grip::nextiset[stateset][tilt::trans_move[tilt][m]];
      else
        stateset1 = grip::nextset[stateset][tilt::trans_move[tilt][m]];
      if (!stateset1)
        continue;

      int flip1 = coord::move_flip[flip][m];
      int slice1 = coord::move_edges4[slice][m];
      int twist1 = coord::move_twist[twist][m];
      int tilt1 = tilt::move_coord[tilt][m];
      move::mask next1;
      int dist1 = prun::get_phase1(flip1, slice1, twist1, tilt1, togo, next1);

      // Check inside loop to avoid unnecessary recursion unwinds
      if (dist1 == togo || dist1 + togo >= 5) { // Rokicki optimization
        if (m == move::G && !regrip) // only explore forced regrips
          continue;
        regrip = false;

        int corners1 = coord::move_corners[corners][m];
        moves[depth - 1] = m;
        next1 &= move::p1mask & move::next[m] & tilt::moves[tilt1];
        phase1(depth, togo, flip1, slice1, twist1, corners1, tilt1, next1, stateset1);
      }
    }

    // We always want to maintain the maximum number of already reconstructed EDGES coordinates, hence we only
    // decrement when the depth level gets lower than the current valid index (note that we will typically also
    // visit other deeper branches in between that might not have any effect on this)
    if (edges_depth == depth - 1)
      edges_depth--;
  }

  bool Search::phase2(
    int depth, int togo, int slice, int udedges2, int corners, int tilt, move::mask next, int stateset
  ) {
    if (togo == 0) {
      if (slice != coord::N_SLICE2 * coord::SLICE1_SOLVED) // check if SLICE2 is also solved
        return false;
      if (inv && !(stateset & 1)) // neutral state must be reachable when we want to invert the solution
        return false;

      searchres sol = {std::vector<int>(depth), (dir << 1) | (inv ? FLIP_TILTS[tilt] : 0) };
      for (int i = 0; i < depth; i++)
        sol.first[i] = moves[i];
      solver.report_sol(sol);

      return true; // we will not find any shorter solutions
    }

    bool regrip = true;

    while (next) {
      int m = ffsll(next) -  1; // get rightmost move index (`ffsll()` uses 1-based indexing)
      next &= next - 1;

      int stateset1;
      if (inv)
        stateset1 = grip::nextiset[stateset][tilt::trans_move[tilt][m]];
      else
        stateset1 = grip::nextset[stateset][tilt::trans_move[tilt][m]];
      if (!stateset1)
        continue;

      int slice1 = coord::move_edges4[slice][m];
      int udedges21 = coord::move_udedges2[udedges2][m];
      int corners1 = coord::move_corners[corners][m];
      int tilt1 = tilt::move_coord[tilt][m];

      if (prun::get_phase2(corners1, udedges21, tilt1, inv, dirtilt) < togo) {
        if (m == move::G && !regrip)
          continue;
        regrip = false;

        moves[depth] = m;
        move::mask next1 = move::p2mask & move::next[m] & tilt::moves[tilt1];
        if (phase2(depth + 1, togo - 1, slice1, udedges21, corners1, tilt1, next1, stateset1))
          return true; // return as soon as we have a solution
      }
    }

    return false;
  }

  Engine::Engine(
    int n_threads, int tlim,
    int n_sols, int max_len, int n_splits
  ) : n_threads(n_threads), tlim(tlim), n_sols(n_sols), max_len(max_len), n_splits(n_splits) {
    for (int stilt = 0; stilt < tilt::N_COORD_SYM; stilt++) {
      int n_moves = std::bitset<64>(tilt::moves[tilt::coord_rep[stilt]]).count();
      int tmp = (n_moves + n_splits - 1) / n_splits; // ceil to make sure that we always include all moves
      move::mask moves = tilt::moves[tilt::coord_rep[stilt]];
      for (int i = 0; i < n_splits; i++) {
        masks[stilt][i] = 0;
        for (int j = 0; j < tmp && moves; j++) {
          masks[stilt][i] |= move::bit(ffsll(moves) - 1);
          moves &= moves - 1;
        }
      }
    }
    done = true; // make sure that the first `prepare()` will actually do something
  }

  void Engine::thread() {
    int mindir = 0;
    do {
      /* Select next job to execute; don't forget to lock */
      job_mtx.lock();
      for (int dir = 0; dir < N_DIRS; dir++) {
        if (depths[dir] < depths[mindir])
          mindir = dir;
      }
      int split = splits[mindir]++;
      int togo = depths[mindir];
      if (splits[mindir] == n_splits) {
        depths[mindir]++;
        splits[mindir] = 0;
      }
      job_mtx.unlock();

      Search search(mindir, dirs[mindir], togo, masks[tilt::coord_cls[dirs[mindir].tilt]][split], done, lenlim, *this);
      search.run();
    } while (!done); // we should never actually get to the truly optimal depth anyways in general
  }

  void Engine::prepare() {
    if (!done) // avoid double preparation
      return;
    finish();

    job_mtx.lock(); // make spawned threads wait for initialization of the cube to be solved
    for (int i = 0; i < n_threads; i++)
      threads.push_back(std::thread([&]() { this->thread(); }));

    done = false;
    lenlim = max_len > 0 ? max_len + 1: 50; // only search for strictly shorter solutions than this
    // `sols` is always emptied after a solve
  }

  void Engine::solve(const cubie::cube& c, std::vector<std::vector<int>>& res) {
    prepare(); // make sure we are prepared; will do nothing if that should already be the case

    cubie::cube tmp1, tmp2;
    cubie::cube invc;
    cubie::inv(c, invc);

    for (int dir = 0; dir < N_DIRS; dir++) {
      const cubie::cube& c1 = (dir & 1) ? invc : c; // reference is enough, we do not need to copy
      int rot = sym::ROT * (dir / 2);
      cubie::mul(sym::cubes[sym::inv[rot]], c1, tmp1);
      cubie::mul(tmp1, sym::cubes[rot], tmp2);

      dirs[dir].flip = coord::get_flip(tmp2);
      dirs[dir].slice = coord::get_slice(tmp2);
      dirs[dir].twist = coord::get_twist(tmp2);
      dirs[dir].uedges = coord::get_uedges(tmp2);
      dirs[dir].dedges = coord::get_dedges(tmp2);
      dirs[dir].corners = coord::get_corners(tmp2);
      dirs[dir].tilt = DIR_TILTS[dir];

      move::mask tmp; // simply ignore, makes no sense anyways without proper `togo`
      if (dir & 1) { // inverse searches may start the phase 1 search with any of the tilt classes
        depths[dir] = 100;
        for (int stilt = 0; stilt < 3; stilt++) {
          depths[dir] = std::min(
            depths[dir],
            prun::get_phase1(dirs[dir].flip, dirs[dir].slice, dirs[dir].twist, tilt::coord_rep[stilt], 100, tmp)
          );
        }
        depths[dir]++; // to not bias the search towards inverse searches
      } else
        depths[dir] = prun::get_phase1(dirs[dir].flip, dirs[dir].slice, dirs[dir].twist, dirs[dir].tilt, 100, tmp);
      splits[dir] = 0;
    }

    job_mtx.unlock(); // start solving

    { // timeout
      std::unique_lock<std::mutex> lock(tout_mtx);
      tout_cvar.wait_for(lock, std::chrono::milliseconds(tlim), [&]{ return done; });
      if (!done)
        done = true; // if we get here, this was a timeout
    }
    std::lock_guard<std::mutex> lock(sol_mtx); // make sure no thread is writing any more solutions

    res.resize(sols.size());
    for (int i = 0; i < res.size(); i++) {
      const searchres& sol = sols.top();
      res[i].resize(sol.first.size());

      bool flip = sol.second & 1;
      bool inv = (sol.second >> 1) & 1;
      int rot = sym::ROT * (sol.second >> 2);

      for (int j = 0; j < res[i].size(); j++) // undo rotation
        res[i][j] = sol.first[j] < move::COUNT_CUBE ? sym::conj_move[sol.first[j]][rot] : sol.first[j];
      if (inv) { // undo inversion
        for (int j = 0; j < res[i].size(); j++) {
          if (res[i][j] < move::COUNT_CUBE)
            res[i][j] = move::inv[res[i][j]];
          else if (flip && res[i][j] != move::G)
            res[i][j] = move::COUNT_CUBE + !(res[i][j] - move::COUNT_CUBE);
        }
        std::reverse(res[i].begin(), res[i].end());
      }

      int tilt = 0;
      for (int j = 0; j < res[i].size(); j++) {
        int tmp = res[i][j];
        res[i][j] = tilt::trans_move[tilt][res[i][j]];
        tilt = tilt::move_coord[tilt][tmp];
      }

      sols.pop();
    }
    std::reverse(res.begin(), res.end()); // return solutions in order of increasing length
  }

  void Engine::report_sol(searchres& sol) {
    std::lock_guard<std::mutex> lock(sol_mtx);

    if (done) // prevent any type of reporting after the solver has terminated (important for threading)
      return;

    sols.push(sol); // usually we only get here if we actually have a solution that will be added
    if (sols.size() > n_sols)
      sols.pop();
    if (sols.size() == n_sols) {
      lenlim = sols.top().first.size(); // only search for strictly shorter solutions

      if (lenlim <= max_len) { // already found a solution that is short enough
        done = true; // end searching
        // Wake up timeout
        std::lock_guard<std::mutex> lock(tout_mtx);
        tout_cvar.notify_one();
      }
    }
  }

  void Engine::finish() {
    for (std::thread& t : threads) // wait for all existing threads to actually finish
      t.join();
    threads.clear(); // they are now invalid
  }

}
