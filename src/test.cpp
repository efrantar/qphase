/**
 * Tests & sanity checks for individual parts of the solver; very useful during development.
 */

#include <bitset>
#include <chrono>
#include <iostream>
#include <strings.h>

#include "coord.h"
#include "face.h"
#include "move.h"
#include "prun.h"
#include "qubie.h"
#include "sym.h"

inline void ok() { std::cout << "Ok." << std::endl; }
inline void error() { std::cout << "Error." << std::endl; }

void test_qubie() {
  std::cout << "Testing qubie level ..." << std::endl;
  qubie::cube c = qubie::ID_CUBE;

  qubie::cube tmp1, tmp2;
  qubie::inv(c, tmp1);
  if (c != tmp1)
    error();
  qubie::mul(c, tmp1, tmp2);
  if (c != tmp2)
    error();

  qubie::shuffle(c);
  qubie::inv(c, tmp1);
  qubie::mul(c, tmp1, tmp2);
  if (tmp2 != qubie::ID_CUBE)
    error();

  for (int i = 0; i < 100; i++) {
    qubie::shuffle(c);
    if (qubie::check(c) != 0)
      error();
  }
  qubie::shuffle(c);
  std::swap(c.cperm[0], c.cperm[1]);
  if (check(c) == 0)
    error();

  ok();
}

void test_getset(int (*get_coord)(const qubie::cube&), void (*set_coord)(qubie::cube&, int), int count) {
  qubie::cube c;
  for (int i = 0; i < count; i++) {
    set_coord(c, i);
    if (get_coord(c) != i)
      error();
  }
  ok();
}

void test_movecoord(uint16_t move_coord[][move::COUNT], int n_coord, move::mask moves = move::p1mask | move::p2mask) {
  for (int coord = 0; coord < n_coord; coord++) {
    for (; moves; moves &= moves - 1) {
      int m = ffsll(moves) - 1;
      if (move_coord[move_coord[coord][m]][move::inv[m]] != coord)
        error();
      if (move_coord[move_coord[coord][move::inv[m]]][m] != coord)
        error();
    }
  }
  ok();
}

void test_coord() {
  std::cout << "Testing coord level ..." << std::endl;
  test_getset(coord::get_flip, coord::set_flip, coord::N_FLIP);
  test_getset(coord::get_twist, coord::set_twist, coord::N_TWIST);
  test_getset(coord::get_slice, coord::set_slice, coord::N_SLICE);
  test_getset(coord::get_uedges, coord::set_uedges, coord::N_UEDGES);
  test_getset(coord::get_dedges, coord::set_dedges, coord::N_DEDGES);
  test_getset(coord::get_corners, coord::set_corners, coord::N_CORNERS);
  test_getset(coord::get_tilt, coord::set_tilt, coord::N_TILT);

  test_getset(coord::get_slice1, coord::set_slice1, coord::N_SLICE1);
  test_getset(coord::get_udedges2, coord::set_udedges2, coord::N_UDEDGES2);

  test_movecoord(coord::move_flip, coord::N_FLIP);
  test_movecoord(coord::move_twist, coord::N_TWIST);
  test_movecoord(coord::move_edges4, coord::N_SLICE);
  test_movecoord(coord::move_corners, coord::N_CORNERS);
  test_movecoord(coord::move_udedges2, coord::N_UDEDGES2, move::p2mask);
}

void test_move() {
  std::cout << "Testing move level ..." << std::endl;

  qubie::cube c;
  for (int m = 0; m < move::COUNT; m++) {
    if (move::inv[move::inv[m]] != m)
      error();
    qubie::mul(move::cubes[m], move::cubes[move::inv[m]], c);
    if (c != qubie::ID_CUBE)
      error();
    qubie::mul(move::cubes[move::inv[m]], move::cubes[m], c);
    if (c != qubie::ID_CUBE)
      error();
  }
  ok();

  std::cout << "Phase 1: ";
  for (int m = 0; m < move::COUNT; m++) {
    if (move::in(m, move::p1mask))
      std::cout << move::names[m] << " ";
  }
  std::cout << std::endl;
  std::cout << "Phase 2: ";
  for (int m = 0; m < move::COUNT; m++) {
    if (move::in(m, move::p2mask))
      std::cout << move::names[m] << " ";
  }
  std::cout << std::endl;

  std::cout << "Forbidden:" << std::endl;
  for (int m = 0; m < move::COUNT; m++) {
    std::cout << move::names[m] << ": ";
    for (int m1 = 0; m1 < move::COUNT; m1++) {
      if (!move::in(m1, move::next[m]))
        std::cout << move::names[m1] << " ";
    }
    if (move::qt_skip[m] != 0) {
      std::cout << "| ";
      for (int m1 = 0; m1 < move::COUNT; m1++) {
        if (move::in(m1, move::qt_skip[m]))
          std::cout << move::names[m1] << " ";
      }
    }
    std::cout << std::endl;
  }

}

void test_conj(uint16_t conj_coord[][sym::COUNT_SUB], int n_coord) {
  for (int coord = 0; coord < n_coord; coord++) {
    for (int s = 0; s < sym::COUNT_SUB; s++) {
      if (conj_coord[conj_coord[coord][s]][sym::inv[s]] != coord)
        error();
      if (conj_coord[conj_coord[coord][sym::inv[s]]][s] != coord)
        error();
    }
  }
  ok();
}

void test_sym() {
  std::cout << "Testing sym level ..." << std::endl;
  test_conj(sym::conj_twist, coord::N_TWIST);
  test_conj(sym::conj_udedges2, coord::N_UDEDGES2);
}

/*
void test_prun() {
  std::cout << "Testing pruning ..." << std::endl;

  srand(0);
  int n_moves = std::bitset<64>(move::p1mask).count(); // make sure not to consider B-moves in F5-mode

  for (int i = 0; i < 1000; i++) {
    int flip = rand() % coord::N_FLIP;
    int slice = rand() % coord::N_SLICE;
    int twist = rand() % coord::N_TWIST;

    move::mask next;
    move::mask next1;
    move::mask tmp;
    int togo = prun::get_phase1(flip, slice, twist, 100, tmp);

    int dist = prun::get_phase1(flip, slice, twist, togo, next);
    next &= move::p1mask;

    next1 = 0;
    for (int m = 0; m < n_moves; m++) {
      int flip1 = coord::move_flip[flip][m];
      int slice1 = coord::move_edges4[slice][m];
      int twist1 = coord::move_twist[twist][m];
      if (prun::get_phase1(flip1, slice1, twist1, 100, tmp) < dist)
        next1 |= move::bit(m);
    }
    if (next1 != next)
      error();

    dist = prun::get_phase1(flip, slice, twist, togo + 1, next);
    next &= move::p1mask;

    next1 = 0;
    for (int m = 0; m < n_moves; m++) {
      int flip1 = coord::move_flip[flip][m];
      int slice1 = coord::move_edges4[slice][m];
      int twist1 = coord::move_twist[twist][m];
      if (prun::get_phase1(flip1, slice1, twist1, 100, tmp) <= dist)
        next1 |= move::bit(m);
    }
    if (next1 != next)
      error();
  }

  ok();
}
*/

bool check(const qubie::cube &c, const std::vector<int>& sol) {
  qubie::cube c1;
  qubie::cube c2;

  c1 = c;
  for (int m : sol) {
    qubie::mul(c1, move::cubes[m], c2);
    std::swap(c1, c2);
  }

  return c1 == qubie::ID_CUBE;
}

using namespace qubie;
using namespace face::color;

int main() {
  auto tick = std::chrono::high_resolution_clock::now();
  move::init();
  coord::init();

  qubie::cube x = {
    {URF, UFL, ULB, UBR, DFR, DLF, DBL, DRB},
    {UR, UF, UL, UB, DR, DF, DL, DB, FR, FL, BL, BR},
    {}, {},
    {F, R, D, B, L, U}
  };
  qubie::cube y = {
    {URF, UFL, ULB, UBR, DFR, DLF, DBL, DRB},
    {UR, UF, UL, UB, DR, DF, DL, DB, FR, FL, BL, BR},
    {}, {},
    {R, D, F, L, U, B}
  };
  qubie::cube z = {
    {URF, UFL, ULB, UBR, DFR, DLF, DBL, DRB},
    {UR, UF, UL, UB, DR, DF, DL, DB, FR, FL, BL, BR},
    {}, {},
    {U, D, R, D, F, L}
  };

  sym::init();
  // prun::init();
  std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - tick).count() / 1000. << "ms" << std::endl;

  test_qubie();
  test_coord();
  // test_move();
  test_sym();
  // test_prun();

  return 0;
}
