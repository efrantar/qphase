/**
 *          +--+-----+
 *          |U1|U2|U3|
 *          |--+--+--|
 *          |U4|U5|U6|
 *          |--+--+--|
 *          |U7|U8|U9|
 * +--+--+--+--+--+--+--+--+--+--+--+--+
 * |L1|L2|L3|F1|F2|F3|R1|R2|R3|B1|B2|B3|
 * |--+--+--|--+--+--|--+--+--|--+--+--|
 * |L4|L5|L6|F4|F5|F6|R4|R5|R6|B4|B5|B6|
 * |--+--+--|--+--+--|--+--+--|--+--+--|
 * |L7|L8|L9|F7|F8|F9|R7|R8|R9|B7|B8|B9|
 * +--+--+--+--+--+--+--+--+--+--+--+--+
 *          |D1|D2|D3|
 *          |--+--+--|
 *          |D4|D5|D6|
 *          |--+--+--|
 *          |D7|D8|D9|
 *          +--+--+--+
 *
 *  Face-order: U, R, F, D, L, B
 */

#ifndef __FACE__
#define __FACE__

#include <string>
#include "qubie.h"

namespace face {

  const int COUNT = 6;
  const int N_FACELETS = 54; // 9 * 6

  namespace color {
    const int U = 0;
    const int R = 1;
    const int F = 2;
    const int D = 3;
    const int L = 4;
    const int B = 5;

    const char NAMES[] = {'U', 'R', 'F', 'D', 'L', 'B'};
  }
  using namespace color;

  // All valid tilts of a cube
  const int PERMS[][6] = {
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

  void mul(const qubie::cube& c1, const qubie::cube& c2, qubie::cube& into); // multiply only faces

  int to_cubie(const std::string& s, qubie::cube& c);
  std::string from_cubie(const qubie::cube& c);

  void init();

}

#endif
