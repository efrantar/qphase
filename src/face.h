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
#include <unordered_map>
#include "cubie.h"

namespace face {

  const int N_FACELETS = 54; // 9 * 6

  namespace color {
    const int COUNT = 6;

    const int U = 0;
    const int R = 1;
    const int F = 2;
    const int D = 3;
    const int L = 4;
    const int B = 5;

    const char NAMES[] = {'U', 'R', 'F', 'D', 'L', 'B'};

    const std::unordered_map<char, int> FROM_NAME = {
      {'U', U}, {'R', R}, {'F', F}, {'D', D}, {'L', L}, {'B', B}
    };
  }

  /* Facelet positions corresponding to cubies */
  const int CORNLETS[][3] = {
    {8, 9, 20}, {6, 18, 38}, {0, 36, 47}, {2, 45, 11},
    {29, 26, 15}, {27, 44, 24}, {33, 53, 42}, {35, 17, 51}
  };
  const int EDGELETS[][2] = {
    {5, 10}, {7, 19}, {3, 37}, {1, 46}, {32, 16}, {28, 25},
    {30, 43}, {34, 52}, {23, 12}, {21, 41}, {50, 39}, {48, 14}
  };

  int to_cubie(const std::string& s, cubie::cube &c);
  std::string from_cubie(const cubie::cube &c);

  void init();

}

#endif
