#include "qubie.h"

#include <algorithm>
#include <random>
#include "coord.h"
#include "face.h"

namespace qubie {

  /* Faster than tricky if-else sequences for handling mirrored states */
  int mul_coris[][6] = {
    {0, 1, 2, 3, 4, 5},
    {1, 2, 0, 4, 5, 3},
    {2, 0, 1, 5, 3, 4},
    {3, 5, 4, 0, 2, 1},
    {4, 3, 5, 1, 0, 2},
    {5, 4, 3, 2, 1, 0}
  };
  int inv_cori[] = {
    0, 2, 1, 3, 4, 5
  };

  std::random_device device;
  std::mt19937 gen(device());

  void corner::mul(const qubie::cube& c1, const qubie::cube& c2, qubie::cube& into) {
    for (int i = 0; i < corner::COUNT; i++) {
      into.cperm[i] = c1.cperm[c2.cperm[i]];
      into.cori[i] = mul_coris[c1.cori[c2.cperm[i]]][c2.cori[i]];
    }
  }

  void edge::mul(const qubie::cube& c1, const qubie::cube& c2, qubie::cube& into) {
    for (int i = 0; i < edge::COUNT; i++) {
      into.eperm[i] = c1.eperm[c2.eperm[i]];
      into.eori[i] = (c1.eori[c2.eperm[i]] + c2.eori[i]) & 1;
    }
  }

  void mul(const qubie::cube& c1, const qubie::cube& c2, qubie::cube& into) {
    corner::mul(c1, c2, into);
    edge::mul(c1, c2, into);
    for (int i = 0; i < face::COUNT; i++)
      into.fperm[i] = c1.fperm[c2.fperm[i]];
  }

  // Permutation parity =  #inversions % 2
  bool parity(const int perm[], int len) {
    int par = 0;
    for (int i = 0; i < len; i++) {
      for (int j = 0; j < i; j++) {
        if (perm[j] > perm[i])
          par++;
      }
    }
    return par & 1;
  }

  void inv(const cube& c, cube& into) {
    for (int corner = 0; corner < corner::COUNT; corner++)
      into.cperm[c.cperm[corner]] = corner; // inv[a[i]] = i
    for (int edge = 0; edge < edge::COUNT; edge++)
      into.eperm[c.eperm[edge]] = edge;
    for (int i = 0; i < corner::COUNT; i++)
      into.cori[i] = inv_cori[c.cori[into.cperm[i]]];
    for (int i = 0; i < edge::COUNT; i++)
      into.eori[i] = c.eori[into.eperm[i]];
    for (int face = 0; face < face::COUNT; face++)
      into.fperm[c.fperm[face]] = face;
  }

  int check(const cube& c) {
    bool corners[corner::COUNT] = {};
    int cori_sum = 0;

    for (int i = 0; i < corner::COUNT; i++) {
      if (c.cperm[i] < 0 || c.cperm[i] >= corner::COUNT)
        return 1; // invalid corner cubie
      corners[c.cperm[i]] = true;
      if (c.cori[i] < 0 || c.cori[i] >= 3)
        return 2; // invalid corner orientation
      cori_sum += c.cori[i];
    }
    if (cori_sum % 3 != 0)
      return 3; // invalid twist parity
    for (bool corner : corners) {
      if (!corner)
        return 4; // missing corner
    }

    bool edges[edge::COUNT] = {};
    int eori_sum = 0;

    for (int i = 0; i < edge::COUNT; i++) {
      if (c.eperm[i] < 0 || c.eperm[i] >= edge::COUNT)
        return 5; // invalid edge cubie
      edges[c.eperm[i]] = true;
      if (c.eori[i] < 0 || c.eori[i] >= 2)
        return 6; // invalid edge orientation
      eori_sum += c.eori[i];
    }
    if ((eori_sum & 1) != 0)
      return 7; // invalid flip parity
    for (bool edge : edges) {
      if (!edge)
        return 8; // missing edge
    }

    if (parity(c.cperm, corner::COUNT) != parity(c.eperm, edge::COUNT))
      return 9; // corner and edge permutation parity mismatch

    bool ok = false;
    for (int i = 0; i < coord::N_TILT; i++) {
      // Note that the default tilt is the first in the permutation list, thus this will break immediately
      if (std::equal(c.fperm, c.fperm + face::COUNT, face::PERMS[i])) {
        ok = true;
        break;
      }
    }
    if (!ok)
      return 10; // invalid face permutation

    return 0;
  }

  void shuffle(cube& c) {
    std::iota(c.cperm, c.cperm + corner::COUNT, 0);
    std::iota(c.eperm, c.eperm + edge::COUNT, 0);

    coord::set_corners(c, std::uniform_int_distribution<int>(0, coord::N_CORNERS)(gen));
    std::shuffle(c.eperm, c.eperm + edge::COUNT, gen); // no coordinate for all edges
    if (parity(c.cperm, corner::COUNT) != parity(c.eperm, edge::COUNT))
      std::swap(c.cperm[corner::COUNT - 2], c.cperm[corner::COUNT - 1]); // flip parity

    coord::set_twist(c, std::uniform_int_distribution<int>(0, coord::N_TWIST - 1)(gen));
    coord::set_flip(c, std::uniform_int_distribution<int>(0, coord::N_FLIP - 1)(gen));

    std::iota(c.fperm, c.fperm + face::COUNT, 0);
  }

  bool operator==(const cube& c1, const cube& c2) {
    return
      std::equal(c1.cperm, c1.cperm + corner::COUNT, c2.cperm) &&
      std::equal(c1.eperm, c1.eperm + edge::COUNT, c2.eperm) &&
      std::equal(c1.cori, c1.cori + corner::COUNT, c2.cori) &&
      std::equal(c1.eori, c1.eori + edge::COUNT, c2.eori) &&
      std::equal(c1.fperm, c1.fperm + face::COUNT, c2.fperm)
    ;
  }

  bool operator!=(const cube& c1, const cube& c2) {
    return !(c1 == c2);
  }

}
