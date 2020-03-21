/**
 * Cubie level.
 */

#ifndef __CUBIE__
#define __CUBIE__

#include <string>

namespace qubie {

  /* All qubie definitions are plain integers to making uniform handling much easier */

  namespace corner {
    const int COUNT = 8;

    const int URF = 0;
    const int UFL = 1;
    const int ULB = 2;
    const int UBR = 3;
    const int DFR = 4;
    const int DLF = 5;
    const int DBL = 6;
    const int DRB = 7;

    const std::string NAMES[] = {
      "URF", "UFL", "ULB", "UBR", "DFR", "DLF", "DBL", "DRB"
    };
  }
  using namespace corner;

  namespace edge {
    const int COUNT = 12;

    const int UR = 0;
    const int UF = 1;
    const int UL = 2;
    const int UB = 3;
    const int DR = 4;
    const int DF = 5;
    const int DL = 6;
    const int DB = 7;
    // SLICE-edges last s.t. UDEDGES2 is easier to handle
    const int FR = 8;
    const int FL = 9;
    const int BL = 10;
    const int BR = 11;

    const std::string NAMES[] = {
      "UR", "UF", "UL", "UB", "DR", "DF", "DL", "DB", "FR", "FL", "BL", "BR"
    };
  }
  using namespace edge;

  struct cube {
    int cperm[corner::COUNT]; // corner qubie permutation
    int eperm[edge::COUNT]; // edge qubie permutation
    int cori[corner::COUNT]; // corner qubie orientation; 0 if U/D-facelet on U/D-face; 1 clockwise rot; 2 c-clock
    int eori[edge::COUNT]; // edge qubie orientation; 0 if U/D-facelet on U/D-face or same for F/B for slice edges

    int fperm[6]; // face permutation
  };

  const cube ID_CUBE = {
    {URF, UFL, ULB, UBR, DFR, DLF, DBL, DRB},
    {UR, UF, UL, UB, DR, DF, DL, DB, FR, FL, BL, BR},
    {}, {},
    {0, 1, 2, 3, 4, 5}
  }; // identity cubie-cube, i.e. solved cube with default face ordering

  /* Explicitly pass result cube to avoid unnecessary copying during table generation */

  namespace corner {
    void mul(const cube& c1, const cube& c2, cube& into); // multiply only corner cubies
  }
  namespace edge {
    void mul(const cube& c1, const cube& c2, cube& into); // multiply only edge cubies
  }

  void mul(const cube& c1, const cube& c2, cube& into, bool tilt = true); // fully multiply two cubes
  void inv(const cube& c, cube& into, bool tilt = true);
  void shuffle(cube& c); // generate a uniformly random cube with default face ordering
  int check(const cube& c, bool tilt = true); // check a cube for being solvable

  bool cubie_equal(const cube& c1, const cube& c2); // equality only in terms of cubies
  bool operator==(const cube& c1, const cube& c2);
  bool operator!=(const cube& c1, const cube& c2);

}

#endif
