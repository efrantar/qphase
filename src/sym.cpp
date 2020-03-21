#include "sym.h"

#include "face.h"

namespace sym {

  using namespace qubie::corner;
  using namespace qubie::edge;
  using namespace face::color;

  const uint32_t EMPTY = ~uint32_t(0);

  qubie::cube cubes[COUNT];
  int inv[COUNT];
  int effect[COUNT][3];

  int conj_move[move::COUNT][COUNT];
  uint16_t conj_twist[coord::N_TWIST][COUNT_SUB];
  uint16_t conj_udedges2[coord::N_UDEDGES2][COUNT_SUB];
  int conj_tilt[coord::N_TILT][COUNT_SUB];

  uint32_t fslice1_sym[coord::N_FSLICE1];
  uint32_t corners_sym[coord::N_CORNERS];
  int tilt_sym[coord::N_TILT];
  uint32_t fslice1_raw[N_FSLICE1];
  uint16_t corners_raw[N_CORNERS];
  int tilt_raw[N_TILT];
  uint16_t fslice1_selfs[N_FSLICE1];
  uint16_t corners_selfs[N_CORNERS];

  void init_base() {
    qubie::cube c = qubie::ID_CUBE;
    qubie::cube tmp;

    qubie::cube f2 = {
      {DLF, DFR, DRB, DBL, UFL, URF, UBR, ULB},
      {DL, DF, DR, DB, UL, UF, UR, UB, FL, FR, BR, BL},
      {}, {},
      {D, L, F, U, R, B}
    };
    qubie::cube u4 = {
      {UBR, URF, UFL, ULB, DRB, DFR, DLF, DBL},
      {UB, UR, UF, UL, DB, DR, DF, DL, BR, FR, FL, BL},
      {}, {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1},
      {U, B, R, D, F, L}
    };
    qubie::cube lr2 = {
      {UFL, URF, UBR, ULB, DLF, DFR, DRB, DBL},
      {UL, UF, UR, UB, DL, DF, DR, DB, FL, FR, BR, BL},
      {3, 3, 3, 3, 3, 3, 3, 3}, {}, // special mirror ori
      {U, R, F, D, L, B} // do not affect the tilt state
    };
    qubie::cube urf3 = {
      {URF, DFR, DLF, UFL, UBR, DRB, DBL, ULB},
      {UF, FR, DF, FL, UB, BR, DB, BL, UR, DR, DL, UL},
      {1, 2, 1, 2, 2, 1, 2, 1},
      {1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1},
      {F, U, R, B, D, L}
    };

    // First 8 symmetries encode cube rotations also used for reducing the tilt
    for (int i = 0; i < COUNT; i++) {
      cubes[i] = c;

      qubie::mul(c, f2, tmp);
      std::swap(tmp, c);

      if (i % 2 == 1) {
        qubie::mul(c, u4, tmp);
        std::swap(tmp, c);
      }
      if (i % 8 == 7) {
        qubie::mul(c, lr2, tmp);
        std::swap(tmp, c);
      }
      if (i % 16 == 15) {
        qubie::mul(c, urf3, tmp);
        std::swap(tmp, c);
      }
    }

    // TODO: somehow do equality without
    for (int i = 0; i < COUNT; i++) {
      for (int j = 0; j < COUNT; j++) {
        qubie::mul(cubes[i], cubes[j], c);
        if (c == qubie::ID_CUBE) {
          inv[i] = j;
          break;
        }
      }
    }

    for (int m = 0; m < move::COUNT; m++) {
      for (int s = 0; s < COUNT; s++) {
        qubie::mul(cubes[s], move::cubes[m], tmp);
        qubie::mul(tmp, cubes[inv[s]], c);
        for (int conj = 0; conj < move::COUNT; conj++) {
          if (c == move::cubes[conj]) {
            conj_move[m][s] = conj;
            break;
          }
        }
      }
    }

    /* Figure this out right here instead of defining even more "weird" constants */
    int per_axis = move::COUNT1 / 3;
    int per_face = 3;
    #ifdef QT
      per_face -= 1;
    #endif
    for (int s = 0; s < COUNT; s++) {
      for (int ax = 0; ax < 3; ax++) {
        effect[s][ax] = (conj_move[per_axis * ax][inv[s]] / per_axis) << 2; // shift
        effect[s][ax] |= (conj_move[per_axis * ax][inv[s]] % per_axis >= per_face) << 1; // flip
        effect[s][ax] |= (conj_move[per_axis * ax][inv[s]] % per_face != 0); // inv
      }
    }
  }

  void init_conjcoord(
    uint16_t conj_coord[][COUNT_SUB],
    int n_coords,
    int (*get_coord)(const qubie::cube&),
    void (*set_coord)(qubie::cube&, int),
    void (*mul)(const qubie::cube&, const qubie::cube&, qubie::cube&)
  ) {
    qubie::cube c1 = qubie::ID_CUBE; // make sure all multiplications will work
    qubie::cube c2;
    qubie::cube tmp;

    for (int coord = 0; coord < n_coords; coord++) {
      set_coord(c1, coord);
      conj_coord[coord][0] = coord; // sym 0 is identity
      for (int s = 1; s < COUNT_SUB; s++) {
        mul(cubes[s], c1, tmp);
        mul(tmp, cubes[inv[s]], c2);
        conj_coord[coord][s] = get_coord(c2);
      }
    }
  }

  void init_fslice1() {
    std::fill(fslice1_sym, fslice1_sym + coord::N_FSLICE1, EMPTY);

    qubie::cube c1 = qubie::ID_CUBE;
    qubie::cube c2;
    qubie::cube tmp;
    int cls = 0;

    for (int slice1 = 0; slice1 < coord::N_SLICE1; slice1++) {
      coord::set_slice1(c1, slice1); // SLICE is slightly more expensive to set
      for (int flip = 0; flip < coord::N_FLIP; flip++) {
        coord::set_flip(c1, flip);
        int fslice1 = coord::fslice1(flip, slice1);

        if (fslice1_sym[fslice1] != EMPTY)
          continue;
        fslice1_sym[fslice1] = COUNT_SUB * cls;
        fslice1_raw[cls] = fslice1;
        fslice1_selfs[cls] = 1; // symmetry 0 is identity and always a self-sym

        for (int s = 1; s < COUNT_SUB; s++) {
          qubie::edge::mul(cubes[inv[s]], c1, tmp);
          qubie::edge::mul(tmp, cubes[s], c2);
          int fslice11 = coord::fslice1(coord::get_flip(c2), coord::get_slice1(c2));
          if (fslice1_sym[fslice11] == EMPTY)
            fslice1_sym[fslice11] = COUNT_SUB * cls + s;
          else if (fslice11 == fslice1) // collect self-symmetries
            fslice1_selfs[cls] |= 1 << s;
        }
        cls++;
      }
    }
  }

  void init_corners() {
    std::fill(corners_sym, corners_sym + coord::N_CORNERS, EMPTY);

    qubie::cube c1 = qubie::ID_CUBE;
    qubie::cube c2;
    qubie::cube tmp;
    int cls = 0;

    for (int corners = 0; corners < coord::N_CORNERS; corners++) {
      coord::set_corners(c1, corners);

      if (corners_sym[corners] != EMPTY)
        continue;
      corners_sym[corners] = COUNT_SUB * cls;
      corners_raw[cls] = corners;
      corners_selfs[cls] = 1;

      for (int s = 1; s < COUNT_SUB; s++) {
        qubie::corner::mul(cubes[inv[s]], c1, tmp);
        qubie::corner::mul(tmp, cubes[s], c2);
        int corners1 = coord::get_corners(c2);
        if (corners_sym[corners1] == EMPTY)
          corners_sym[corners1] = COUNT_SUB * cls + s;
        else if (corners1 == corners)
          corners_selfs[cls] |= 1 << s;
      }
      cls++;
    }
  }

  // There are no self-symmetries here
  void init_tilt() {
    qubie::cube c = qubie::ID_CUBE;
    qubie::cube c1;

    for (int tilt = 0; tilt < coord::N_TILT; tilt++) {
      coord::set_tilt(c, tilt);
      conj_tilt[tilt][0] = tilt; // sym 0 is identity
      for (int s = 1; s < COUNT_SUB; s++) {
        qubie::mul(c, cubes[inv[s]], c1);
        conj_tilt[tilt][s] = coord::get_tilt(c1);
      }
    }
    
    std::fill(tilt_sym, tilt_sym + coord::N_TILT, EMPTY);
    int cls = 0;

    for (int tilt = 0; tilt < coord::N_TILT; tilt++) {
      coord::set_tilt(c, tilt);

      if (tilt_sym[tilt] != EMPTY)
        continue;
      tilt_sym[tilt] = COUNT_SUB1 * cls;
      tilt_raw[cls] = tilt;

      for (int s = 1; s < COUNT_SUB1; s++) {
        qubie::mul(c, cubes[s], c1);
        int tilt1 = coord::get_tilt(c1);
        if (tilt_sym[tilt1] == EMPTY)
          tilt_sym[tilt1] = COUNT_SUB1 * cls + s;
      }
      cls++;
    }
  }

  void init() {
    init_base();

    init_conjcoord(conj_twist, coord::N_TWIST, coord::get_twist, coord::set_twist, qubie::corner::mul);
    init_conjcoord(conj_udedges2, coord::N_UDEDGES2, coord::get_udedges2, coord::set_udedges2, qubie::edge::mul);
    init_fslice1();
    init_corners();

    init_tilt();
  }

}
