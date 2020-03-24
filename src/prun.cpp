#include "prun.h"

#include <bitset>
#include <iostream>
#include <strings.h>

namespace prun {

  const std::string SAVE = "twophase.tbl";
  const int EMPTY = 0xff;

  uint8_t *phase1;
  uint8_t *phase2;
  uint8_t *precheck;

  void init_phase1() {
    phase1 = new uint8_t[N_FS1TWIST];
    std::fill(phase1, phase1 + N_FS1TWIST, EMPTY);

    // Note that SLICE is not 0 at the end of phase 1
    for (int sstate = 0; sstate < state::N_COORD_SYM; sstate++) {
      int tmp = coord::N_TWIST * sym::coord_c(sym::fslice1_sym[coord::fslice1(0, coord::SLICE1_SOLVED)]);
      phase1[state::N_COORD_SYM * tmp + sstate] = 0;
    }
    int count = 0;
    int dist = 0;

    while (count < N_FS1TWIST) {
      int coord = 0;

      for (int fs1sym = 0; fs1sym < sym::N_FSLICE1; fs1sym++) {
        int fslice1 = sym::fslice1_raw[fs1sym];
        int flip = coord::fslice1_to_flip(fslice1);
        int slice = coord::slice1_to_slice(coord::fslice1_to_slice1(fslice1));

        for (int twist = 0; twist < coord::N_TWIST; twist++) {
          for (int sstate = 0; sstate < state::N_COORD_SYM; sstate++) {
            int state = state::coord_rep[sstate];

            if (phase1[coord] == dist) {
              count++;

              for (move::mask moves = move::p1mask & state::moves[sstate]; moves; moves &= moves - 1) {
                int m = ffsll(moves) - 1;
                int dist1 = dist + 1;

                int state1 = state::move_coord[state][m];
                int sstate1;
                int fs1sym1;
                int twist1;
                int coord1;
                if (m >= move::COUNT_CUBE) {
                  // Don't forget that we are symmetry reducing w.r.t. FSLICE1 and thus need to conjugate here
                  sstate1 = state::cored_coord[state1][sym::coord_s(sym::fslice1_sym[fslice1])];
                  fs1sym1 = fs1sym;
                  twist1 = twist;
                  coord1 = (coord - sstate) + sstate1;
                } else {
                  int slice11 = coord::slice_to_slice1(coord::move_edges4[slice][m]);
                  int fslice11 = coord::fslice1(coord::move_flip[flip][m], slice11);
                  int tmp = sym::fslice1_sym[fslice11];
                  twist1 = sym::conj_twist[coord::move_twist[twist][m]][sym::coord_s(tmp)];
                  fs1sym1 = sym::coord_c(tmp);
                  sstate1 = state::cored_coord[state1][sym::coord_s(tmp)];
                  coord1 = state::N_COORD_SYM * (coord::N_TWIST * fs1sym1 + twist1) + sstate1;
                }
                state1 = state::coord_rep[sstate1]; // we need to apply self-symmetries to the conjugated raw state

                if (phase1[coord1] <= dist)
                  continue;
                phase1[coord1] = dist1;
                coord1 -= state::N_COORD_SYM * twist1 + sstate1; // only TWIST and SSTATE parts change below

                int selfs = sym::fslice1_selfs[fs1sym1] >> 1;
                for (int s = 1; selfs > 0; s++) { // bit 0 is always on
                  if (selfs & 1) {
                    int coord2 = coord1 + state::N_COORD_SYM * sym::conj_twist[twist1][s] + state::cored_coord[state1][s];
                    if (phase1[coord2] > dist1)
                      phase1[coord2] = dist1;
                  }
                  selfs >>= 1;
                }
              }
            }

            coord++;
          }
        }
      }

      std::cout << dist << " " << count << std::endl;
      dist++;
    }
  }

  void init_phase2() {
    phase2 = new uint8_t[N_CORNUD2];
    std::fill(phase2, phase2 + N_CORNUD2, EMPTY);

    for (int sstate = 0; sstate < state::N_COORD_SYM; sstate++)
      phase2[sstate] = 0;
    int count = 0;
    int dist = 0;

    while (count < N_CORNUD2) {
      int coord = 0;

      for (int csym = 0; csym < sym::N_CORNERS; csym++) {
        int corners = sym::corners_raw[csym];

        for (int udedges2 = 0; udedges2 < coord::N_UDEDGES2; udedges2++) {
          for (int sstate = 0; sstate < state::N_COORD_SYM; sstate++) {
            int state = state::coord_rep[sstate];

            if (phase2[coord] == dist) {
              count++;

              for (move::mask moves = move::p2mask & state::moves[sstate]; moves; moves &= moves - 1) {
                int m = ffsll(moves) - 1;
                int dist1 = dist + 1;

                int state1 = state::move_coord[state][m];
                int sstate1;
                int csym1;
                int udedges21;
                int coord1;
                if (m >= move::COUNT_CUBE) {
                  sstate1 = state::cored_coord[state1][sym::coord_s(sym::corners_sym[corners])];
                  csym1 = csym;
                  udedges21 = udedges2;
                  coord1 = (coord - sstate) + sstate1;
                } else {
                  int corners1 = coord::move_corners[corners][m];
                  udedges21 = coord::move_udedges2[udedges2][m];
                  int tmp = sym::corners_sym[corners1];
                  udedges21 = sym::conj_udedges2[udedges21][sym::coord_s(tmp)];
                  csym1 = sym::coord_c(tmp);
                  sstate1 = state::cored_coord[state1][sym::coord_s(tmp)];
                  coord1 = state::N_COORD_SYM * (coord::N_UDEDGES2 * csym1 + udedges21) + sstate1;
                }
                state1 = state::coord_rep[sstate1];

                if (phase2[coord1] <= dist1)
                  continue;
                phase2[coord1] = dist1;
                coord1 -= state::N_COORD_SYM * udedges21 + sstate1;

                int selfs = sym::corners_selfs[csym1] >> 1;
                for (int s = 1; selfs > 0; s++) {
                  if (selfs & 1) {
                    int coord2 = coord1 + state::N_COORD_SYM * sym::conj_udedges2[udedges21][s] + state::cored_coord[state1][s];
                    if (phase2[coord2] > dist1)
                      phase2[coord2] = dist1;
                  }
                  selfs >>= 1;
                }
              }
            }

            coord++;
          }
        }
      }

      std::cout << dist << " " << count << std::endl;
      dist++;
    }
  }

  void init_precheck() {
    precheck = new uint8_t[N_CSLICE2];
    std::fill(precheck, precheck + N_CSLICE2, EMPTY);

    for (int sstate = 0; sstate < state::N_COORD_SYM; sstate++)
      precheck[sstate] = 0;
    int dist = 0;
    int count = 0;

    while (count < N_CSLICE2) {
      int coord = 0;

      for (int corners = 0; corners < coord::N_CORNERS; corners++) {
        for (int slice2 = 0; slice2 < coord::N_SLICE2; slice2++) {
          int slice = coord::slice2_to_slice(slice2);

          for (int sstate = 0; sstate < state::N_COORD_SYM; sstate++) {
            int state = state::coord_rep[sstate];

            if (precheck[coord] == dist) {
              count++;

              for (move::mask moves = move::p2mask & state::moves[sstate]; moves; moves &= moves - 1) {
                int m = ffsll(moves) - 1;
                int dist1 = dist + 1;

                int coord1;
                int state1 = state::move_coord[state][m];
                if (m >= move::COUNT_CUBE)
                  coord1 = (coord - sstate) + state::coord_cls[state1];
                else {
                  int corners1 = coord::move_corners[corners][m];
                  int slice21 = coord::slice_to_slice2(coord::move_edges4[slice][m]);
                  coord1 = state::N_COORD_SYM * (coord::N_SLICE2 * corners1 + slice21) + state::coord_cls[state1];
                }

                if (precheck[coord1] > dist1)
                  precheck[coord1] = dist1;
              }
            }
            coord++;
          }
        }
      }

      std::cout << dist << " " << count << std::endl;
      dist++;
    }
  }

  int get_phase1(int flip, int slice, int twist, int state) {
    int tmp = sym::fslice1_sym[coord::fslice1(flip, coord::slice_to_slice1(slice))];
    int fs1twist = coord::N_TWIST * sym::coord_c(tmp) + sym::conj_twist[twist][sym::coord_s(tmp)];
    return phase1[state::N_COORD_SYM * fs1twist + state::cored_coord[state][sym::coord_s(tmp)]];
  }

  int get_phase2(int corners, int udedges2, int state) {
    int tmp = sym::corners_sym[corners];
    int cornud = coord::N_UDEDGES2 * sym::coord_c(tmp) + sym::conj_udedges2[udedges2][sym::coord_s(tmp)];
    return phase2[state::N_COORD_SYM * cornud + state::cored_coord[state][sym::coord_s(tmp)]];
  }

  int get_precheck(int corners, int slice, int state) {
    return precheck[
      state::N_COORD_SYM * (coord::N_SLICE2 * corners + coord::slice_to_slice2(slice)) + state::coord_cls[state]
    ];
  }

  bool init(bool file) {
    if (!file) {
      init_phase1();
      init_phase2();
      init_precheck();
      return true;
    }

    FILE *f = fopen(SAVE.c_str(), "rb");
    int err = 0;

    if (f == NULL) {
      init_phase1();
      init_phase2();
      init_precheck();

      f = fopen(SAVE.c_str(), "wb");
      if (fwrite(phase1, sizeof(uint8_t), N_FS1TWIST, f) != N_FS1TWIST)
        err = 1;
      if (fwrite(phase2, sizeof(uint8_t), N_CORNUD2, f) != N_CORNUD2)
        err = 1;
      if (fwrite(precheck, sizeof(uint8_t), N_CSLICE2, f) != N_CSLICE2)
        err = 1;
      if (err)
        remove(SAVE.c_str()); // delete file if there was some error writing it
    } else {
      phase1 = new uint8_t[N_FS1TWIST];
      phase2 = new uint8_t[N_CORNUD2];
      precheck = new uint8_t[N_CSLICE2];
      if (fread(phase1, sizeof(uint8_t), N_FS1TWIST, f) != N_FS1TWIST)
        err = 1;
      if (fread(phase2, sizeof(uint8_t), N_CORNUD2, f) != N_CORNUD2)
        err = 1;
      if (fread(precheck, sizeof(uint8_t), N_CSLICE2, f) != N_CSLICE2)
        err = 1;
    }

    fclose(f);
    return err;
  }

}
