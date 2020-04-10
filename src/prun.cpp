#include "prun.h"

#include <bitset>
#include <iostream>
#include <strings.h>

namespace prun {

  using namespace state;

  const std::string SAVE = "twophase.tbl";
  const int EMPTY = 0xff;

  uint8_t *phase1;
  uint8_t *phase2;
  uint8_t *precheck;

  void init_phase1() {
    // We use this small to table to dramatically speed up the backsearching for the actual phase 1 table generation
    // (especially the early iterations where only very few entries are filled

    /*
    int n_twistcheck = coord::N_TWIST * state::N_COORD_SYM;
    uint8_t twistcheck[n_twistcheck];
    std::fill(twistcheck, twistcheck + n_twistcheck, EMPTY);

    for (int sstate = 0; sstate < state::N_COORD_SYM; sstate++)
      twistcheck[sstate] = 0;
    int dist = 1;
    int count = state::N_COORD_SYM;

    while (count < n_twistcheck) {
      int coord = 0;

      for (int twist = 0; twist < coord::N_TWIST; twist++) {
        for (int sstate = 0; sstate < state::N_COORD_SYM; sstate++) {
          int state = state::coord_rep[sstate];

          if (twistcheck[coord] == EMPTY) {
            for (move::mask moves = move::p1mask & state::moves[state]; moves; moves &= moves - 1) {
              int m = ffsll(moves) - 1;

              int coord1;
              int sstate1 = state::coord_cls[state::move_coord[state][m]];
              if (m >= move::COUNT_CUBE)
                coord1 = (coord - sstate) + sstate1;
              else {
                int twist1 = coord::move_twist[twist][m];
                coord1 = state::N_COORD_SYM * twist1 + sstate1;
              }

              if (twistcheck[coord1] == dist - 1) {
                twistcheck[coord] = dist;
                count++;
                break;
              }
            }
          }
          coord++;
        }
      }

      std::cout << dist << " " << count << std::endl;
      dist++;
    }
    */

    int n_fs1symcheck = sym::N_FSLICE1 * state::N_COORD_SYM;
    uint8_t fs1symcheck[n_fs1symcheck];
    std::fill(fs1symcheck, fs1symcheck + n_fs1symcheck, EMPTY);

    for (int sstate = 0; sstate < state::N_COORD_SYM; sstate++) {
      int tmp = sym::coord_c(sym::fslice1_sym[coord::fslice1(0, coord::SLICE1_SOLVED)]);
      fs1symcheck[state::N_COORD_SYM * tmp + sstate] = 0;
    }
    int dist = 1;
    int count = state::N_COORD_SYM;

    while (count < n_fs1symcheck) {
      int coord = 0;

      for (int fs1sym = 0; fs1sym < sym::N_FSLICE1; fs1sym++) {
        int fslice1 = sym::fslice1_raw[fs1sym];
        int flip = coord::fslice1_to_flip(fslice1);
        int slice = coord::slice1_to_slice(coord::fslice1_to_slice1(fslice1));

        for (int sstate = 0; sstate < state::N_COORD_SYM; sstate++) {
          int state = state::coord_rep[sstate];

          if (fs1symcheck[coord] == EMPTY) {
            for (move::mask moves = move::p1mask & state::moves[state]; moves; moves &= moves - 1) {
              int m = ffsll(moves) - 1;

              int coord1;
              int state1 = state::move_coord[state][m];
              if (m >= move::COUNT_CUBE) {
                // Don't forget that we are symmetry reducing w.r.t. FSLICE1 and thus need to conjugate here
                int sstate1 = state::cored_coord[state1][sym::coord_s(sym::fslice1_sym[fslice1])];
                coord1 = (coord - sstate) + sstate1;
              } else {
                int slice11 = coord::slice_to_slice1(coord::move_edges4[slice][m]);
                int fslice11 = coord::fslice1(coord::move_flip[flip][m], slice11);
                int tmp = sym::fslice1_sym[fslice11];
                int sstate1 = state::cored_coord[state1][sym::coord_s(tmp)];
                coord1 = state::N_COORD_SYM * sym::coord_c(tmp) + sstate1;
              }

              if (fs1symcheck[coord1] == dist - 1) {
                fs1symcheck[coord] = dist;
                count++;
                break;
              }
            }
          }
          coord++;
        }
      }

      std::cout << dist << " " << count << std::endl;
      dist++;
    }

    phase1 = new uint8_t[N_FS1TWIST];
    std::fill(phase1, phase1 + N_FS1TWIST, EMPTY);

    // Note that SLICE is not 0 at the end of phase 1
    for (int sstate = 0; sstate < state::N_COORD_SYM; sstate++) {
      int tmp = coord::N_TWIST * sym::coord_c(sym::fslice1_sym[coord::fslice1(0, coord::SLICE1_SOLVED)]);
      phase1[state::N_COORD_SYM * tmp + sstate] = 0;
    }
    dist = 1;
    count = state::N_COORD_SYM;

    while (count < N_FS1TWIST) {
      int coord = 0;

      for (int fs1sym = 0; fs1sym < sym::N_FSLICE1; fs1sym++) {
        int fslice1 = sym::fslice1_raw[fs1sym];
        int flip = coord::fslice1_to_flip(fslice1);
        int slice = coord::slice1_to_slice(coord::fslice1_to_slice1(fslice1));

        // int check = 0;
        int check = state::N_COORD_SYM * fs1sym;
        for (int twist = 0; twist < coord::N_TWIST; twist++) {
          for (int sstate = 0; sstate < state::N_COORD_SYM; sstate++) {
            int state = state::coord_rep[sstate];

            if (phase1[coord] == EMPTY && fs1symcheck[check + sstate] <= dist) {
              for (move::mask moves = move::p1mask & state::moves[state]; moves; moves &= moves - 1) {
                int m = ffsll(moves) - 1;

                int coord1;
                int state1 = state::move_coord[state][m];
                if (m >= move::COUNT_CUBE) {
                  int sstate1 = state::cored_coord[state1][sym::coord_s(sym::fslice1_sym[fslice1])];
                  coord1 = (coord - sstate) + sstate1;
                } else {
                  int slice11 = coord::slice_to_slice1(coord::move_edges4[slice][m]);
                  int fslice11 = coord::fslice1(coord::move_flip[flip][m], slice11);
                  int tmp = sym::fslice1_sym[fslice11];
                  int twist1 = sym::conj_twist[coord::move_twist[twist][m]][sym::coord_s(tmp)];
                  int sstate1 = state::cored_coord[state1][sym::coord_s(tmp)];
                  coord1 = state::N_COORD_SYM * (coord::N_TWIST * sym::coord_c(tmp) + twist1) + sstate1;
                }

                if (phase1[coord1] == dist - 1) {
                  phase1[coord] = dist;
                  count++;
                  break;
                }
              }
            }

            coord++;
            // check++;
          }
        }
      }

      std::cout << dist << " " << count << std::endl;
      dist++;
    }
  }

  void init_phase2() {
    int n_edgecheck = coord::N_UDEDGES2 * state::N_COORD_SYM;
    uint8_t edgecheck[n_edgecheck];
    std::fill(edgecheck, edgecheck + n_edgecheck, EMPTY);

    for (int sstate = 0; sstate < state::N_COORD_SYM; sstate++)
      edgecheck[sstate] = 0;
    int dist = 1;
    int count = state::N_COORD_SYM;

    while (count < n_edgecheck) {
      int coord = 0;

      for (int udedges2 = 0; udedges2 < coord::N_UDEDGES2; udedges2++) {
        for (int sstate = 0; sstate < state::N_COORD_SYM; sstate++) {
          int state = state::coord_rep[sstate];

          if (edgecheck[coord] == EMPTY) {
            for (move::mask moves = move::p2mask & state::moves[state]; moves; moves &= moves - 1) {
              int m = ffsll(moves) - 1;

              int coord1;
              int sstate1 = state::coord_cls[state::move_coord[state][m]];
              if (m >= move::COUNT_CUBE)
                coord1 = (coord - sstate) + sstate1;
              else {
                int udedges21 = coord::move_udedges2[udedges2][m];
                coord1 = state::N_COORD_SYM * udedges21 + sstate1;
              }

              if (edgecheck[coord1] == dist - 1) {
                edgecheck[coord] = dist;
                count++;
                break;
              }
            }
          }
          coord++;
        }
      }

      std::cout << dist << " " << count << std::endl;
      dist++;
    }

    phase2 = new uint8_t[N_CORNUD2];
    std::fill(phase2, phase2 + N_CORNUD2, EMPTY);

    for (int sstate = 0; sstate < state::N_COORD_SYM; sstate++)
      phase2[sstate] = 0;
    dist = 1;
    count = state::N_COORD_SYM;

    while (count < N_CORNUD2) {
      int coord = 0;

      for (int csym = 0; csym < sym::N_CORNERS; csym++) {
        int corners = sym::corners_raw[csym];

        int check = 0;
        for (int udedges2 = 0; udedges2 < coord::N_UDEDGES2; udedges2++) {
          for (int sstate = 0; sstate < state::N_COORD_SYM; sstate++) {
            int state = state::coord_rep[sstate];

            if (phase2[coord] == EMPTY && edgecheck[check] <= dist) {
              for (move::mask moves = move::p2mask & state::moves[state]; moves; moves &= moves - 1) {
                int m = ffsll(moves) - 1;

                int coord1;
                int state1 = state::move_coord[state][m];
                if (m >= move::COUNT_CUBE) {
                  int sstate1 = state::cored_coord[state1][sym::coord_s(sym::corners_sym[corners])];
                  coord1 = (coord - sstate) + sstate1;
                } else {
                  int corners1 = coord::move_corners[corners][m];
                  int udedges21 = coord::move_udedges2[udedges2][m];
                  int tmp = sym::corners_sym[corners1];
                  udedges21 = sym::conj_udedges2[udedges21][sym::coord_s(tmp)];
                  int sstate1 = state::cored_coord[state1][sym::coord_s(tmp)];
                  coord1 = state::N_COORD_SYM * (coord::N_UDEDGES2 * sym::coord_c(tmp) + udedges21) + sstate1;
                }

                if (phase2[coord1] == dist - 1) {
                  phase2[coord] = dist;
                  count++;
                  break;
                }
              }
            }

            coord++;
            check++;
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
    int dist = 1;
    int count = state::N_COORD_SYM;

    while (count < N_CSLICE2) {
      int coord = 0;

      for (int corners = 0; corners < coord::N_CORNERS; corners++) {
        for (int slice2 = 0; slice2 < coord::N_SLICE2; slice2++) {
          int slice = coord::slice2_to_slice(slice2);

          for (int sstate = 0; sstate < state::N_COORD_SYM; sstate++) {
            int state = state::coord_rep[sstate];

            if (precheck[coord] == EMPTY) {
              for (move::mask moves = move::p2mask & state::moves[state]; moves; moves &= moves - 1) {
                int m = ffsll(moves) - 1;

                if (state::move_coord[state][m] == -1)
                  break;

                int coord1;
                int sstate1 = state::coord_cls[state::move_coord[state][m]];
                if (m >= move::COUNT_CUBE)
                  coord1 = (coord - sstate) + sstate1;
                else {
                  int corners1 = coord::move_corners[corners][m];
                  int slice21 = coord::slice_to_slice2(coord::move_edges4[slice][m]);
                  coord1 = state::N_COORD_SYM * (coord::N_SLICE2 * corners1 + slice21) + sstate1;
                }

                if (precheck[coord1] == dist - 1) {
                  precheck[coord] = dist;
                  count++;
                  break;
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
