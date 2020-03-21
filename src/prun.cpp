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
    int n_moves = std::bitset<64>(move::p1mask).count(); // make sure not to consider B-moves in F5-mode

    phase1 = new uint8_t[N_FS1TWIST];
    std::fill(phase1, phase1 + N_FS1TWIST, EMPTY);

    for (int ssumm = 0; ssumm < state::N_SUMM_SYM; ssumm++) {
      int tmp = coord::N_TWIST * sym::coord_c(sym::fslice1_sym[coord::fslice1(0, coord::SLICE1_SOLVED)]);;
      phase1[state::N_SUMM_SYM * tmp + ssumm] = 0;
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
          for (int ssumm = 0; ssumm < state::N_SUMM_SYM; ssumm++) {
            int summ = state::summ_rep[ssumm];

            if ((phase1[coord] & 0xff) == dist) {
              count++;

              for (move::mask moves = move::p1mask & state::moves[ssumm]; moves; moves &= moves - 1) {
                int m = ffsll(moves) - 1;

                if (m >= move::COUNT_CUBE) {
                  int summ1 = state::move_summ[summ][m];
                  int coord1 = (coord - ssumm) + state::summ_cls[summ1];
                  if (phase1[coord1] != EMPTY)
                    continue;
                  phase1[coord1] = dist + 1;
                } else {
                  int slice11 = coord::slice_to_slice1(coord::move_edges4[slice][m]);
                  int fslice11 = coord::fslice1(coord::move_flip[flip][m], slice11);
                  int tmp = sym::fslice1_sym[fslice11];
                  int twist1 = sym::conj_twist[coord::move_twist[twist][m]][sym::coord_s(tmp)];
                  int fs1sym1 = sym::coord_c(tmp);
                  int ssumm1 = state::cored_summ[summ][sym::coord_s(tmp)];
                  int coord1 = state::N_SUMM_SYM * (coord::N_TWIST * fs1sym1 + twist1) + ssumm1;

                  if (phase1[coord1] == EMPTY)
                    phase1[coord1] = dist + 1;
                  coord1 -= state::N_SUMM_SYM * twist1; // only TWIST part changes below

                  int selfs = sym::fslice1_selfs[fs1sym1] >> 1;
                  for (int s = 1; selfs > 0; s++) { // bit 0 is always on
                    if (selfs & 1) {
                      int coord2 = coord1 + state::N_SUMM_SYM * sym::conj_twist[twist1][s];
                      if (phase1[coord2] == EMPTY)
                        phase1[coord2] = dist + 1;
                    }
                    selfs >>= 1;
                  }
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

    for (int ssumm = 0; ssumm < state::N_SUMM_SYM; ssumm++)
      phase2[ssumm] = 0;
    int count = 0;
    int dist = 0;

    while (count < N_CORNUD2) {
      int coord = 0;

      for (int csym = 0; csym < sym::N_CORNERS; csym++) {
        int corners = sym::corners_raw[csym];

        for (int udedges2 = 0; udedges2 < coord::N_UDEDGES2; udedges2++) {
          for (int ssumm = 0; ssumm < state::N_SUMM_SYM; ssumm++) {
            int summ = state::summ_rep[ssumm];

            if (phase2[coord] == dist) {
              count++;

              for (move::mask moves = move::p2mask & state::moves[ssumm]; moves; moves &= moves - 1) {
                int m = ffsll(moves) - 1;
                int dist1 = dist + 1;

                if (m >= move::COUNT_CUBE) {
                  int summ1 = state::move_summ[summ][m];
                  int coord1 = (coord - ssumm) + state::summ_cls[summ1];
                  if (phase2[coord1] <= dist1)
                    continue;
                  phase2[coord1] = dist1;
                } else {
                  int corners1 = coord::move_corners[corners][m];
                  int udedges21 = coord::move_udedges2[udedges2][m];
                  int tmp = sym::corners_sym[corners1];
                  udedges21 = sym::conj_udedges2[udedges21][sym::coord_s(tmp)];
                  int csym1 = sym::coord_c(tmp);
                  int ssumm1 = state::cored_summ[summ][sym::coord_s(tmp)];
                  int coord1 = state::N_SUMM_SYM * (coord::N_UDEDGES2 * csym1 + udedges21) + ssumm1;

                  if (phase2[coord1] <= dist1)
                    continue;
                  phase2[coord1] = dist1;
                  coord1 -= state::N_SUMM_SYM * udedges21;

                  int selfs = sym::corners_selfs[csym1] >> 1;
                  for (int s = 1; selfs > 0; s++) {
                    if (selfs & 1) {
                      int coord2 = coord1 + state::N_SUMM_SYM * sym::conj_udedges2[udedges21][s];
                      if (phase2[coord2] > dist1)
                        phase2[coord2] = dist1;
                    }
                    selfs >>= 1;
                  }
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

    for (int ssumm = 0; ssumm < state::N_SUMM_SYM; ssumm++)
      precheck[ssumm] = 0;
    int dist = 0;
    int count = 0;

    while (count < N_CSLICE2) {
      int coord = 0;

      for (int corners = 0; corners < coord::N_CORNERS; corners++) {
        for (int slice2 = 0; slice2 < coord::N_SLICE2; slice2++) {
          for (int ssumm = 0; ssumm < state::N_SUMM_SYM; ssumm++) {
            int summ = state::summ_rep[ssumm];

            if (precheck[coord] == dist) {
              count++;
              int slice = coord::slice2_to_slice(slice2);

              for (move::mask moves = move::p2mask & state::moves[ssumm]; moves; moves &= moves - 1) {
                int m = ffsll(moves) - 1;
                int dist1 = dist + 1;

                int coord1;
                if (m >= move::COUNT_CUBE) {
                  int summ1 = state::move_summ[summ][m];
                  coord1 = (coord - ssumm) + state::summ_cls[summ1];
                } else {
                  int corners1 = coord::move_corners[corners][m];
                  int slice21 = coord::slice_to_slice2(coord::move_edges4[slice][m]);
                  coord1 = state::N_SUMM_SYM * (coord::N_SLICE2 * corners1 + slice21) + ssumm;
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

  bool init(bool file) {
    init_phase1();
    init_phase2();
    init_precheck();
  }

}
