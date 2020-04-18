#include "prun.h"

#include <bitset>
#include <iostream>
#include <strings.h>

namespace prun {

  using namespace tilt;

  const std::string SAVE = "twophase.tbl";
  const int EMPTY = 0xff;

  uint64_t *phase1;
  uint8_t *phase2;
  uint8_t *precheck;

  move::mask remap[2][12][1 << 16];
  move::mask remap_tilt[2][2][1 << 4];

  int permute(int mask, int *perm, int len, int step) {
    int permuted = 0;
    for (int i = 0; i < len; i++)
      permuted |= ((mask >> step * perm[i]) & ((1 << step) - 1)) << step * i;
    return permuted;
  }

  void init_base() {
    for (int eff = 0; eff < 12; eff++) {
      for (int enc = 0; enc < 1 << 16; enc++) {
        int tmp = permute(enc >> 1, sym::eff_mperm[sym::eff_nshift(eff)], 15, 1);
        remap[0][eff][enc] = move::mask(enc & 1 ? 0 : ~tmp & 0x7fff) << 15 * sym::eff_shift(eff);
        remap[1][eff][enc] = move::mask(enc & 1 ? ~tmp & 0x7fff : 0x7fff) << 15 * sym::eff_shift(eff);
      }
    }

    for (bool flip : {false, true}) {
      for (int enc = 0; enc < 1 << 4; enc++) {
        for (int delta : {0, 1}) {
          remap_tilt[delta][flip][enc] = 0;
          remap_tilt[delta][flip][enc] |= ((enc >> 2 * flip) & 0x3) <= delta;
          remap_tilt[delta][flip][enc] |= (((enc >> 2 * (1 - flip)) & 0x3) <= delta) << 1;
          remap_tilt[delta][flip][enc] <<= move::COUNT_CUBE;
        }
      }
    }
  }

  void init_phase1() {
    phase1 = new uint64_t[N_FS1TWIST];
    std::fill(phase1, phase1 + N_FS1TWIST, EMPTY);

    // Note that SLICE is not 0 at the end of phase 1
    for (int stilt = 0; stilt < tilt::N_COORD_SYM; stilt++) {
      int tmp = coord::N_TWIST * sym::coord_c(sym::fslice1_sym[coord::fslice1(0, coord::SLICE1_SOLVED)]);
      phase1[tilt::N_COORD_SYM * tmp + stilt] = 0;
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
          for (int stilt = 0; stilt < tilt::N_COORD_SYM; stilt++) {
            int tilt = tilt::coord_rep[stilt];

            if (phase1[coord] == dist) {
              count++;
              int deltas[move::COUNT];

              for (move::mask moves = move::p1mask & tilt::moves[tilt]; moves; moves &= moves - 1) {
                int m = ffsll(moves) - 1;
                int dist1 = dist + 1;

                int tilt1 = tilt::move_coord[tilt][m];
                int stilt1;
                int fs1sym1;
                int twist1;
                int coord1;
                if (m >= move::COUNT_CUBE) {
                  // Don't forget that we are symmetry reducing w.r.t. FSLICE1 and thus need to conjugate here
                  stilt1 = tilt::cored_coord[tilt1][sym::coord_s(sym::fslice1_sym[fslice1])];
                  fs1sym1 = fs1sym;
                  twist1 = twist;
                  coord1 = (coord - stilt) + stilt1;
                } else {
                  int slice11 = coord::slice_to_slice1(coord::move_edges4[slice][m]);
                  int fslice11 = coord::fslice1(coord::move_flip[flip][m], slice11);
                  int tmp = sym::fslice1_sym[fslice11];
                  twist1 = sym::conj_twist[coord::move_twist[twist][m]][sym::coord_s(tmp)];
                  fs1sym1 = sym::coord_c(tmp);
                  stilt1 = tilt::cored_coord[tilt1][sym::coord_s(tmp)];
                  coord1 = tilt::N_COORD_SYM * (coord::N_TWIST * fs1sym1 + twist1) + stilt1;
                }
                tilt1 = tilt::coord_rep[stilt1]; // we need to apply self-symmetries to the conjugated raw tilt

                if (phase1[coord1] != EMPTY) {
                  deltas[m] = (phase1[coord1] & 0xff) - dist;
                  continue;
                }
                phase1[coord1] = dist1;
                deltas[m] = 1;
                coord1 -= tilt::N_COORD_SYM * twist1 + stilt1; // only TWIST and Stilt parts change below

                int selfs = sym::fslice1_selfs[fs1sym1] >> 1;
                for (int s = 1; selfs > 0; s++) { // bit 0 is always on
                  if (selfs & 1) {
                    int coord2 = coord1 + tilt::N_COORD_SYM * sym::conj_twist[twist1][s] + tilt::cored_coord[tilt1][s];
                    if (phase1[coord2] == EMPTY)
                      phase1[coord2] = dist1;
                  }
                  selfs >>= 1;
                }
              }

              /* Encode from left to right to preserve indexing of moves */
              uint64_t prun = 0;

              for (int m = move::COUNT - 1; m >= move::COUNT_CUBE; m--)
                prun = (prun << 2) | (deltas[m] + 1);
              for (int ax = 30; ax >= 0; ax -= 15) {
                bool away = false;
                for (int i = ax; i < ax + 15; i++) {
                  if (deltas[i] != 0) {
                    away = deltas[i] > 0;
                    break; // stop immediately once we found a value != 0
                  }
                }

                int tmp = 0;
                for (int i = (ax + 15) - 1; i >= ax; i--)
                  tmp = (tmp | (deltas[i] + !away)) << 1;
                tmp |= away;

                prun = (prun << 16) | tmp;
              }

              phase1[coord] |= prun << 8;
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

    for (int stilt = 0; stilt < tilt::N_COORD_SYM; stilt++)
      phase2[stilt] = 0;
    int count = 0;
    int dist = 0;

    while (count < N_CORNUD2) {
      int coord = 0;

      for (int csym = 0; csym < sym::N_CORNERS; csym++) {
        int corners = sym::corners_raw[csym];

        for (int udedges2 = 0; udedges2 < coord::N_UDEDGES2; udedges2++) {
          for (int stilt = 0; stilt < tilt::N_COORD_SYM; stilt++) {
            int tilt = tilt::coord_rep[stilt];

            if (phase2[coord] == dist) {
              count++;

              for (move::mask moves = move::p2mask & tilt::moves[tilt]; moves; moves &= moves - 1) {
                int m = ffsll(moves) - 1;
                int dist1 = dist + 1;

                int tilt1 = tilt::move_coord[tilt][m];
                int stilt1;
                int csym1;
                int udedges21;
                int coord1;
                if (m >= move::COUNT_CUBE) {
                  stilt1 = tilt::cored_coord[tilt1][sym::coord_s(sym::corners_sym[corners])];
                  csym1 = csym;
                  udedges21 = udedges2;
                  coord1 = (coord - stilt) + stilt1;
                } else {
                  int corners1 = coord::move_corners[corners][m];
                  udedges21 = coord::move_udedges2[udedges2][m];
                  int tmp = sym::corners_sym[corners1];
                  udedges21 = sym::conj_udedges2[udedges21][sym::coord_s(tmp)];
                  csym1 = sym::coord_c(tmp);
                  stilt1 = tilt::cored_coord[tilt1][sym::coord_s(tmp)];
                  coord1 = tilt::N_COORD_SYM * (coord::N_UDEDGES2 * csym1 + udedges21) + stilt1;
                }
                tilt1 = tilt::coord_rep[stilt1];

                if (phase2[coord1] <= dist1)
                  continue;
                phase2[coord1] = dist1;
                coord1 -= tilt::N_COORD_SYM * udedges21 + stilt1;

                int selfs = sym::corners_selfs[csym1] >> 1;
                for (int s = 1; selfs > 0; s++) {
                  if (selfs & 1) {
                    int coord2 = coord1 + tilt::N_COORD_SYM * sym::conj_udedges2[udedges21][s] + tilt::cored_coord[tilt1][s];
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

    for (int stilt = 0; stilt < tilt::N_COORD_SYM; stilt++)
      precheck[stilt] = 0;
    int dist = 0;
    int count = 0;

    while (count < N_CSLICE2) {
      int coord = 0;

      for (int corners = 0; corners < coord::N_CORNERS; corners++) {
        for (int slice2 = 0; slice2 < coord::N_SLICE2; slice2++) {
          int slice = coord::slice2_to_slice(slice2);

          for (int stilt = 0; stilt < tilt::N_COORD_SYM; stilt++) {
            int tilt = tilt::coord_rep[stilt];

            if (precheck[coord] == dist) {
              count++;

              for (move::mask moves = move::p2mask & tilt::moves[tilt]; moves; moves &= moves - 1) {
                int m = ffsll(moves) - 1;
                int dist1 = dist + 1;

                int coord1;
                int tilt1 = tilt::move_coord[tilt][m];
                if (m >= move::COUNT_CUBE)
                  coord1 = (coord - stilt) + tilt::coord_cls[tilt1];
                else {
                  int corners1 = coord::move_corners[corners][m];
                  int slice21 = coord::slice_to_slice2(coord::move_edges4[slice][m]);
                  coord1 = tilt::N_COORD_SYM * (coord::N_SLICE2 * corners1 + slice21) + tilt::coord_cls[tilt1];
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

  int get_phase1(int flip, int slice, int twist, int tilt, int togo, move::mask& next) {
    int tmp = sym::fslice1_sym[coord::fslice1(flip, coord::slice_to_slice1(slice))];
    int s = sym::coord_s(tmp);
    int fs1twist = coord::N_TWIST * sym::coord_c(tmp) + sym::conj_twist[twist][sym::coord_s(tmp)];
    uint64_t prun = phase1[tilt::N_COORD_SYM * fs1twist + tilt::cored_coord[tilt][sym::coord_s(tmp)]];

    int dist = prun & 0xff;
    int delta = togo - dist;

    // `delta` < 0 case can never happen during a real search
    if (delta > 1)
      next = move::p1mask; // all moves are possible
    else {
      prun >>= 8; // get rid of dist
      next = 0;
      for (int ax = 0; ax < 3; ax++) {
        next |= remap[delta][sym::effect[s][ax]][prun & 0xffff];
        prun >>= 16;
      }
      next |= remap_tilt[delta][tilt::cored_flip[tilt][s]][prun & 0xf];
    }

    return dist;
  }

  /*
  int get_phase1(int flip, int slice, int twist, int tilt) {
    int tmp = sym::fslice1_sym[coord::fslice1(flip, coord::slice_to_slice1(slice))];
    int fs1twist = coord::N_TWIST * sym::coord_c(tmp) + sym::conj_twist[twist][sym::coord_s(tmp)];
    return phase1[tilt::N_COORD_SYM * fs1twist + tilt::cored_coord[tilt][sym::coord_s(tmp)]] & 0xff;
  }
   */

  int get_phase2(int corners, int udedges2, int tilt) {
    int tmp = sym::corners_sym[corners];
    int cornud = coord::N_UDEDGES2 * sym::coord_c(tmp) + sym::conj_udedges2[udedges2][sym::coord_s(tmp)];
    return phase2[tilt::N_COORD_SYM * cornud + tilt::cored_coord[tilt][sym::coord_s(tmp)]];
  }

  int get_precheck(int corners, int slice, int tilt) {
    return precheck[
      tilt::N_COORD_SYM * (coord::N_SLICE2 * corners + coord::slice_to_slice2(slice)) + tilt::coord_cls[tilt]
    ];
  }

  bool init(bool file) {
    init_base();

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
      if (fwrite(phase1, sizeof(uint64_t), N_FS1TWIST, f) != N_FS1TWIST)
        err = 1;
      if (fwrite(phase2, sizeof(uint8_t), N_CORNUD2, f) != N_CORNUD2)
        err = 1;
      if (fwrite(precheck, sizeof(uint8_t), N_CSLICE2, f) != N_CSLICE2)
        err = 1;
      if (err)
        remove(SAVE.c_str()); // delete file if there was some error writing it
    } else {
      phase1 = new uint64_t[N_FS1TWIST];
      phase2 = new uint8_t[N_CORNUD2];
      precheck = new uint8_t[N_CSLICE2];
      if (fread(phase1, sizeof(uint64_t), N_FS1TWIST, f) != N_FS1TWIST)
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
