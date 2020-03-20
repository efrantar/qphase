#include "face.h"

namespace face {

  // True mod
  int mod(int a, int m) {
    return a > 0 ? a % m : (a % m + m) % m;
  }

  // Convert cubelet + ori into a single number
  int encode(const std::string& cubelet, int ori) {
    int code = 0;
    for (int i = 0; i < cubelet.size(); i++)
      code = COUNT * code + color::FROM_NAME.at(cubelet[mod(i - ori, cubelet.size())]);
    return code;
  }

  // Map code to qubie + ori
  std::unordered_map<int, std::pair<int, int>> corners;
  std::unordered_map<int, std::pair<int, int>> edges;

  void init() {
    for (int corner = 0; corner < qubie::corner::COUNT; corner++) {
      for (int ori = 0; ori < 3; ori++)
        corners[encode(qubie::corner::NAMES[corner], ori)] = std::make_pair(corner, ori);
    }
    for (int edge = 0; edge < qubie::edge::COUNT; edge++) {
      for (int ori = 0; ori < 2; ori++)
        edges[encode(qubie::edge::NAMES[edge], ori)] = std::make_pair(edge, ori);
    }
  }

  int to_cubie(const std::string& s, qubie::cube& c) {
    for (int i = 0; i < N_FACELETS; i++) {
      if (color::FROM_NAME.find(s[i]) == color::FROM_NAME.end())
        return 1; // invalid color
      if ((i - 4) % 9 == 0 && color::FROM_NAME.at(s[i]) != i / 9)
        return 2; // invalid center facelet
    }

    for (int corner = 0; corner < qubie::corner::COUNT; corner++) {
      char cornlet[3];
      for (int i = 0; i < 3; i++)
        cornlet[i] = s[CORNLETS[corner][i]];
      auto tmp = corners.find(encode(std::string(cornlet, 3), 0));
      if (tmp == corners.end())
        return 3; // invalid corner qubie
      c.cperm[corner] = tmp->second.first;
      c.cori[corner] = tmp->second.second;
    }

    for (int edge = 0; edge < qubie::edge::COUNT; edge++) {
      char edgelet[2];
      for (int i = 0; i < 2; i++)
        edgelet[i] = s[EDGELETS[edge][i]];
      auto tmp = edges.find(encode(std::string(edgelet, 2), 0));
      if (tmp == edges.end())
        return 4; // invalid edge qubie
      c.eperm[edge] = tmp->second.first;
      c.eori[edge] = tmp->second.second;
    }

    return 0;
  }

  // Assumes given cube to be valid
  std::string from_cubie(const qubie::cube& c) {
    char s[N_FACELETS];

    for (int color = 0; color < COUNT; color++)
      s[9 * color + 4] = color::NAMES[color];
    for (int corner = 0; corner < qubie::corner::COUNT; corner++) {
      for (int i = 0; i < 3; i++)
        // Corner twist defined clockwise
        s[CORNLETS[corner][i]] = qubie::corner::NAMES[c.cperm[corner]][mod(i - c.cori[corner], 3)];
    }
    for (int edge = 0; edge < qubie::edge::COUNT; edge++) {
      for (int i = 0; i < 2; i++)
        s[EDGELETS[edge][i]] = qubie::edge::NAMES[c.eperm[edge]][mod(i - c.eori[edge], 2)];
    }

    return std::string(s, N_FACELETS);
  }

}
