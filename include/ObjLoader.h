
#pragma once

#include <locale>
#include <string>
#include <vector>

#include "unsuck.hpp"

#include "glm/common.hpp"
#include "glm/gtx/transform.hpp"
#include "glm/matrix.hpp"

// note: stored as non-indexed triangles.
// [      triangle     ][      triangle     ]
// [x,y,z][x,y,z][x,y,z][x,y,z][x,y,z][x,y,z]
// [u,v]  [u,v]  [u,v]  [u,v]  [u,v]  [u,v]
struct ObjData {
  int numTriangles = 0;
  std::vector<glm::vec3> xyz;
  std::vector<glm::vec2> uv;
};

struct ObjLoader {
  static std::shared_ptr<ObjData> load(const std::string& file) {
    std::string str = readTextFile(file);

    std::vector<glm::vec3> v_s;
    std::vector<glm::vec2> vt_s;
    std::vector<std::vector<std::string>> f_s;

    setlocale(LC_ALL, "en_US.utf8");

    // parse file
    int pos = 0;
    while (true) {
      int nextPos = str.find('\n', pos);

      if (nextPos == string::npos)
        break;

      std::string line = str.substr(pos, nextPos - pos);

      auto tokens = split(line, ' ');

      if (tokens[0] == "v") {
        float x = stof(tokens[1]);
        float y = stof(tokens[2]);
        float z = stof(tokens[3]);
        glm::vec3 v = {x, y, z};
        v_s.push_back(v);
      } else if (tokens[0] == "vt") {
        float u = stof(tokens[1]);
        float v = stof(tokens[2]);
        glm::vec2 vt = {u, v};
        vt_s.push_back(vt);
      } else if (tokens[0] == "f") {
        f_s.push_back(tokens);
      }

      pos = nextPos + 1;
    }

    // assemble non-indexed triangles
    auto obj = std::make_shared<ObjData>();
    for (int i = 0; i < f_s.size(); i++) {
      auto tokens_f = f_s[i];
      auto tokens_v0 = split(tokens_f[1], '/');
      auto tokens_v1 = split(tokens_f[2], '/');
      auto tokens_v2 = split(tokens_f[3], '/');

      glm::vec3 v0 = v_s[stoi(tokens_v0[0]) - 1];
      glm::vec3 v1 = v_s[stoi(tokens_v1[0]) - 1];
      glm::vec3 v2 = v_s[stoi(tokens_v2[0]) - 1];

      glm::vec2 uv0 = vt_s[stoi(tokens_v0[1]) - 1];
      glm::vec2 uv1 = vt_s[stoi(tokens_v1[1]) - 1];
      glm::vec2 uv2 = vt_s[stoi(tokens_v2[1]) - 1];

      obj->xyz.push_back(v0);
      obj->xyz.push_back(v1);
      obj->xyz.push_back(v2);
      obj->uv.push_back(uv0);
      obj->uv.push_back(uv1);
      obj->uv.push_back(uv2);

      obj->numTriangles++;
    }

    return obj;
  }
};
