#pragma once

#include "HostDeviceInterface.h"

struct HeightmapInstance {
  float offsetX = 0.0f;
  float offsetY = 0.0f;
  uint32_t heightmapIndex = 0;
};

struct CTexture {
  int width = 0;
  int height = 0;
  uint8_t* data = nullptr;
};

constexpr int MATERIAL_MODE_COLOR = 0;
constexpr int MATERIAL_MODE_VERTEXCOLOR = 1;
constexpr int MATERIAL_MODE_UVS = 2;
constexpr int MATERIAL_MODE_POSITION = 3;

struct TriangleMaterial {
  float4 color = float4{1.0f, 0.0f, 0.0f, 1.0f};
  int mode = MATERIAL_MODE_COLOR;
  CTexture texture;
  float2 uv_offset = {0.0f, 0.0f};
  float2 uv_scale = {1.0f, 1.0f};
};

