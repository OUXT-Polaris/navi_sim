// Copyright (c) 2021 OUXT Polaris
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <embree3/rtcore.h>
#include <array>
#include <vector>
#include <limits>
#include <fstream>
#include <chrono>
#include <iostream>

struct Vertex
{
  float x, y, z;
};

struct PolygonIndex
{
  unsigned int v0, v1, v2;
};

struct ColorRGB
{
  uint8_t r, g, b;
};

constexpr uint32_t SCREEN_WIDTH_PX = 100;
constexpr uint32_t SCREEN_HEIGHT_PX = 100;

void SaveAsPpm(
  const std::vector<std::array<ColorRGB, SCREEN_WIDTH_PX>> & image,
  const char * const fileName)
{
  std::ofstream ofs(fileName);

  if (!ofs) {
    std::exit(1);
  }

  ofs << "P3\n" <<
    SCREEN_WIDTH_PX << " " << SCREEN_HEIGHT_PX << "\n255\n";
  for (const auto & row : image) {
    for (const auto & pixel : row) {
      ofs << static_cast<int>(pixel.r) << " " <<
        static_cast<int>(pixel.g) << " " <<
        static_cast<int>(pixel.b) << "\n";
    }
  }

  ofs.close();
}

int main()
{
  std::chrono::system_clock::time_point start, end;
  start = std::chrono::system_clock::now();
  RTCDevice deviceHandle = rtcNewDevice(nullptr);
  RTCScene sceneHandle = rtcNewScene(deviceHandle);
  RTCGeometry geometryHandle = rtcNewGeometry(deviceHandle, RTC_GEOMETRY_TYPE_TRIANGLE);

  constexpr size_t numVertices = 6;
  constexpr size_t numPolygons = 8;

  std::array<Vertex, numVertices> geometryVertices;
  std::array<PolygonIndex, numPolygons> geometryPolygons;

  rtcSetSharedGeometryBuffer(
    geometryHandle,
    RTC_BUFFER_TYPE_VERTEX,
    0,
    RTC_FORMAT_FLOAT3,
    &geometryVertices,
    0,
    sizeof(Vertex),
    geometryVertices.size());

  geometryVertices[0] = {-0.5f, 0.0f, 0.0f};
  geometryVertices[1] = {0.0f, 1.0f, 0.0f};
  geometryVertices[2] = {0.5f, 0.0f, 0.0f};
  geometryVertices[3] = {0.0f, -1.0f, 0.0f};
  geometryVertices[4] = {0.0f, 0.0f, -0.5f};
  geometryVertices[5] = {0.0f, 0.0f, 0.5f};

  rtcSetSharedGeometryBuffer(
    geometryHandle,
    RTC_BUFFER_TYPE_INDEX,
    0,
    RTC_FORMAT_UINT3,
    &geometryPolygons,
    0,
    sizeof(PolygonIndex),
    geometryPolygons.size());

  geometryPolygons[0] = {1, 0, 4};
  geometryPolygons[1] = {1, 4, 2};
  geometryPolygons[2] = {3, 0, 4};
  geometryPolygons[3] = {3, 4, 2};
  geometryPolygons[4] = {1, 5, 0};
  geometryPolygons[5] = {1, 2, 5};
  geometryPolygons[6] = {3, 5, 0};
  geometryPolygons[7] = {3, 2, 5};

  rtcCommitGeometry(geometryHandle);
  rtcAttachGeometry(sceneHandle, geometryHandle);
  rtcReleaseGeometry(geometryHandle);
  rtcCommitScene(sceneHandle);

  constexpr float FLOAT_INFINITY = std::numeric_limits<float>::max();
  std::vector<std::array<ColorRGB, SCREEN_WIDTH_PX>> screen{SCREEN_HEIGHT_PX};

  RTCIntersectContext context;
  rtcInitIntersectContext(&context);

  RTCRayHit rayhit;
  rayhit.ray.org_z = -10.0f;
  rayhit.ray.dir_x = 0.0f;
  rayhit.ray.dir_y = 0.0f;
  rayhit.ray.dir_z = 1.0f;
  rayhit.ray.tnear = 0.0f;

  constexpr uint32_t HALF_HEIGHT = static_cast<int>(SCREEN_HEIGHT_PX) / 2;
  constexpr uint32_t HALF_WIDTH = static_cast<int>(SCREEN_WIDTH_PX) / 2;

  for (int32_t y = 0; y < SCREEN_HEIGHT_PX; ++y) {
    for (int32_t x = 0; x < SCREEN_WIDTH_PX; ++x) {
      rayhit.ray.org_x = (x - static_cast<int>(HALF_WIDTH)) / static_cast<float>(HALF_WIDTH);
      rayhit.ray.org_y = (y - static_cast<int>(HALF_HEIGHT)) / static_cast<float>(HALF_HEIGHT);
      rayhit.ray.tfar = FLOAT_INFINITY;
      rayhit.ray.flags = false;
      rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
      rtcIntersect1(sceneHandle, &context, &rayhit);
      if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
        screen[y][x].r = 0xff;
        screen[y][x].g = 0x00;
        screen[y][x].b = 0xff;
      } else {
        screen[y][x].r = 0x00;
        screen[y][x].g = 0x00;
        screen[y][x].b = 0x00;
      }
    }
  }

  SaveAsPpm(screen, "result.ppm");

  rtcReleaseScene(sceneHandle);
  rtcReleaseDevice(deviceHandle);

  end = std::chrono::system_clock::now();
  std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() <<
    " milli seconds" <<
    std::endl;
}
