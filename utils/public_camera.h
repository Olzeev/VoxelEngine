#pragma once
#include "LiteMath.h"
#include <cstring>
#include <cstdio>
#include <errno.h>

using LiteMath::float3, LiteMath::float2;
struct Camera
{
  float3 pos;         // camera position
  float fov_rad = 3.14159265f / 2;    // field of view in radians
  float speed = 5.0f;
  float3 dir = float3(1.0f, 0.0f, 0.0f);
  float sensitivity = 0.5f;
  float2 angle = float2(0.0f, 0.0f);
};