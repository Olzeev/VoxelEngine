#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION

#include "utils/mesh.h"
#include "utils/LiteMath.h"
#include "utils/public_camera.h"
#include "utils/public_image.h"

#include <cstdio>
#include <cstring>
#include <SDL_keycode.h>
#include <cstdint>
#include <iostream>
#include <fstream>
#include <SDL.h>
#include <chrono>

using LiteMath::float2;
using LiteMath::float3;
using LiteMath::float4;
using LiteMath::int2;
using LiteMath::int3;
using LiteMath::int4;
using LiteMath::uint2;
using LiteMath::uint3;
using LiteMath::uint4;

#define PI 3.14159265358979323846264338327950288

static constexpr int SCREEN_WIDTH  = 640;
static constexpr int SCREEN_HEIGHT = 480;

float rad_to_deg(float rad) { return rad * 180.0f / PI; }

uint32_t float3_to_RGBA8(float3 c)
{
  uint8_t r = (uint8_t)(std::clamp(c.x,0.0f,1.0f)*255.0f);
  uint8_t g = (uint8_t)(std::clamp(c.y,0.0f,1.0f)*255.0f);
  uint8_t b = (uint8_t)(std::clamp(c.z,0.0f,1.0f)*255.0f);
  return 0xFF000000 | (r<<16) | (g<<8) | b;
}

float scene_distance(float3 p)
{
  const float2 t = float2(1.0f, 0.4f);
  float2 q = float2(LiteMath::length(float2(p.x,p.y))-t.x,p.z);
  return LiteMath::length(q)-t.y;
}

void torus_sphere_tracing_demo(const Camera &camera, uint32_t *out_image, int W, int H)
{
  LiteMath::float4x4 view = LiteMath::lookAt(camera.pos, camera.target, camera.up);
  LiteMath::float4x4 proj = LiteMath::perspectiveMatrix(rad_to_deg(camera.fov_rad), (float)W/(float)H, camera.z_near, camera.z_far);
  LiteMath::float4x4 viewProjInv = LiteMath::inverse4x4(proj*view);

  const float3 light_dir = normalize(float3(-1,-1,-1));

  for (int y=0;y<H;y++)
  {
    for (int x=0;x<W;x++)
    {
      float4 point_NDC = float4(2.0f*((x+0.5f)/W)-1.0f, 2.0f*((y+0.5f)/H)-1.0f, 0, 1);
      float4 point_W = viewProjInv*point_NDC;
      float3 point = LiteMath::to_float3(point_W)/point_W.w;
      float3 ray_pos = camera.pos;
      float3 ray_dir = LiteMath::normalize(point - ray_pos);

      float t = 0;
      int iter = 0;
      float d = scene_distance(ray_pos + t*ray_dir);
      while (d > 1e-6f && iter < 1000 && d < 5)
      {
        t += d;
        d = scene_distance(ray_pos + t*ray_dir);
        iter++;
      }

      float3 color = float3(0,0,0);
      if (d <= 1e-6f)
      {
        float3 p = ray_pos + t*ray_dir;
        float dx = 0.001f;
        float idx = 1.0f/dx;
        float3 n = float3((scene_distance(p + float3(dx,0,0)) - d)*idx,
                          (scene_distance(p + float3(0,dx,0)) - d)*idx,
                          (scene_distance(p + float3(0,0,dx)) - d)*idx);
        color = float3(0.7,0.3,0.1) * (0.25f + std::max(0.0f, dot(normalize(n), light_dir)));
      }

      out_image[y*W + x] = float3_to_RGBA8(color);
    }
  }
}

void draw_frame_example(const Camera &camera, std::vector<uint32_t> &pixels)
{
  torus_sphere_tracing_demo(camera, pixels.data(), SCREEN_WIDTH, SCREEN_HEIGHT);
}

// You must include the command line parameters for your main function to be recognized by SDL
int main(int argc, char **args)
{
  // Pixel buffer (RGBA format)
  std::vector<uint32_t> pixels(SCREEN_WIDTH * SCREEN_HEIGHT, 0xFFFFFFFF); // Initialize with white pixels

  // Initialize SDL. SDL_Init will return -1 if it fails.
  if (SDL_Init(SDL_INIT_EVERYTHING) < 0)
  {
    std::cerr << "Error initializing SDL: " << SDL_GetError() << std::endl;
    return 1;
  }

  // Create our window
  SDL_Window *window = SDL_CreateWindow("SDF Viewer", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                                        SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);

  // Make sure creating the window succeeded
  if (!window)
  {
    std::cerr << "Error creating window: " << SDL_GetError() << std::endl;
    return 1;
  }

  // Create a renderer
  SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
  if (!renderer)
  {
    std::cerr << "Renderer could not be created! SDL_Error: " << SDL_GetError() << std::endl;
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 1;
  }

  // Create a texture
  SDL_Texture *texture = SDL_CreateTexture(
      renderer,
      SDL_PIXELFORMAT_ARGB8888,    // 32-bit RGBA format
      SDL_TEXTUREACCESS_STREAMING, // Allows us to update the texture
      SCREEN_WIDTH,
      SCREEN_HEIGHT);

  if (!texture)
  {
    std::cerr << "Texture could not be created! SDL_Error: " << SDL_GetError() << std::endl;
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 1;
  }

  SDL_Event ev;
  bool running = true;

  Camera camera;
  camera.pos = float3(0, 0, 7);
  camera.target = float3(0, 0, 0);
  camera.up = float3(0, 1, 0);

  auto time = std::chrono::high_resolution_clock::now();
  auto prev_time = time;
  float time_from_start = 0;
  uint32_t frameNum = 0;

  

  // Main loop
  while (running)
  {
    // Process keyboard input
    while (SDL_PollEvent(&ev) != 0)
    {
      // check event type
      switch (ev.type)
      {
      case SDL_QUIT:
        // shut down
        running = false;
        break;
      case SDL_KEYDOWN:
        // test keycode
        switch (ev.key.keysym.sym)
        {
        //ESC to exit 
        case SDLK_ESCAPE:
          running = false;
          break;
          // etc
        }
        break;
      }
    }

    //update camera or scene
    prev_time = time;
    time = std::chrono::high_resolution_clock::now();

    //get delta time in seconds
    float dt = std::chrono::duration<float, std::milli>(time - prev_time).count() / 1000.0f;
    time_from_start += dt;
    frameNum++;

    if (frameNum % 10 == 0)
      printf("Render time: %f ms\n", 1000.0f*dt);

    // example - rotate camera around the origin in XZ plane at a constant speed, 1 revolution per 10 seconds
    float rot_speed = PI * 2.0f / 10.0f;
    camera.pos = 7.0f*float3(sin(time_from_start) * rot_speed, 0, cos(time_from_start) * rot_speed);

    // Render the scene
    draw_frame_example(camera, pixels);

    // Update the texture with the pixel buffer
    SDL_UpdateTexture(texture, nullptr, pixels.data(), SCREEN_WIDTH * sizeof(uint32_t));

    // Clear the renderer
    SDL_RenderClear(renderer);

    // Copy the texture to the renderer
    SDL_RenderCopy(renderer, texture, nullptr, nullptr);

    // Update the screen
    SDL_RenderPresent(renderer);
  }

  // Destroy the window. This will also destroy the surface
  SDL_DestroyWindow(window);

  // Quit SDL
  SDL_Quit();

  // End the program
  return 0;
}