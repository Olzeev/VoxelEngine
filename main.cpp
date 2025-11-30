#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION

#include "utils/mesh.h"
#include "utils/LiteMath.h"
#include "utils/public_camera.h"
#include "utils/public_image.h"
#include "utils/blocks.h"

#include <cstdio>
#include <cstring>
#include <SDL_keycode.h>
#include <cstdint>
#include <iostream>
#include <fstream>
#include <SDL.h>
#include <chrono>
#include <math.h>
#include <SDL_keycode.h>
#include <bitset>

#include "utils/voxel_octree.h"

using LiteMath::float2;
using LiteMath::float3;
using LiteMath::float4;
using LiteMath::int2;
using LiteMath::int3;
using LiteMath::int4;
using LiteMath::uint2;
using LiteMath::uint3;
using LiteMath::uint4;
using LiteMath::max;
using LiteMath::min;

#define PI 3.14159265358979323846264338327950288

int SCREEN_WIDTH  = 800;
int SCREEN_HEIGHT = 600;

int WORLD_SIZE = 256;

int texture_size = 16;

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

using LiteMath::float3, LiteMath::float2;

float2 normalize_screen_offset(int x, int y, const int W, const int H) {
    float dx = float(x - W / 2) / (W / 2);
    float dy = float(H / 2 - y) / (H / 2);
    return float2(dx, dy);
}


float3 screen_offset(float3 dir, int x, int y, const int W, const int H) {
    float2 dv = normalize_screen_offset(x, y, W, H);

    float2 offset = float2(
        dv.x * tan(LiteMath::M_PI / 2 * 0.5f),
        dv.y * tan(LiteMath::M_PI / 3 * 0.5f)
    );

    float3 world_up = float3(0, 1, 0);
    float3 right = normalize(cross(world_up, dir));
    float3 up = normalize(cross(dir, right));

    float3 new_dir = dir + offset.x * right + offset.y * up;
    return normalize(new_dir);
}


int voxel_trace_dda(const float3& ro, const float3& rd, float max_t, int3& out_voxel, float3& out_normal, float3& out_hit_point)
{
    int3 voxel = int3(floor(ro.x), floor(ro.y), floor(ro.z));
    int3 step;
    float3 t_max, t_delta;

    // Инициализация DDA
    for (int i = 0; i < 3; i++) {
        if (fabs(rd[i]) < 1e-6) {
            step[i] = 0;
            t_max[i] = 1e6f;
            t_delta[i] = 1e6f;
        } else {
            step[i] = (rd[i] > 0) ? 1 : -1;
            float next_boundary = (rd[i] > 0) ? (voxel[i] + 1) : voxel[i];
            t_max[i] = (next_boundary - ro[i]) / rd[i];
            t_delta[i] = fabs(1.0f / rd[i]);
        }
    }
    
    float t = 0;
    int3 prev_voxel = voxel;
    float3 hit_point;
    
    while (t < max_t) {
        float terrain_height = sin(voxel.x * 0.1f) * 3.0f + cos(voxel.z * 0.1f) * 3.0f;
        
        if (voxel.y <= terrain_height && 
            fabs(voxel.x) <= 50 && 
            fabs(voxel.z) <= 50) {
            
            out_voxel = voxel;
            
            // Вычисляем точку пересечения
            hit_point = ro + rd * t;
            
            // Определяем нормаль
            if (voxel.x != prev_voxel.x) {
                out_normal = float3(-step.x, 0, 0);
                // Уточняем точку пересечения для X грани
                hit_point.x = (step.x > 0) ? voxel.x : voxel.x + 1;
            } else if (voxel.y != prev_voxel.y) {
                out_normal = float3(0, -step.y, 0);
                // Уточняем точку пересечения для Y грани
                hit_point.y = (step.y > 0) ? voxel.y : voxel.y + 1;
            } else {
                out_normal = float3(0, 0, -step.z);
                // Уточняем точку пересечения для Z грани
                hit_point.z = (step.z > 0) ? voxel.z : voxel.z + 1;
            }
            
            out_hit_point = hit_point;
            return fabs(voxel.y - terrain_height) < 2 ? SNOW : COBBLESTONE;
        }
        
        prev_voxel = voxel;

        // Шагаем к следующему вокселю
        if (t_max.x < t_max.y && t_max.x < t_max.z) {
            voxel.x += step.x;
            t = t_max.x;
            t_max.x += t_delta.x;
        } else if (t_max.y < t_max.z) {
            voxel.y += step.y;
            t = t_max.y;
            t_max.y += t_delta.y;
        } else {
            voxel.z += step.z;
            t = t_max.z;
            t_max.z += t_delta.z;
        }
    }
    
    return 0;
}

void render(const Camera &camera, uint32_t *out_image, int W, int H, VoxelTexture *voxel_textures)
{
    float3 light_source = normalize(float3(-1, 1.4, 0.2));
    float ray_length = 100;
    
    #pragma omp parallel for collapse(2)
    for (int y = 0; y < H; y++)
    {
        for (int x = 0; x < W; x++)
        {
            float3 cur_dir = screen_offset(camera.dir, x, y, W, H);
            float3 color = float3(0.1f, 0.1f, 0.1f); // фон
            float3 normal;
            int3 voxel_pos;
            int voxel_size;
            
            int id;
            float dist;

            int3 world_pos = int3(-WORLD_SIZE / 2);
            if ((id = traverse_octree(camera.pos, cur_dir, 0, WORLD_SIZE, world_pos, dist, voxel_pos, voxel_size)) >= 1) {
                float3 hit_point = camera.pos + cur_dir * dist;
                float3 local = hit_point - float3(voxel_pos);
                
                // Определяем, какая грань ближе всего к точке пересечения
                float3 to_center = local - float3(voxel_size) * 0.5f; // вектор к центру вокселя
                float3 abs_to_center = LiteMath::abs(to_center);
                
                // Находим грань с максимальным отклонением от центра
                if (abs_to_center.x >= abs_to_center.y && abs_to_center.x >= abs_to_center.z) {
                    normal = float3(LiteMath::sign(to_center.x), 0, 0);
                } else if (abs_to_center.y >= abs_to_center.z) {
                    normal = float3(0, LiteMath::sign(to_center.y), 0);
                } else {
                    normal = float3(0, 0, LiteMath::sign(to_center.z));
                }
                
                color = voxel_textures[id - 1].get_color(local, normal);
                //color = float3(1);
            }
            out_image[y*W + x] = float3_to_RGBA8(color);
        }
    }
}


void printBinary(int num, FILE *out) {
    for (int i = 31; i >= 0; i--) {
        fprintf(out, "%d", ((num >> i) & 1));
        if (i % 8 == 0) fprintf(out, " "); // разделитель для байтов
    }
    fprintf(out, "\n");
}
// You must include the command line parameters for your main function to be recognized by SDL
int main(int argc, char **args)
{

  std::cout << "Building octree with world size " << WORLD_SIZE << "...\n";

  build_SO(WORLD_SIZE);

  std::cout << "Built octree with length " << world_octree_len << '\n';
  FILE *out = fopen("out.txt", "w");
  for (int i = 0; i < world_octree_len; ++i) {
    printBinary(world_octree[i], out);
  }
  fclose(out);
  std::cout << octree_far[0] << '\n';
  

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
                                        SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN | SDL_WINDOW_INPUT_GRABBED);

  
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
  camera.pos = float3(0, 0, 0);

  auto time = std::chrono::high_resolution_clock::now();
  auto prev_time = time;
  float time_from_start = 0;
  uint32_t frameNum = 0;

  //SDL_ShowCursor(SDL_DISABLE);

  const Uint8* keys = SDL_GetKeyboardState(NULL);

  

  VoxelTexture voxel_textures[8];
  for (int i = 0; i < 8; ++i) {
    voxel_textures[i].generate_texture(i);
  }
  
  // Main loop
  while (running)
  {
    //update camera or scene
    prev_time = time;
    time = std::chrono::high_resolution_clock::now();

    //get delta time in seconds
    float dt = std::chrono::duration<float, std::milli>(time - prev_time).count() / 1000.0f;
    time_from_start += dt;
    frameNum++;

    if (frameNum % 10 == 0)
      printf("Render time: %f ms\n", 1000.0f*dt);
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
      
      case SDL_MOUSEMOTION:
        {
            int dx = -ev.motion.xrel;
            int dy = ev.motion.yrel;

            camera.angle.x += dx * camera.sensitivity * dt;
            camera.angle.y -= dy * camera.sensitivity * dt;

            if(camera.angle.y > PI / 2.001) camera.angle.y = PI / 2.001;
            if(camera.angle.y < -PI / 2.001) camera.angle.y = -PI / 2.001;

            camera.dir.x = cos(camera.angle.y) * cos(camera.angle.x);
            camera.dir.y = sin(camera.angle.y);
            camera.dir.z = cos(camera.angle.y) * sin(camera.angle.x);
            camera.dir = normalize(camera.dir);

            SDL_WarpMouseInWindow(window, SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2);
        }
        break;
      }
    }

    

    
    float3 forward = normalize(float3(camera.dir.x, 0, camera.dir.z));
    float3 right = normalize(cross(forward, float3(0, 1, 0)));

    if (keys[SDL_SCANCODE_W]) camera.pos += camera.speed * forward * dt;
    if (keys[SDL_SCANCODE_S]) camera.pos -= camera.speed * forward * dt;
    if (keys[SDL_SCANCODE_D]) camera.pos -= camera.speed * right * dt;
    if (keys[SDL_SCANCODE_A]) camera.pos += camera.speed * right * dt;
    if (keys[SDL_SCANCODE_SPACE]) camera.pos += float3(0, camera.speed, 0) * dt;
    if (keys[SDL_SCANCODE_LSHIFT]) camera.pos -= float3(0, camera.speed, 0) * dt;
    std::cout << camera.pos.x << ' ' << camera.pos.y << ' ' << camera.pos.z << '\n';
    // Render the scene
    render(camera, pixels.data(), SCREEN_WIDTH, SCREEN_HEIGHT, voxel_textures);

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