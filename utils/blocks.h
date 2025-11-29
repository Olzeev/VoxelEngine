#include <vector>
#include <string>
#include <iostream>


int TEXTURE_SIZE = 16;

enum BlockId {
    EMPTY,
    DIRT, 
    GRASS, 
    STONE, 
    COBBLESTONE, 
    OAK, 
    SAND, 
    SNOW, 
    WATER
};

int texture_mask[8][6] = {  // +x -x +z -z +y -y
    {0, 0, 0, 0, 0, 0}, 
    {1, 1, 1, 1, 0, 2}, 
    {0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0}, 
    {1, 1, 1, 1, 0, 0}, 
    {0, 0, 0, 0, 0, 0}, 
    {0, 0, 0, 0, 0, 0}, 
    {0, 0, 0, 0, 0, 0}
};

std::vector <std::string> path_to_texture = {
    "textures/dirt/", 
    "textures/grass/", 
    "textures/stone/", 
    "textures/cobblestone/",
    "textures/oak/", 
    "textures/sand/", 
    "textures/snow/", 
    "textures/water/", 
};


struct VoxelTexture {
    std::vector<float> textures[6]; // +x -x +z -z +y -y
    int id;

    void generate_texture(int id1) {
        id = id1;
        for (int i = 0; i < 6; ++i) {
            std::string path = path_to_texture[id1] + std::to_string(texture_mask[id1][i]);
            path += ".png";
            std::cout << path << '\n';
            read_image_rgb(path, textures[i], TEXTURE_SIZE, TEXTURE_SIZE);
        }
    }
    float3 get_color(float3 local_coord, float3 normal) {
        int texture_x, texture_y;
        int ind = -1;
        if (normal.x != 0) {
            texture_y = floor(local_coord.y * TEXTURE_SIZE);
            texture_x = floor(local_coord.z * TEXTURE_SIZE);
            ind = 0;
        } else if (normal.y != 0) {
            texture_y = floor(local_coord.x * TEXTURE_SIZE);
            texture_x = floor(local_coord.z * TEXTURE_SIZE);
            ind = 4;
        } else {
            texture_x = floor(local_coord.x * TEXTURE_SIZE);
            texture_y = floor(local_coord.y * TEXTURE_SIZE);
            ind = 2;
        }
        texture_y = TEXTURE_SIZE - 1 - texture_y;
        return float3(
            textures[ind][(texture_y * TEXTURE_SIZE + texture_x) * 3], 
            textures[ind][(texture_y * TEXTURE_SIZE + texture_x) * 3 + 1], 
            textures[ind][(texture_y * TEXTURE_SIZE + texture_x) * 3 + 2]
        );
    }
};