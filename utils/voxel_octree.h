#include <vector>
#include "LiteMath.h"

using LiteMath::int3;
using LiteMath::float3;

unsigned int CHILD_MASK = 0xfffe0000;
unsigned int FAR_MASK = 0x00010000;
unsigned int VALID_MASK = 0x0000ff00;
unsigned int LEAF_MASK = 0x000000ff;

int world_octree_len = 0;
std::vector <unsigned int> world_octree;
int octree_far_len = 0;
std::vector <unsigned int> octree_far;

int3 node_offset[8] = {
    int3(0, 0, 0), 
    int3(0, 0, 1),
    int3(0, 1, 0),
    int3(0, 1, 1),
    int3(1, 0, 0), 
    int3(1, 0, 1),
    int3(1, 1, 0),
    int3(1, 1, 1),
};

char next_node_table[8][3] = {
    {4, 2, 1}, 
    {5, 3, -1}, 
    {6, -1, 3}, 
    {7, -1, -1}, 
    {-1, 6, 5}, 
    {-1, 7, -1}, 
    {-1, -1, 7}, 
    {-1, -1, -1}
};

int check_block(int3 cur_pos) {
    if (cur_pos.x * cur_pos.x  + (cur_pos.y + 50) * (cur_pos.y + 50) + cur_pos.z * cur_pos.z <= 2500) {
        return 2;
    }
    return 0;
}

struct SparseOctree {
    int general_size;
    int len = 0;
    std::vector <int> nodes;
};

struct OctreeNode {
    OctreeNode *children[8];
    int block_id;
};

OctreeNode * build_dummy_octree(int cur_size, int3 cur_pos) {
    OctreeNode *node = new OctreeNode;
    if (cur_size == 1) {
        int id = check_block(cur_pos);
        node->block_id = id;
        for (int i = 0; i < 8; ++i) {
            node->children[i] = NULL;
        }
        return node;
    }
    int new_size = cur_size / 2;
    int id = -1;
    for (int i = 0; i < 8; ++i) {
        node->children[i] = build_dummy_octree(new_size, cur_pos + node_offset[i] * new_size);
        if (i == 0) { 
            id = node->children[i]->block_id;
        } else {
            if (node->children[i]->block_id != id) {
                id = -1;
            }
        }
    }
    node->block_id = id;
    return node;
}


int build_real_octree(OctreeNode *dummy_node, int cur_offset) { // <- believe that dummy_node is alreay built and is in wrold_octree
    if (dummy_node->block_id != -1) {
        return 0;
    }
    int new_len = 0;
    for (int i = 0; i < 8; ++i) {
        if (dummy_node->children[i] != NULL) {
            if (dummy_node->children[i]->block_id != -1) {
                world_octree.push_back(dummy_node->children[i]->block_id);
                new_len++;
            } else {
                unsigned int new_node = 0;
                for (int j = 0; j < 8; ++j) {
                    if (dummy_node->children[i]->children[j] != NULL) {
                        new_node |= ((1 << 15) >> j);
                        
                    }
                    if (dummy_node->children[i]->children[j]->block_id != -1) {
                        new_node |= ((1 << 7) >> j);
                    }
                    
                }
                world_octree.push_back(new_node);
                new_len++;
            }
        }
    }
    int ind = 0;
    world_octree_len += new_len;
    int old_octree_len = world_octree_len;
    for (int i = 0; i < 8; ++i) {
        if (dummy_node->children[i] != NULL) {
            unsigned int offset = build_real_octree(dummy_node->children[i], old_octree_len - new_len + ind);
            if (offset >= 32768) { // 2^15
                world_octree[old_octree_len - new_len + ind] |= FAR_MASK;
                world_octree[old_octree_len - new_len + ind] |= (octree_far_len << 17);
                octree_far.push_back(offset);
                octree_far_len++;
            } else {
                world_octree[old_octree_len - new_len + ind] |= (offset << 17);
            }
            ind++;
        }       
    }
    return old_octree_len - new_len - cur_offset;
}

void free_dummy_octree(OctreeNode *cur_node) {
    for (int i = 0; i < 8; ++i) {
        if (cur_node->children[i] != NULL) {
            free_dummy_octree(cur_node->children[i]);
        }
    }
    delete cur_node;
}

void print_dummy_octree(OctreeNode *cur_node) {
    std::cout << "-----------------\n";
    std::cout << cur_node->block_id << '\n';
    for (int i = 0; i < 8; ++i) {
        if (cur_node->children[i] != 0) {
            print_dummy_octree(cur_node->children[i]);
        }
    }
    std::cout << "--------+--------\n";
}

void build_SO(int world_size) {
    OctreeNode *dummy_root = build_dummy_octree(world_size, int3(-world_size / 2, -world_size / 2, -world_size / 2));
    if (dummy_root->block_id != -1) {
        world_octree.push_back(dummy_root->block_id);
        world_octree_len = 1;
        return;
    }
    unsigned int cur_node = (1 << 17);
    for (int i = 0; i < 8; ++i) {
        if (dummy_root->children[i] != NULL) {
            cur_node |= ((1 << 15) >> i);
        }
        if (dummy_root->children[i]->block_id != -1) {
            cur_node |= ((1 << 7) >> i);
        }
    }

    world_octree.push_back(cur_node);
    world_octree_len = 1;
    build_real_octree(dummy_root, 0);
    free_dummy_octree(dummy_root);
}

void printBinary(int num) {
    for (int i = 31; i >= 0; i--) {
        std::cout << ((num >> i) & 1);
        if (i % 8 == 0) std::cout << " "; // разделитель для байтов
    }
    std::cout << '\n';
}

int traverse_octree(float3 ray_origin, float3 ray_dir, int cur_ind, int cur_size, int3 cur_pos, 
    float &dist, int3 &voxel_pos, int &voxel_size) {
    float3 t0 = (float3(cur_pos) - ray_origin) / ray_dir;
    float3 t1 = (float3(cur_pos) + float3(cur_size) - ray_origin) / ray_dir;
    float3 t_min = float3(ray_dir.x > 0 ? t0.x : t1.x, ray_dir.y > 0 ? t0.y : t1.y, ray_dir.z > 0 ? t0.z : t1.z);
    float3 t_max = float3(ray_dir.x > 0 ? t1.x : t0.x, ray_dir.y > 0 ? t1.y : t0.y, ray_dir.z > 0 ? t1.z : t0.z);
    float t_enter = std::max(t_min.x, std::max(t_min.y, t_min.z));
    float t_exit = std::min(t_max.x, std::min(t_max.y, t_max.z));
    
    
    if (t_enter < t_exit && t_exit > 0) {
        
        if (((world_octree[cur_ind] & CHILD_MASK) == 0 && !(world_octree[cur_ind] & FAR_MASK))) {
            dist = t_enter;
            voxel_pos = cur_pos;
            voxel_size = cur_size;
            return world_octree[cur_ind];
        }
        bool flag = false;
        int min_id;
        int half_size = cur_size / 2;
        int ind = 0;
        int new_ind;
        if (world_octree[cur_ind] & FAR_MASK) 
            new_ind = cur_ind + octree_far[(world_octree[cur_ind] & CHILD_MASK) >> 17];
        else
            new_ind = cur_ind + ((world_octree[cur_ind] & CHILD_MASK) >> 17);

        for (int i = 0; i < 8; ++i) {
            int cur_id;
            float cur_dist;
            int3 cur_voxel_pos;
            int cur_voxel_size;
            if (world_octree[cur_ind] & ((1 << 15) >> i)) {
                cur_id = traverse_octree(ray_origin, ray_dir, new_ind + ind, half_size, cur_pos + node_offset[i] * half_size, cur_dist, cur_voxel_pos, cur_voxel_size);
                ind++;
                if (cur_id != -1 && cur_id != 0) {
                    if (!flag || cur_dist < dist) {
                        min_id = cur_id;
                        dist = cur_dist;
                        voxel_pos = cur_voxel_pos;
                        voxel_size = cur_voxel_size;
                        flag = true;
                    }
                }
                
            }
        } 
        if (flag) {
            return min_id;
        }
        return -1;
    }
    return -1;
}