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
    int3(1, 0, 0),
    int3(0, 0, 1),
    int3(1, 0, 1),
    int3(0, 1, 0), 
    int3(1, 1, 0),
    int3(0, 1, 1),
    int3(1, 1, 1)
};

int check_block(int3 cur_pos) {
    if (cur_pos.y < sin(cur_pos.x * 0.1f) * 3.0f + cos(cur_pos.z * 0.1f) * 3.0f) {
        return 1;
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
        world_octree.push_back(dummy_node->block_id);
        world_octree_len++;
        return world_octree_len - 1 - cur_offset;
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
    return world_octree_len - new_len - cur_offset;
}

void free_dummy_octree(OctreeNode *cur_node) {
    for (int i = 0; i < 8; ++i) {
        if (cur_node->children[i] != NULL) {
            free_dummy_octree(cur_node->children[i]);
        }
    }
    free(cur_node);
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

