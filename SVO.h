#pragma once
#include <cstdint>
#include <vector>
#include <string>
#include <LiteMath.h>

using LiteMath::float3;

static constexpr uint32_t MAX_CHILD_POINTER = 0x7FFF;
static constexpr uint32_t IS_FAR_BIT = 0x8000;

#define CHILD_POINTER_MASK 0xFFFE0000
#define IS_FAR_MASK 0x00010000

struct AABB {
  float3 pos;
  float3 size;

  float dist(float3 ro, float3 rd, float minDist, float maxDist) {
    float3 half = size * 0.5f;
    float3 minB = pos - half;
    float3 maxB = pos + half;

    float3 t1 = (minB - ro) / rd;
    float3 t2 = (maxB - ro) / rd;

    float3 tMin3 = min(t1, t2);
    float3 tMax3 = max(t1, t2);

    minDist = std::max(std::max(tMin3.x, tMin3.y), tMin3.z);
    maxDist = std::min(std::min(tMax3.x, tMax3.y), tMax3.z);

    return maxDist >= minDist;
  }
};

struct SVO_Node {
  uint32_t mask;
};

struct SparseVoxelOctree {
  std::vector <SVO_Node> nodes;
};

struct TLSVO_Node {
  uint32_t mask;
};

struct TLSparseVoxelOctree {
  std::vector <TLSVO_Node> nodes;
  std::vector <SparseVoxelOctree> svo_trees;
};


float traverse_svo(
  SparseVoxelOctree svo, 
  float3 &ro, float3 &rd, 
  float3 &box_pos, float3 &box_size, 
  float3 &normal
) {
  float3 invD = float3(1.0) / rd;

  float3 t0v = (box_pos - box_size / 2 - ro) * invD;
  float3 t1v = (box_pos + box_size / 2 - ro) * invD;
  float3 tmin3 = min(t0v, t1v);
  float3 tmax3 = max(t0v, t1v);

  float tmin = std::max(tmin3.x, std::max(tmin3.y, tmin3.z));
  float tmax = std::min(tmax3.x, std::min(tmax3.y, tmax3.z));
  if (tmax < tmin) return -1.0f;

  return traverse_svo_node(svo, 0, ro, rd, box_pos, box_size, tmin, tmax, normal);
}

float traverse_tlsvo(
  TLSparseVoxelOctree &tlsvo, 
  float3 &ro, float3 &rd, 
  float3 &box_pos, float3 &box_size,
  float3 &normal
) {

  float3 invD = float3(1.0) / rd;

  float3 t0v = (box_pos - box_size / 2 - ro) * invD;
  float3 t1v = (box_pos + box_size / 2 - ro) * invD;
  float3 tmin3 = min(t0v, t1v);
  float3 tmax3 = max(t0v, t1v);

  float tmin = std::max(tmin3.x, std::max(tmin3.y, tmin3.z));
  float tmax = std::min(tmax3.x, std::min(tmax3.y, tmax3.z));
  if (tmax < tmin) return -1.0f;

  return traverse_tlsvo_node(tlsvo, 0, ro, rd, box_pos, box_size, tmin, tmax, normal);
}


inline int count_bits(uint32_t x)
{
    return __builtin_popcount(x);
}

inline int ordered_octant(int order, int first)
{
    return first ^ order;
}

void calc_octant_intervals(
    int first,
    float t0, float t1,
    const float3 &tm,
    float tEnter[8],
    float tExit[8])
{
    for (int i = 0; i < 8; i++)
    {
        float te = t0;
        float tx = t1;

        // X
        if (i & 1) {
            te = std::max(te, tm.x);
        } else {
            tx = std::min(tx, tm.x);
        }

        // Y
        if (i & 2) {
            te = std::max(te, tm.y);
        } else {
            tx = std::min(tx, tm.y);
        }

        // Z
        if (i & 4) {
            te = std::max(te, tm.z);
        } else {
            tx = std::min(tx, tm.z);
        }

        tEnter[i] = te;
        tExit[i] = tx;
    }
}


float traverse_svo_node(
  SparseVoxelOctree &svo, int cur_ind,
  float3 &ro, float3 &rd, 
  float3 &box_pos, float3 &box_size,
  float t0, float t1,
  float3 &normal
) {

    SVO_Node &node = svo.nodes[cur_ind];

    if (node.mask & CHILD_POINTER_MASK == 0) {
      return t0;
    }

    float3 invD = float3(1.0f) / rd;
    float3 tm = (box_pos - ro) * invD;

    int c0 = (tm.x < t0) ? 1 : 0;
    int c1 = (tm.y < t0) ? 2 : 0;
    int c2 = (tm.z < t0) ? 4 : 0;

    int first = c0 | c1 | c2;

    float tEnter[8], tExit[8];

    calc_octant_intervals(first, t0, t1, tm, tEnter, tExit);

    for (int i = 0; i < 8; i++)
    {
        int oct = ordered_octant(i, first);

        if (tExit[oct] < t0 || tEnter[oct] > t1)
            continue;

        if (node.mask & (1 << oct))
        {
            int childId = (node.mask & CHILD_POINTER_MASK >> 9) + count_bits(node.mask & ((1 << oct) - 1));

            float3 cmin = box_pos - box_size / 2;
            float3 cmax = box_pos + box_size / 2;

            
            if (oct & 1) cmin.x = box_pos.x; else cmax.x = box_pos.x;
            if (oct & 2) cmin.y = box_pos.y; else cmax.y = box_pos.y;
            if (oct & 4) cmin.z = box_pos.z; else cmax.z = box_pos.z;

            float hit = traverse_svo_node(
                svo, childId,
                ro, rd,
                (cmin + cmax) / 2, cmax - cmin,
                tEnter[oct], tExit[oct], 
                normal);

            if (hit >= 0.0f)
                return hit;
        }
    }
    return -1.0f;
}

float traverse_tlsvo_node(
  TLSparseVoxelOctree &tlsvo, int cur_ind,
  float3 &ro, float3 &rd, 
  float3 &box_pos, float3 &box_size,
  float t0, float t1,
  float3 &normal
) {

    TLSVO_Node &node = tlsvo.nodes[cur_ind];

    if (node.mask & CHILD_POINTER_MASK == 0) {
      return traverse_svo(tlsvo.svo_trees[cur_ind], ro, rd, box_pos, box_size, normal);
    }

    float3 invD = float3(1.0f) / rd;
    float3 tm = (box_pos - ro) * invD;

    int c0 = (tm.x < t0) ? 1 : 0;
    int c1 = (tm.y < t0) ? 2 : 0;
    int c2 = (tm.z < t0) ? 4 : 0;

    int first = c0 | c1 | c2;

    float tEnter[8], tExit[8];

    calc_octant_intervals(first, t0, t1, tm, tEnter, tExit);

    for (int i = 0; i < 8; i++)
    {
        int oct = ordered_octant(i, first);

        if (tExit[oct] < t0 || tEnter[oct] > t1)
            continue;

        if (node.mask & (1 << oct))
        {
            int childId = (node.mask & CHILD_POINTER_MASK >> 9) + count_bits(node.mask & ((1 << oct) - 1));

            float3 cmin = box_pos - box_size / 2;
            float3 cmax = box_pos + box_size / 2;

            
            if (oct & 1) cmin.x = box_pos.x; else cmax.x = box_pos.x;
            if (oct & 2) cmin.y = box_pos.y; else cmax.y = box_pos.y;
            if (oct & 4) cmin.z = box_pos.z; else cmax.z = box_pos.z;

            float hit = traverse_tlsvo_node(
                tlsvo, childId,
                ro, rd,
                (cmin + cmax) / 2, cmax - cmin,
                tEnter[oct], tExit[oct], 
                normal);

            if (hit >= 0.0f)
                return hit;
        }
    }
    return -1.0f;
}


void build_tlsvo(TLSparseVoxelOctree &tlsvo, int cur_ind, int size) {
  if (size == 1) {
    TLSVO_Node node;
    if (cur_ind > 32768) {
      node.mask |= IS_FAR_BIT;
      node.mask |= (cur_ind + 1) << 9;
      tlsvo.nodes.push_back()
    }
    tlsvo.nodes.append(TLSVO_Node)
  }
}