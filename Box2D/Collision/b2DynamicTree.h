/*
* Copyright (c) 2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef B2_DYNAMIC_TREE_H
#define B2_DYNAMIC_TREE_H

#include <Box2D/Collision/b2Collision.h>

#include <vector>

namespace box2d
{
    
constexpr int NULL_NODE = -1;

/// A node in the dynamic tree. The client does not interact with this directly.
struct b2TreeNode
{
    bool IsLeaf() const
    {
        return child1 == NULL_NODE;
    }

    /// Enlarged AABB
    b2AABB aabb;

    void* userData;

    union
    {
        int32_t parent;
        int32_t next;
    };

    int32_t child1;
    int32_t child2;

    // leaf = 0, free node = -1
    int32_t height;
};

/// A dynamic AABB tree broad-phase, inspired by Nathanael Presson's btDbvt.
/// A dynamic tree arranges data in a binary tree to accelerate
/// queries such as volume queries and ray casts. Leafs are proxies
/// with an AABB. In the tree we expand the proxy AABB by b2_fatAABBFactor
/// so that the proxy AABB is bigger than the client object. This allows the
/// client
/// object to move by small amounts without triggering a tree update.
///
/// Nodes are pooled and relocatable, so we use node indices rather than
/// pointers.
class b2DynamicTree
{
public:
    /// Constructing the tree initializes the node pool.
    b2DynamicTree();

    /// Destroy the tree, freeing the node pool.
    ~b2DynamicTree();

    /// Create a proxy. Provide a tight fitting AABB and a userData pointer.
    int32_t CreateProxy(const b2AABB& aabb, void* userData);

    /// Destroy a proxy. This asserts if the id is invalid.
    void DestroyProxy(int32_t proxyId);

    /// Move a proxy with a swepted AABB. If the proxy has moved outside of its
    /// fattened AABB,
    /// then the proxy is removed from the tree and re-inserted. Otherwise
    /// the function returns immediately.
    /// @return true if the proxy was re-inserted.
    bool MoveProxy(int32_t proxyId, const b2AABB& aabb1, const b2Vec2& displacement);

    /// Get proxy user data.
    /// @return the proxy user data or 0 if the id is invalid.
    void* GetUserData(int32_t proxyId) const;

    /// Get the fat AABB for a proxy.
    const b2AABB& GetFatAABB(int32_t proxyId) const;

    /// Query an AABB for overlapping proxies. The callback class
    /// is called for each proxy that overlaps the supplied AABB.
    template <typename T>
    void Query(T* callback, const b2AABB& aabb) const;

    /// Ray-cast against the proxies in the tree. This relies on the callback
    /// to perform a exact ray-cast in the case were the proxy contains a shape.
    /// The callback also performs the any collision filtering. This has
    /// performance
    /// roughly equal to k * log(n), where k is the number of collisions and n is
    /// the
    /// number of proxies in the tree.
    /// @param input the ray-cast input data. The ray extends from p1 to p1 +
    /// maxFraction * (p2 -
    /// p1).
    /// @param callback a callback class that is called for each proxy that is hit
    /// by the ray.
    template <typename T>
    void RayCast(T* callback, const b2RayCastInput& input) const;

    /// Validate this tree. For testing.
    void Validate() const;

    /// Compute the height of the binary tree in O(N) time. Should not be
    /// called often.
    int32_t GetHeight() const;

    /// Get the maximum balance of an node in the tree. The balance is the
    /// difference
    /// in height of the two children of a node.
    int32_t GetMaxBalance() const;

    /// Get the ratio of the sum of the node areas to the root area.
    float32 GetAreaRatio() const;

    /// Build an optimal tree. Very expensive. For testing.
    void RebuildBottomUp();

    /// Shift the world origin. Useful for large worlds.
    /// The shift formula is: position -= newOrigin
    /// @param newOrigin the new origin with respect to the old origin
    void ShiftOrigin(const b2Vec2& newOrigin);

private:
    int32_t AllocateNode();
    void FreeNode(int32_t node);

    void InsertLeaf(int32_t node);
    void RemoveLeaf(int32_t node);

    int32_t Balance(int32_t index);

    int32_t ComputeHeight() const;
    int32_t ComputeHeight(int32_t nodeId) const;

    void ValidateStructure(int32_t index) const;
    void ValidateMetrics(int32_t index) const;

    int32_t m_root;

    std::vector<b2TreeNode> m_nodes;
    int32_t m_nodeCount;
    int32_t m_nodeCapacity;

    int32_t m_freeList;

    /// This is used to incrementally traverse the tree for re-balancing.
    uint32_t m_path;

    int32_t m_insertionCount;
};

inline void* b2DynamicTree::GetUserData(int32_t proxyId) const
{
    b2Assert(0 <= proxyId && proxyId < m_nodeCapacity);
    return m_nodes[proxyId].userData;
}

inline const b2AABB& b2DynamicTree::GetFatAABB(int32_t proxyId) const
{
    b2Assert(0 <= proxyId && proxyId < m_nodeCapacity);
    return m_nodes[proxyId].aabb;
}

template <typename T>
inline void b2DynamicTree::Query(T* callback, const b2AABB& aabb) const
{
    std::vector<int32_t> stack(256);
    stack.push_back(m_root);

    while (stack.size() > 0)
    {
        int32_t nodeId = stack.back();
        stack.pop_back();
        if (nodeId == NULL_NODE)
        {
            continue;
        }

        const b2TreeNode* node = &m_nodes[nodeId];

        if (b2TestOverlap(node->aabb, aabb))
        {
            if (node->IsLeaf())
            {
                bool proceed = callback->QueryCallback(nodeId);
                if (proceed == false)
                {
                    return;
                }
            }
            else
            {
                stack.push_back(node->child1);
                stack.push_back(node->child2);
            }
        }
    }
}

template <typename T>
inline void b2DynamicTree::RayCast(T* callback, const b2RayCastInput& input) const
{
    b2Vec2 p1 = input.p1;
    b2Vec2 p2 = input.p2;
    b2Vec2 r = p2 - p1;
    b2Assert(LengthSquared(r) > 0.0f);
    Normalize(r);

    // v is perpendicular to the segment.
    b2Vec2 v = b2Cross(1.0f, r);
    b2Vec2 abs_v = b2Abs(v);

    // Separating axis for segment (Gino, p80).
    // |dot(v, p1 - c)| > dot(|v|, h)

    float32 maxFraction = input.maxFraction;

    // Build a bounding box for the segment.
    b2AABB segmentAABB;
    {
        b2Vec2 t = p1 + maxFraction * (p2 - p1);
        segmentAABB.lowerBound = b2Min(p1, t);
        segmentAABB.upperBound = b2Max(p1, t);
    }

    std::vector<int32_t> stack(256);
    stack.push_back(m_root);

    while (stack.size() > 0)
    {
        int32_t nodeId = stack.back();
        stack.pop_back();
        if (nodeId == NULL_NODE)
        {
            continue;
        }

        const b2TreeNode* node = &m_nodes[nodeId];

        if (b2TestOverlap(node->aabb, segmentAABB) == false)
        {
            continue;
        }

        // Separating axis for segment (Gino, p80).
        // |dot(v, p1 - c)| > dot(|v|, h)
        b2Vec2 c = node->aabb.GetCenter();
        b2Vec2 h = node->aabb.GetExtents();
        float32 separation = std::abs(b2Dot(v, p1 - c)) - b2Dot(abs_v, h);
        if (separation > 0.0f)
        {
            continue;
        }

        if (node->IsLeaf())
        {
            b2RayCastInput subInput;
            subInput.p1 = input.p1;
            subInput.p2 = input.p2;
            subInput.maxFraction = maxFraction;

            float32 value = callback->RayCastCallback(subInput, nodeId);

            if (value == 0.0f)
            {
                // The client has terminated the ray cast.
                return;
            }

            if (value > 0.0f)
            {
                // Update segment bounding box.
                maxFraction = value;
                b2Vec2 t = p1 + maxFraction * (p2 - p1);
                segmentAABB.lowerBound = b2Min(p1, t);
                segmentAABB.upperBound = b2Max(p1, t);
            }
        }
        else
        {
            stack.push_back(node->child1);
            stack.push_back(node->child2);
        }
    }
}
}

#endif
