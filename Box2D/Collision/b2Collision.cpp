/*
* Copyright (c) 2007-2009 Erin Catto http://www.box2d.org
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

#include <Box2D/Collision/b2Collision.h>
#include <Box2D/Collision/b2Distance.h>

using namespace box2d;

void b2WorldManifold::Initialize(const b2Manifold* manifold, const b2Transform& xfA,
                                 float radiusA, const b2Transform& xfB, float radiusB)
{
    if (manifold->pointCount == 0)
    {
        return;
    }

    switch (manifold->type)
    {
        case b2Manifold::e_circles:
        {
            normal = {{1.0f, 0.0f}};
            b2Vec<float, 2> pointA = b2Mul(xfA, manifold->localPoint);
            b2Vec<float, 2> pointB = b2Mul(xfB, manifold->points[0].localPoint);
            if (b2DistanceSquared(pointA, pointB) > EPSILON * EPSILON)
            {
                normal = pointB - pointA;
                normal.Normalize();
            }

            b2Vec<float, 2> cA = pointA + radiusA * normal;
            b2Vec<float, 2> cB = pointB - radiusB * normal;
            points[0] = 0.5f * (cA + cB);
            separations[0] = b2Dot(cB - cA, normal);
        }
        break;

        case b2Manifold::e_faceA:
        {
            normal = b2Mul(xfA.q, manifold->localNormal);
            b2Vec<float, 2> planePoint = b2Mul(xfA, manifold->localPoint);

            for (int32_t i = 0; i < manifold->pointCount; ++i)
            {
                b2Vec<float, 2> clipPoint = b2Mul(xfB, manifold->points[i].localPoint);
                b2Vec<float, 2> cA = clipPoint + (radiusA - b2Dot(clipPoint - planePoint, normal)) * normal;
                b2Vec<float, 2> cB = clipPoint - radiusB * normal;
                points[i] = 0.5f * (cA + cB);
                separations[i] = b2Dot(cB - cA, normal);
            }
        }
        break;

        case b2Manifold::e_faceB:
        {
            normal = b2Mul(xfB.q, manifold->localNormal);
            b2Vec<float, 2> planePoint = b2Mul(xfB, manifold->localPoint);

            for (int32_t i = 0; i < manifold->pointCount; ++i)
            {
                b2Vec<float, 2> clipPoint = b2Mul(xfA, manifold->points[i].localPoint);
                b2Vec<float, 2> cB = clipPoint + (radiusB - b2Dot(clipPoint - planePoint, normal)) * normal;
                b2Vec<float, 2> cA = clipPoint - radiusA * normal;
                points[i] = 0.5f * (cA + cB);
                separations[i] = b2Dot(cA - cB, normal);
            }

            // Ensure normal points from A to B.
            normal = -normal;
        }
        break;
    }
}

void box2d::b2GetPointStates(b2PointState state1[MAX_MANIFOLD_POINTS],
                             b2PointState state2[MAX_MANIFOLD_POINTS], const b2Manifold* manifold1,
                             const b2Manifold* manifold2)
{
    for (int32_t i = 0; i < MAX_MANIFOLD_POINTS; ++i)
    {
        state1[i] = b2_nullState;
        state2[i] = b2_nullState;
    }

    // Detect persists and removes.
    for (int32_t i = 0; i < manifold1->pointCount; ++i)
    {
        b2ContactID id = manifold1->points[i].id;

        state1[i] = b2_removeState;

        for (int32_t j = 0; j < manifold2->pointCount; ++j)
        {
            if (manifold2->points[j].id.key == id.key)
            {
                state1[i] = b2_persistState;
                break;
            }
        }
    }

    // Detect persists and adds.
    for (int32_t i = 0; i < manifold2->pointCount; ++i)
    {
        b2ContactID id = manifold2->points[i].id;

        state2[i] = b2_addState;

        for (int32_t j = 0; j < manifold1->pointCount; ++j)
        {
            if (manifold1->points[j].id.key == id.key)
            {
                state2[i] = b2_persistState;
                break;
            }
        }
    }
}

// From Real-time Collision Detection, p179.
bool b2AABB::RayCast(b2RayCastOutput* output, const b2RayCastInput& input) const
{
    float tmin = -MAX_FLOAT;
    float tmax = MAX_FLOAT;

    b2Vec<float, 2> p = input.p1;
    b2Vec<float, 2> d = input.p2 - input.p1;
    b2Vec<float, 2> absD = b2Abs(d);

    b2Vec<float, 2> normal;

    for (int32_t i = 0; i < 2; ++i)
    {
        if (absD[i] < EPSILON)
        {
            // Parallel.
            if (p[i] < lowerBound[i] || upperBound[i] < p[i])
            {
                return false;
            }
        }
        else
        {
            float inv_d = 1.0f / d[i];
            float t1 = (lowerBound[i] - p[i]) * inv_d;
            float t2 = (upperBound[i] - p[i]) * inv_d;

            // Sign of the normal vector.
            float s = -1.0f;

            if (t1 > t2)
            {
                b2Swap(t1, t2);
                s = 1.0f;
            }

            // Push the min up
            if (t1 > tmin)
            {
                normal = {{0.0f, 0.0f}};
                normal[i] = s;
                tmin = t1;
            }

            // Pull the max down
            tmax = b2Min(tmax, t2);

            if (tmin > tmax)
            {
                return false;
            }
        }
    }

    // Does the ray start inside the box?
    // Does the ray intersect beyond the max fraction?
    if (tmin < 0.0f || input.maxFraction < tmin)
    {
        return false;
    }

    // Intersection.
    output->fraction = tmin;
    output->normal = normal;
    return true;
}

// Sutherland-Hodgman clipping.
int32_t box2d::b2ClipSegmentToLine(std::array<b2ClipVertex, 2>& vOut, const std::array<b2ClipVertex, 2>& vIn,
                                   const b2Vec<float, 2>& normal, float offset, int32_t vertexIndexA)
{
    // Start with no output points
    int32_t numOut = 0;

    // Calculate the distance of end points to the line
    float distance0 = b2Dot(normal, vIn[0].v) - offset;
    float distance1 = b2Dot(normal, vIn[1].v) - offset;

    // If the points are behind the plane
    if (distance0 <= 0.0f)
        vOut[numOut++] = vIn[0];
    if (distance1 <= 0.0f)
        vOut[numOut++] = vIn[1];

    // If the points are on different sides of the plane
    if (distance0 * distance1 < 0.0f)
    {
        // Find intersection point of edge and plane
        float interp = distance0 / (distance0 - distance1);
        vOut[numOut].v = vIn[0].v + interp * (vIn[1].v - vIn[0].v);

        // VertexA is hitting edgeB.
        vOut[numOut].id.cf.indexA = static_cast<uint8_t>(vertexIndexA);
        vOut[numOut].id.cf.indexB = vIn[0].id.cf.indexB;
        vOut[numOut].id.cf.typeA = b2ContactFeature::e_vertex;
        vOut[numOut].id.cf.typeB = b2ContactFeature::e_face;
        ++numOut;
    }

    return numOut;
}

bool box2d::b2TestOverlap(const b2Shape* shapeA, int32_t indexA, const b2Shape* shapeB,
                          int32_t indexB, const b2Transform& xfA, const b2Transform& xfB)
{
    b2DistanceInput input;
    input.proxyA.Set(shapeA, indexA);
    input.proxyB.Set(shapeB, indexB);
    input.transformA = xfA;
    input.transformB = xfB;
    input.useRadii = true;

    b2SimplexCache cache;
    cache.count = 0;

    b2DistanceOutput output;

    b2Distance(&output, &cache, &input);

    return output.distance < 10.0f * EPSILON;
}
