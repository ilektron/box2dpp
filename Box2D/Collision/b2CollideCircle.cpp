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
#include <Box2D/Collision/Shapes/b2CircleShape.h>
#include <Box2D/Collision/Shapes/b2PolygonShape.h>

using namespace box2d;

void box2d::b2CollideCircles(b2Manifold* manifold, const b2CircleShape* circleA,
                             const b2Transform& xfA, const b2CircleShape* circleB,
                             const b2Transform& xfB)
{
    manifold->pointCount = 0;

    b2Vec<float, 2> pA = b2Mul(xfA, circleA->m_p);
    b2Vec<float, 2> pB = b2Mul(xfB, circleB->m_p);

    b2Vec<float, 2> d = pB - pA;
    float distSqr = b2Dot(d, d);
    float rA = circleA->GetRadius(), rB = circleB->GetRadius();
    float radius = rA + rB;
    if (distSqr > radius * radius)
    {
        return;
    }

    manifold->type = b2Manifold::e_circles;
    manifold->localPoint = circleA->m_p;
    manifold->localNormal = {{0.0f, 0.0f}};
    manifold->pointCount = 1;

    manifold->points[0].localPoint = circleB->m_p;
    manifold->points[0].id.key = 0;
}

void box2d::b2CollidePolygonAndCircle(b2Manifold* manifold, const b2PolygonShape* polygonA,
                                      const b2Transform& xfA, const b2CircleShape* circleB,
                                      const b2Transform& xfB)
{
    manifold->pointCount = 0;

    // Compute circle position in the frame of the polygon.
    b2Vec<float, 2> c = b2Mul(xfB, circleB->m_p);
    b2Vec<float, 2> cLocal = b2MulT(xfA, c);

    // Find the min separating edge.
    std::size_t normalIndex = 0;
    float separation = -MAX_FLOAT;
    float radius = polygonA->GetRadius() + circleB->GetRadius();
    auto vertices = polygonA->GetVertices();
    auto normals = polygonA->GetNormals();

    for (std::size_t i = 0; i < vertices.size(); ++i)
    {
        float s = b2Dot(normals[i], cLocal - vertices[i]);

        if (s > radius)
        {
            // Early out.
            return;
        }

        if (s > separation)
        {
            separation = s;
            normalIndex = i;
        }
    }

    // Vertices that subtend the incident face.
    auto vertIndex1 = normalIndex;
    auto vertIndex2 = vertIndex1 + 1 < vertices.size() ? vertIndex1 + 1 : 0;
    b2Vec<float, 2> v1 = vertices[vertIndex1];
    b2Vec<float, 2> v2 = vertices[vertIndex2];

    // If the center is inside the polygon ...
    if (separation < EPSILON)
    {
        manifold->pointCount = 1;
        manifold->type = b2Manifold::e_faceA;
        manifold->localNormal = normals[normalIndex];
        manifold->localPoint = 0.5f * (v1 + v2);
        manifold->points[0].localPoint = circleB->m_p;
        manifold->points[0].id.key = 0;
        return;
    }

    // Compute barycentric coordinates
    float u1 = b2Dot(cLocal - v1, v2 - v1);
    float u2 = b2Dot(cLocal - v2, v1 - v2);
    if (u1 <= 0.0f)
    {
        if (b2DistanceSquared(cLocal, v1) > radius * radius)
        {
            return;
        }

        manifold->pointCount = 1;
        manifold->type = b2Manifold::e_faceA;
        manifold->localNormal = cLocal - v1;
        manifold->localNormal.Normalize();
        manifold->localPoint = v1;
        manifold->points[0].localPoint = circleB->m_p;
        manifold->points[0].id.key = 0;
    }
    else if (u2 <= 0.0f)
    {
        if (b2DistanceSquared(cLocal, v2) > radius * radius)
        {
            return;
        }

        manifold->pointCount = 1;
        manifold->type = b2Manifold::e_faceA;
        manifold->localNormal = cLocal - v2;
        manifold->localNormal.Normalize();
        manifold->localPoint = v2;
        manifold->points[0].localPoint = circleB->m_p;
        manifold->points[0].id.key = 0;
    }
    else
    {
        b2Vec<float, 2> faceCenter = 0.5f * (v1 + v2);
        float separation = b2Dot(cLocal - faceCenter, normals[vertIndex1]);
        if (separation > radius)
        {
            return;
        }

        manifold->pointCount = 1;
        manifold->type = b2Manifold::e_faceA;
        manifold->localNormal = normals[vertIndex1];
        manifold->localPoint = faceCenter;
        manifold->points[0].localPoint = circleB->m_p;
        manifold->points[0].id.key = 0;
    }
}
