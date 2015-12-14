/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

#include <Box2D/Collision/Shapes/b2CircleShape.h>
#include <new>

using namespace box2d;

b2Shape* b2CircleShape::Clone(b2BlockAllocator* allocator) const
{
    void* mem = allocator->Allocate(sizeof(b2CircleShape));
    auto clone = new (mem) b2CircleShape;
    *clone = *this;
    return clone;
}

int32_t b2CircleShape::GetChildCount() const
{
    return 1;
}

bool b2CircleShape::TestPoint(const b2Transform& transform, const b2Vec<float, 2>& p) const
{
    b2Vec<float, 2> center = transform.p + b2Mul(transform.q, m_p);
    b2Vec<float, 2> d = p - center;
    return b2Dot(d, d) <= GetRadius() * GetRadius();
}

// Collision Detection in Interactive 3D Environments by Gino van den Bergen
// From Section 3.1.2
// x = s + a * r
// norm(x) = radius
bool b2CircleShape::RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
                            const b2Transform& transform, int32_t childIndex) const
{
    B2_NOT_USED(childIndex);

    b2Vec<float, 2> position = transform.p + b2Mul(transform.q, m_p);
    b2Vec<float, 2> s = input.p1 - position;
    float b = b2Dot(s, s) - GetRadius() * GetRadius();

    // Solve quadratic equation.
    b2Vec<float, 2> r = input.p2 - input.p1;
    float c = b2Dot(s, r);
    float rr = b2Dot(r, r);
    float sigma = c * c - rr * b;

    // Check for negative discriminant and short segment.
    if (sigma < 0.0f || rr < EPSILON)
    {
        return false;
    }

    // Find the point of intersection of the line with the circle.
    float a = -(c + b2Sqrt(sigma));

    // Is the intersection point on the segment?
    if (0.0f <= a && a <= input.maxFraction * rr)
    {
        a /= rr;
        output->fraction = a;
        output->normal = s + a * r;
        output->normal.Normalize();
        return true;
    }

    return false;
}

void b2CircleShape::ComputeAABB(b2AABB* aabb, const b2Transform& transform,
                                int32_t childIndex) const
{
    B2_NOT_USED(childIndex);

    b2Vec<float, 2> p = transform.p + b2Mul(transform.q, m_p);
    aabb->lowerBound = p - GetRadius(); //.Set(p[b2VecX] - GetRadius(), p[b2VecY] - GetRadius());
    aabb->upperBound = p + GetRadius(); //.Set(p[b2VecX] + GetRadius(), p[b2VecY] + GetRadius());
}

void b2CircleShape::ComputeMass(b2MassData* massData, float density) const
{
    massData->mass = density * B2_PI * GetRadius() * GetRadius();
    massData->center = m_p;

    // inertia about the local origin
    massData->I = massData->mass * (0.5f * GetRadius() * GetRadius() + b2Dot(m_p, m_p));
}
