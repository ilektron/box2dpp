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

#include <Box2D/Collision/Shapes/b2PolygonShape.h>
#include <new>

using namespace box2d;

b2Shape* b2PolygonShape::Clone(b2BlockAllocator* allocator) const
{
    void* mem = allocator->Allocate(sizeof(b2PolygonShape));
    auto clone = new (mem) b2PolygonShape;
    *clone = *this;
    return clone;
}

void b2PolygonShape::SetAsBox(float32 hx, float32 hy)
{
    m_vertices = {{{-hx, -hy}},
                  {{hx, -hy}},
                  {{hx, hy}},
                  {{-hx, hy}}};
    m_normals = {{{0.0f, -1.0f}},
                 {{1.0f, 0.0f}},
                 {{0.0f, 1.0f}},
                 {{-1.0f, 0.0f}}};
                 
    m_centroid = {{0.0f, 0.0f}};
}

void b2PolygonShape::SetAsBox(float32 hx, float32 hy, const b2Vec2& center, float32 angle)
{
    SetAsBox(hx, hy);
    m_centroid = center;

    b2Transform xf;
    xf.p = center;
    xf.q.Set(angle);

    // Transform vertices and normals.
    for (int32_t i = 0; i < m_vertices.size(); ++i)
    {
        m_vertices[i] = b2Mul(xf, m_vertices[i]);
        m_normals[i] = b2Mul(xf.q, m_normals[i]);
    }
}

int32_t b2PolygonShape::GetChildCount() const
{
    return 1;
}

static b2Vec2 ComputeCentroid(const std::vector<b2Vec2>& vs)
{
    b2Assert(vs.size() >= 3);

    b2Vec2 c{{0.0f, 0.0f}};
    float32 area = 0.0f;

    // pRef is the reference point for forming triangles.
    // It's location doesn't change the result (except for rounding error).
    b2Vec2 pRef{{0.0f, 0.0f}};
#if 0
	// This code would put the reference point inside the polygon.
	for (int32_t i = 0; i < count; ++i)
	{
		pRef += vs[i];
	}
	pRef *= 1.0f / count;
#endif

    const float32 inv3 = 1.0f / 3.0f;

    for (int32_t i = 0; i < vs.size(); ++i)
    {
        // Triangle vertices.
        b2Vec2 p1 = pRef;
        b2Vec2 p2 = vs[i];
        b2Vec2 p3 = i + 1 < vs.size() ? vs[i + 1] : vs[0];

        b2Vec2 e1 = p2 - p1;
        b2Vec2 e2 = p3 - p1;

        float32 D = b2Cross(e1, e2);

        float32 triangleArea = 0.5f * D;
        area += triangleArea;

        // Area weighted centroid
        c += triangleArea * inv3 * (p1 + p2 + p3);
    }

    // Centroid
    b2Assert(area > EPSILON);
    c *= 1.0f / area;
    return c;
}

void b2PolygonShape::Set(const b2Vec2* vertices, int32_t count)
{
    b2Assert(3 <= count && count <= MAX_POLYGON_VERTICES);
    if (count < 3)
    {
        SetAsBox(1.0f, 1.0f);
        return;
    }

    int32_t n = b2Min(count, MAX_POLYGON_VERTICES);

    // Perform welding and copy vertices into local buffer.
//     b2Vec2 ps[MAX_POLYGON_VERTICES];
    std::array<b2Vec2, MAX_POLYGON_VERTICES> ps;
    int32_t tempCount = 0;
    for (int32_t i = 0; i < n; ++i)
    {
        b2Vec2 v = vertices[i];

        bool unique = true;
        for (int32_t j = 0; j < tempCount; ++j)
        {
            if (b2DistanceSquared(v, ps[j]) < 0.5f * LINEAR_SLOP)
            {
                unique = false;
                break;
            }
        }

        if (unique)
        {
            ps[tempCount++] = v;
        }
    }

    n = tempCount;
    if (n < 3)
    {
        // Polygon is degenerate.
        b2Assert(false);
        SetAsBox(1.0f, 1.0f);
        return;
    }

    // Create the convex hull using the Gift wrapping algorithm
    // http://en.wikipedia.org/wiki/Gift_wrapping_algorithm

    // Find the right most point on the hull
    int32_t i0 = 0;
    float32 x0 = ps[0][b2VecX];
    for (int32_t i = 1; i < n; ++i)
    {
        float32 x = ps[i][b2VecX];
        if (x > x0 || (x == x0 && ps[i][b2VecY] < ps[i0][b2VecY]))
        {
            i0 = i;
            x0 = x;
        }
    }

    int32_t hull[MAX_POLYGON_VERTICES];
    int32_t m = 0;
    int32_t ih = i0;

    for (;;)
    {
        hull[m] = ih;

        int32_t ie = 0;
        for (int32_t j = 1; j < n; ++j)
        {
            if (ie == ih)
            {
                ie = j;
                continue;
            }

            b2Vec2 r = ps[ie] - ps[hull[m]];
            b2Vec2 v = ps[j] - ps[hull[m]];
            float32 c = b2Cross(r, v);
            if (c < 0.0f)
            {
                ie = j;
            }

            // Collinearity check
            if (c == 0.0f && LengthSquared(v) > LengthSquared(r))
            {
                ie = j;
            }
        }

        ++m;
        ih = ie;

        if (ie == i0)
        {
            break;
        }
    }

    if (m < 3)
    {
        // Polygon is degenerate.
        b2Assert(false);
        SetAsBox(1.0f, 1.0f);
        return;
    }
    
    m_vertices.clear();
    m_normals.clear();

    // Copy vertices.
    for (int32_t i = 0; i < m; ++i)
    {
        m_vertices.push_back(ps[hull[i]]);
    }

    // Compute normals. Ensure the edges have non-zero length.
    for (int32_t i = 0; i < m; ++i)
    {
        int32_t i1 = i;
        int32_t i2 = i + 1 < m ? i + 1 : 0;
        b2Vec2 edge = m_vertices[i2] - m_vertices[i1];
        b2Assert(LengthSquared(edge) > EPSILON * EPSILON);
        m_normals.push_back(b2Cross(edge, 1.0f));
        Normalize(m_normals[i]);
    }

    // Compute the polygon centroid.
    m_centroid = ComputeCentroid(m_vertices);
}

bool b2PolygonShape::TestPoint(const b2Transform& xf, const b2Vec2& p) const
{
    b2Vec2 pLocal = b2MulT(xf.q, p - xf.p);

    for (int32_t i = 0; i < m_normals.size(); ++i)
    {
        float32 dot = b2Dot(m_normals[i], pLocal - m_vertices[i]);
        if (dot > 0.0f)
        {
            return false;
        }
    }

    return true;
}

bool b2PolygonShape::RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
                             const b2Transform& xf, int32_t childIndex) const
{
    B2_NOT_USED(childIndex);

    // Put the ray into the polygon's frame of reference.
    b2Vec2 p1 = b2MulT(xf.q, input.p1 - xf.p);
    b2Vec2 p2 = b2MulT(xf.q, input.p2 - xf.p);
    b2Vec2 d = p2 - p1;

    float32 lower = 0.0f, upper = input.maxFraction;

    int32_t index = -1;

    for (int32_t i = 0; i < m_normals.size(); ++i)
    {
        // p = p1 + a * d
        // dot(normal, p - v) = 0
        // dot(normal, p1 - v) + a * dot(normal, d) = 0
        float32 numerator = b2Dot(m_normals[i], m_vertices[i] - p1);
        float32 denominator = b2Dot(m_normals[i], d);

        if (denominator == 0.0f)
        {
            if (numerator < 0.0f)
            {
                return false;
            }
        }
        else
        {
            // Note: we want this predicate without division:
            // lower < numerator / denominator, where denominator < 0
            // Since denominator < 0, we have to flip the inequality:
            // lower < numerator / denominator <==> denominator * lower > numerator.
            if (denominator < 0.0f && numerator < lower * denominator)
            {
                // Increase lower.
                // The segment enters this half-space.
                lower = numerator / denominator;
                index = i;
            }
            else if (denominator > 0.0f && numerator < upper * denominator)
            {
                // Decrease upper.
                // The segment exits this half-space.
                upper = numerator / denominator;
            }
        }

        // The use of epsilon here causes the assert on lower to trip
        // in some cases. Apparently the use of epsilon was to make edge
        // shapes work, but now those are handled separately.
        // if (upper < lower - EPSILON)
        if (upper < lower)
        {
            return false;
        }
    }

    b2Assert(0.0f <= lower && lower <= input.maxFraction);

    if (index >= 0)
    {
        output->fraction = lower;
        output->normal = b2Mul(xf.q, m_normals[index]);
        return true;
    }

    return false;
}

void b2PolygonShape::ComputeAABB(b2AABB* aabb, const b2Transform& xf, int32_t childIndex) const
{
    B2_NOT_USED(childIndex);

    b2Vec2 lower = b2Mul(xf, m_vertices[0]);
    b2Vec2 upper = lower;

    for (int32_t i = 1; i < m_vertices.size(); ++i)
    {
        b2Vec2 v = b2Mul(xf, m_vertices[i]);
        lower = b2Min(lower, v);
        upper = b2Max(upper, v);
    }

    b2Vec2 r{{GetRadius(), GetRadius()}};
    aabb->lowerBound = lower - r;
    aabb->upperBound = upper + r;
}

void b2PolygonShape::ComputeMass(b2MassData* massData, float32 density) const
{
    // Polygon mass, centroid, and inertia.
    // Let rho be the polygon density in mass per unit area.
    // Then:
    // mass = rho * int(dA)
    // centroid.x = (1/mass) * rho * int(x * dA)
    // centroid[b2VecY] = (1/mass) * rho * int(y * dA)
    // I = rho * int((x*x + y*y) * dA)
    //
    // We can compute these integrals by summing all the integrals
    // for each triangle of the polygon. To evaluate the integral
    // for a single triangle, we make a change of variables to
    // the (u,v) coordinates of the triangle:
    // x = x0 + e1x * u + e2x * v
    // y = y0 + e1y * u + e2y * v
    // where 0 <= u && 0 <= v && u + v <= 1.
    //
    // We integrate u from [0,1-v] and then v from [0,1].
    // We also need to use the Jacobian of the transformation:
    // D = cross(e1, e2)
    //
    // Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
    //
    // The rest of the derivation is handled by computer algebra.

    b2Assert(m_vertices.size() >= 3);

    b2Vec2 center;
    center = {{0.0f, 0.0f}};
    float32 area = 0.0f;
    float32 I = 0.0f;

    // s is the reference point for forming triangles.
    // It's location doesn't change the result (except for rounding error).
    b2Vec2 s{{0.0f, 0.0f}};

    // This code would put the reference point inside the polygon.
    for (int32_t i = 0; i < m_vertices.size(); ++i)
    {
        s += m_vertices[i];
    }
    s *= 1.0f / m_vertices.size();

    const float32 k_inv3 = 1.0f / 3.0f;

    for (int32_t i = 0; i < m_vertices.size(); ++i)
    {
        // Triangle vertices.
        b2Vec2 e1 = m_vertices[i] - s;
        b2Vec2 e2 = i + 1 < m_vertices.size() ? m_vertices[i + 1] - s : m_vertices[0] - s;

        float32 D = b2Cross(e1, e2);

        float32 triangleArea = 0.5f * D;
        area += triangleArea;

        // Area weighted centroid
        center += triangleArea * k_inv3 * (e1 + e2);

        float32 ex1 = e1[b2VecX], ey1 = e1[b2VecY];
        float32 ex2 = e2[b2VecX], ey2 = e2[b2VecY];

        float32 intx2 = ex1 * ex1 + ex2 * ex1 + ex2 * ex2;
        float32 inty2 = ey1 * ey1 + ey2 * ey1 + ey2 * ey2;

        I += (0.25f * k_inv3 * D) * (intx2 + inty2);
    }

    // Total mass
    massData->mass = density * area;

    // Center of mass
    b2Assert(area > EPSILON);
    center *= 1.0f / area;
    massData->center = center + s;

    // Inertia tensor relative to the local origin (point s).
    massData->I = density * I;

    // Shift to center of mass then to original body origin.
    massData->I +=
        massData->mass * (b2Dot(massData->center, massData->center) - b2Dot(center, center));
}

bool b2PolygonShape::Validate() const
{
    for (int32_t i = 0; i < m_vertices.size(); ++i)
    {
        int32_t i1 = i;
        int32_t i2 = i < m_vertices.size() - 1 ? i1 + 1 : 0;
        b2Vec2 p = m_vertices[i1];
        b2Vec2 e = m_vertices[i2] - p;

        for (int32_t j = 0; j < m_vertices.size(); ++j)
        {
            if (j == i1 || j == i2)
            {
                continue;
            }

            b2Vec2 v = m_vertices[j] - p;
            float32 c = b2Cross(e, v);
            if (c < 0.0f)
            {
                return false;
            }
        }
    }

    return true;
}
