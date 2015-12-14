/*
* Copyright (c) 2011 Erin Catto http://box2d.org
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

#include <Box2D/Rope/b2Rope.h>
#include <Box2D/Common/b2Draw.h>

using namespace box2d;

b2Rope::b2Rope()
{
    m_count = 0;
    m_ps = nullptr;
    m_p0s = nullptr;
    m_vs = nullptr;
    m_ims = nullptr;
    m_Ls = nullptr;
    m_as = nullptr;
    m_gravity = {{0.0f, 0.0f}};
    m_k2 = 1.0f;
    m_k3 = 0.1f;
}

b2Rope::~b2Rope()
{
    b2Free(m_ps);
    b2Free(m_p0s);
    b2Free(m_vs);
    b2Free(m_ims);
    b2Free(m_Ls);
    b2Free(m_as);
}

void b2Rope::Initialize(const b2RopeDef* def)
{
    b2Assert(def->count >= 3);
    m_count = def->count;
    m_ps = (b2Vec<float, 2>*)b2Alloc(m_count * sizeof(b2Vec<float, 2>));
    m_p0s = (b2Vec<float, 2>*)b2Alloc(m_count * sizeof(b2Vec<float, 2>));
    m_vs = (b2Vec<float, 2>*)b2Alloc(m_count * sizeof(b2Vec<float, 2>));
    m_ims = (float*)b2Alloc(m_count * sizeof(float));

    for (int32_t i = 0; i < m_count; ++i)
    {
        m_ps[i] = def->vertices[i];
        m_p0s[i] = def->vertices[i];
        m_vs[i] = {{0.0f, 0.0f}};

        float m = def->masses[i];
        if (m > 0.0f)
        {
            m_ims[i] = 1.0f / m;
        }
        else
        {
            m_ims[i] = 0.0f;
        }
    }

    int32_t count2 = m_count - 1;
    int32_t count3 = m_count - 2;
    m_Ls = (float*)b2Alloc(count2 * sizeof(float));
    m_as = (float*)b2Alloc(count3 * sizeof(float));

    for (int32_t i = 0; i < count2; ++i)
    {
        b2Vec<float, 2> p1 = m_ps[i];
        b2Vec<float, 2> p2 = m_ps[i + 1];
        m_Ls[i] = b2Distance(p1, p2);
    }

    for (int32_t i = 0; i < count3; ++i)
    {
        b2Vec<float, 2> p1 = m_ps[i];
        b2Vec<float, 2> p2 = m_ps[i + 1];
        b2Vec<float, 2> p3 = m_ps[i + 2];

        b2Vec<float, 2> d1 = p2 - p1;
        b2Vec<float, 2> d2 = p3 - p2;

        float a = b2Cross(d1, d2);
        float b = b2Dot(d1, d2);

        m_as[i] = b2Atan2(a, b);
    }

    m_gravity = def->gravity;
    m_damping = def->damping;
    m_k2 = def->k2;
    m_k3 = def->k3;
}

void b2Rope::Step(float h, int32_t iterations)
{
    if (h == 0.0)
    {
        return;
    }

    float d = expf(-h * m_damping);

    for (int32_t i = 0; i < m_count; ++i)
    {
        m_p0s[i] = m_ps[i];
        if (m_ims[i] > 0.0f)
        {
            m_vs[i] += h * m_gravity;
        }
        m_vs[i] *= d;
        m_ps[i] += h * m_vs[i];
    }

    for (int32_t i = 0; i < iterations; ++i)
    {
        SolveC2();
        SolveC3();
        SolveC2();
    }

    float inv_h = 1.0f / h;
    for (int32_t i = 0; i < m_count; ++i)
    {
        m_vs[i] = inv_h * (m_ps[i] - m_p0s[i]);
    }
}

void b2Rope::SolveC2()
{
    int32_t count2 = m_count - 1;

    for (int32_t i = 0; i < count2; ++i)
    {
        b2Vec<float, 2> p1 = m_ps[i];
        b2Vec<float, 2> p2 = m_ps[i + 1];

        b2Vec<float, 2> d = p2 - p1;
        float L = d.Normalize();

        float im1 = m_ims[i];
        float im2 = m_ims[i + 1];

        if (im1 + im2 == 0.0f)
        {
            continue;
        }

        float s1 = im1 / (im1 + im2);
        float s2 = im2 / (im1 + im2);

        p1 -= m_k2 * s1 * (m_Ls[i] - L) * d;
        p2 += m_k2 * s2 * (m_Ls[i] - L) * d;

        m_ps[i] = p1;
        m_ps[i + 1] = p2;
    }
}

void b2Rope::SetAngle(float angle)
{
    int32_t count3 = m_count - 2;
    for (int32_t i = 0; i < count3; ++i)
    {
        m_as[i] = angle;
    }
}

void b2Rope::SolveC3()
{
    int32_t count3 = m_count - 2;

    for (int32_t i = 0; i < count3; ++i)
    {
        b2Vec<float, 2> p1 = m_ps[i];
        b2Vec<float, 2> p2 = m_ps[i + 1];
        b2Vec<float, 2> p3 = m_ps[i + 2];

        float m1 = m_ims[i];
        float m2 = m_ims[i + 1];
        float m3 = m_ims[i + 2];

        b2Vec<float, 2> d1 = p2 - p1;
        b2Vec<float, 2> d2 = p3 - p2;

        float L1sqr = d1.LengthSquared();
        float L2sqr = d2.LengthSquared();

        if (L1sqr * L2sqr == 0.0f)
        {
            continue;
        }

        float a = b2Cross(d1, d2);
        float b = b2Dot(d1, d2);

        float angle = b2Atan2(a, b);

        b2Vec<float, 2> Jd1 = (-1.0f / L1sqr) * Skew(d1);
        b2Vec<float, 2> Jd2 = (1.0f / L2sqr) * Skew(d2);

        b2Vec<float, 2> J1 = -Jd1;
        b2Vec<float, 2> J2 = Jd1 - Jd2;
        b2Vec<float, 2> J3 = Jd2;

        float mass = m1 * b2Dot(J1, J1) + m2 * b2Dot(J2, J2) + m3 * b2Dot(J3, J3);
        if (mass == 0.0f)
        {
            continue;
        }

        mass = 1.0f / mass;

        float C = angle - m_as[i];

        while (C > B2_PI)
        {
            angle -= 2 * B2_PI;
            C = angle - m_as[i];
        }

        while (C < -B2_PI)
        {
            angle += 2.0f * B2_PI;
            C = angle - m_as[i];
        }

        float impulse = -m_k3 * mass * C;

        p1 += (m1 * impulse) * J1;
        p2 += (m2 * impulse) * J2;
        p3 += (m3 * impulse) * J3;

        m_ps[i] = p1;
        m_ps[i + 1] = p2;
        m_ps[i + 2] = p3;
    }
}

void b2Rope::Draw(b2Draw* draw) const
{
    b2Color c(0.4f, 0.5f, 0.7f);

    for (int32_t i = 0; i < m_count - 1; ++i)
    {
        draw->DrawSegment(m_ps[i], m_ps[i + 1], c);
    }
}
