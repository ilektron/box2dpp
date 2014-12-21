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

#ifndef POLYCOLLISION_H
#define POLYCOLLISION_H

class PolyCollision : public Test
{
public:
    PolyCollision()
    {
        {
            m_polygonA.SetAsBox(0.2f, 0.4f);
            m_transformA.Set({{0.0f, 0.0f}}, 0.0f);
        }

        {
            m_polygonB.SetAsBox(0.5f, 0.5f);
            m_positionB = {{19.345284f, 1.5632932f}};
            m_angleB = 1.9160721f;
            m_transformB.Set(m_positionB, m_angleB);
        }
    }

    static Test* Create()
    {
        return new PolyCollision;
    }

    void Step(Settings* settings) override
    {
        B2_NOT_USED(settings);

        b2Manifold manifold;
        b2CollidePolygons(&manifold, &m_polygonA, m_transformA, &m_polygonB, m_transformB);

        b2WorldManifold worldManifold;
        worldManifold.Initialize(&manifold, m_transformA, m_polygonA.GetRadius(), m_transformB,
                                 m_polygonB.GetRadius());

        g_debugDraw.DrawString({{5.0f, static_cast<float>(m_textLine)}}, "point count = %d", manifold.pointCount);
        m_textLine += DRAW_STRING_NEW_LINE;

        {
            b2Color color(0.9f, 0.9f, 0.9f);
            std::vector<b2Vec2> v;
            for (const auto& vec : m_polygonA.GetVertices())
            {
                v.push_back(b2Mul(m_transformA, vec));
            }
            g_debugDraw.DrawPolygon(v, color);

            v.clear();
            for (const auto& vec : m_polygonB.GetVertices())
            {
                v.push_back(b2Mul(m_transformB, vec));
            }
            g_debugDraw.DrawPolygon(v, color);
        }

        for (int32_t i = 0; i < manifold.pointCount; ++i)
        {
            g_debugDraw.DrawPoint(worldManifold.points[i], 4.0f, b2Color(0.9f, 0.3f, 0.3f));
        }
    }

    void Keyboard(int key) override
    {
        switch (key)
        {
            case 'a':
                m_positionB[b2VecX] -= 0.1f;
                break;

            case 'd':
                m_positionB[b2VecX] += 0.1f;
                break;

            case 's':
                m_positionB[b2VecY] -= 0.1f;
                break;

            case 'w':
                m_positionB[b2VecY] += 0.1f;
                break;

            case 'q':
                m_angleB += 0.1f * B2_PI;
                break;

            case 'e':
                m_angleB -= 0.1f * B2_PI;
                break;
        }

        m_transformB.Set(m_positionB, m_angleB);
    }

    b2PolygonShape m_polygonA;
    b2PolygonShape m_polygonB;

    b2Transform m_transformA;
    b2Transform m_transformB;

    b2Vec2 m_positionB;
    float32 m_angleB;
};

#endif
