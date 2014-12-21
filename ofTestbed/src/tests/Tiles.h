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

#ifndef TILES_H
#define TILES_H

/// This stress tests the dynamic tree broad-phase. This also shows that tile
/// based collision is _not_ smooth due to Box2D not knowing about adjacency.

constexpr int TILES_COUNT = 20;

class Tiles : public Test
{
public:
    Tiles()
    {
        m_fixtureCount = 0;
        b2Timer timer;

        {
            float32 a = 0.5f;
            b2BodyDef bd;
            bd.position[b2VecY] = -a;
            b2Body* ground = m_world->CreateBody(&bd);

#if 1
            int32_t N = 200;
            int32_t M = 10;
            b2Vec2 position;
            position[b2VecY] = 0.0f;
            for (int32_t j = 0; j < M; ++j)
            {
                position[b2VecX] = -N * a;
                for (int32_t i = 0; i < N; ++i)
                {
                    b2PolygonShape shape;
                    shape.SetAsBox(a, a, position, 0.0f);
                    ground->CreateFixture(&shape, 0.0f);
                    ++m_fixtureCount;
                    position[b2VecX] += 2.0f * a;
                }
                position[b2VecY] -= 2.0f * a;
            }
#else
            int32_t N = 200;
            int32_t M = 10;
            b2Vec2 position;
            position[b2VecX] = -N * a;
            for (int32_t i = 0; i < N; ++i)
            {
            position[b2VecY] = 0.0f;
                for (int32_t j = 0; j < M; ++j)
                {
                    b2PolygonShape shape;
                    shape.SetAsBox(a, a, position, 0.0f);
                    ground->CreateFixture(&shape, 0.0f);
                    position[b2VecY] -= 2.0f * a;
                }
                position[b2VecX] += 2.0f * a;
            }
#endif
        }

        {
            float32 a = 0.5f;
            b2PolygonShape shape;
            shape.SetAsBox(a, a);

            b2Vec2 x{{-7.0f, 0.75f}};
            b2Vec2 y;
            b2Vec2 deltaX{{0.5625f, 1.25f}};
            b2Vec2 deltaY{{1.125f, 0.0f}};

            for (int32_t i = 0; i < TILES_COUNT; ++i)
            {
                y = x;

                for (int32_t j = i; j < TILES_COUNT; ++j)
                {
                    b2BodyDef bd;
                    bd.type = b2BodyType::DYNAMIC_BODY;
                    bd.position = y;

                    // if (i == 0 && j == 0)
                    //{
                    //	bd.allowSleep = false;
                    //}
                    // else
                    //{
                    //	bd.allowSleep = true;
                    //}

                    b2Body* body = m_world->CreateBody(&bd);
                    body->CreateFixture(&shape, 5.0f);
                    ++m_fixtureCount;
                    y += deltaY;
                }

                x += deltaX;
            }
        }

        m_createTime = timer.GetMilliseconds();
    }

    void Step(Settings* settings) override
    {
        const b2ContactManager& cm = m_world->GetContactManager();
        int32_t height = cm.m_broadPhase.GetTreeHeight();
        int32_t leafCount = cm.m_broadPhase.GetProxyCount();
        int32_t minimumNodeCount = 2 * leafCount - 1;
        float32 minimumHeight = ceilf(logf(float32(minimumNodeCount)) / logf(2.0f));
        g_debugDraw.DrawString({{5.0f, static_cast<float>(m_textLine)}}, "dynamic tree height = %d, min = %d", height,
                               int32_t(minimumHeight));
        m_textLine += DRAW_STRING_NEW_LINE;

        Test::Step(settings);

        g_debugDraw.DrawString({{5.0f, static_cast<float>(m_textLine)}}, "create time = %6.2f ms, fixture count = %d",
                               m_createTime, m_fixtureCount);
        m_textLine += DRAW_STRING_NEW_LINE;

        // b2DynamicTree* tree = &m_world->m_contactManager.m_broadPhase.m_tree;

        // if (m_stepCount == 400)
        //{
        //	tree->RebuildBottomUp();
        //}
    }

    static Test* Create()
    {
        return new Tiles;
    }

    int32_t m_fixtureCount;
    float32 m_createTime;
};

#endif
