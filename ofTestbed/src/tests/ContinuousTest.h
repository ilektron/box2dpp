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

#ifndef CONTINUOUS_TEST_H
#define CONTINUOUS_TEST_H

class ContinuousTest : public Test
{
public:
    ContinuousTest()
    {
        {
            b2BodyDef bd;
            bd.position = {{0.0f, 0.0f}};
            b2Body* body = m_world->CreateBody(&bd);

            b2EdgeShape edge;

            edge.Set({{-10.0f, 0.0f}}, {{10.0f, 0.0f}});
            body->CreateFixture(&edge, 0.0f);

            b2PolygonShape shape;
            shape.SetAsBox(0.2f, 1.0f, {{0.5f, 1.0f}}, 0.0f);
            body->CreateFixture(&shape, 0.0f);
        }

#if 1
        {
            b2BodyDef bd;
            bd.type = b2BodyType::DYNAMIC_BODY;
            bd.position = {{0.0f, 20.0f}};
            // bd.angle = 0.1f;

            b2PolygonShape shape;
            shape.SetAsBox(2.0f, 0.1f);

            m_body = m_world->CreateBody(&bd);
            m_body->CreateFixture(&shape, 1.0f);

            m_angularVelocity = RandomFloat(-50.0f, 50.0f);
            // m_angularVelocity = 46.661274f;
            m_body->SetLinearVelocity({{0.0f, -100.0f}});
            m_body->SetAngularVelocity(m_angularVelocity);
        }
#else
        {
            b2BodyDef bd;
            bd.type = b2BodyType::DYNAMIC_BODY;
            bd.position = {{0.0f, 2.0f}};
            b2Body* body = m_world->CreateBody(&bd);

            b2CircleShape shape;
            shape.m_p = {{0.0f, 0.0f}};
            shape.SetRadius( 0.5f);
            body->CreateFixture(&shape, 1.0f);

            bd.bullet = true;
            bd.position = {{0.0f, 10.0f}};
            body = m_world->CreateBody(&bd);
            body->CreateFixture(&shape, 1.0f);
            body->SetLinearVelocity({{0.0f, -100.0f}});
        }
#endif

        b2GJKState::b2_gjkCalls = 0;
        b2GJKState::b2_gjkIters = 0;
        b2GJKState::b2_gjkMaxIters = 0;
        b2TOIState::b2_toiCalls = 0;
        b2TOIState::b2_toiIters = 0;
        b2TOIState::b2_toiRootIters = 0;
        b2TOIState::b2_toiMaxRootIters = 0;
        b2TOIState::b2_toiTime = 0.0f;
        b2TOIState::b2_toiMaxTime = 0.0f;
    }

    void Launch()
    {
        
        b2GJKState::b2_gjkCalls = 0;
        b2GJKState::b2_gjkIters = 0;
        b2GJKState::b2_gjkMaxIters = 0;
        b2TOIState::b2_toiCalls = 0;
        b2TOIState::b2_toiIters = 0;
        b2TOIState::b2_toiRootIters = 0;
        b2TOIState::b2_toiMaxRootIters = 0;
        b2TOIState::b2_toiTime = 0.0f;
        b2TOIState::b2_toiMaxTime = 0.0f;

        m_body->SetTransform({{0.0f, 20.0f}}, 0.0f);
        m_angularVelocity = RandomFloat(-50.0f, 50.0f);
        m_body->SetLinearVelocity({{0.0f, -100.0f}});
        m_body->SetAngularVelocity(m_angularVelocity);
    }

    void Step(Settings* settings) override
    {
        Test::Step(settings);

        if (b2GJKState::b2_gjkCalls > 0)
        {
            g_debugDraw.DrawString({{5.0f, static_cast<float>(m_textLine)}},
                                   "gjk calls = %d, ave gjk iters = %3.1f, max gjk iters = %d",
                                   b2GJKState::b2_gjkCalls, b2GJKState::b2_gjkIters / float32(b2GJKState::b2_gjkCalls), b2GJKState::b2_gjkMaxIters);
            m_textLine += DRAW_STRING_NEW_LINE;
        }

        if (b2TOIState::b2_toiCalls > 0)
        {
            g_debugDraw.DrawString({{5.0f, static_cast<float>(m_textLine)}},
                                   "toi calls = %d, ave [max] toi iters = %3.1f [%d]", b2TOIState::b2_toiCalls,
                                   b2_toiIters / float32(b2TOIState::b2_toiCalls), b2TOIState::b2_toiMaxRootIters);
            m_textLine += DRAW_STRING_NEW_LINE;

            g_debugDraw.DrawString({{5.0f, static_cast<float>(m_textLine)}}, "ave [max] toi root iters = %3.1f [%d]",
                                   b2TOIState::b2_toiRootIters / float32(b2TOIState::b2_toiCalls), b2TOIState::b2_toiMaxRootIters);
            m_textLine += DRAW_STRING_NEW_LINE;

            g_debugDraw.DrawString({{5.0f, static_cast<float>(m_textLine)}}, "ave [max] toi time = %.1f [%.1f] (microseconds)",
                                   1000.0f * b2TOIState::b2_toiTime / float32(b2TOIState::b2_toiCalls),
                                   1000.0f * b2TOIState::b2_toiMaxTime);
            m_textLine += DRAW_STRING_NEW_LINE;
        }

        if (m_stepCount % 60 == 0)
        {
            // Launch();
        }
    }

    static Test* Create()
    {
        return new ContinuousTest;
    }

    b2Body* m_body;
    float32 m_angularVelocity;
};

#endif
