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

#ifndef BRIDGE_H
#define BRIDGE_H

constexpr int BRIDGE_COUNT = 30;

class Bridge : public Test
{
public:
    Bridge()
    {
        b2Body* ground = nullptr;
        {
            b2BodyDef bd;
            ground = m_world->CreateBody(&bd);

            b2EdgeShape shape;
            shape.Set({{40.0f, 0.0f}}, {{40.0f, 0.0f}});
            ground->CreateFixture(&shape, 0.0f);
        }

        {
            b2PolygonShape shape;
            shape.SetAsBox(0.5f, 0.125f);

            b2FixtureDef fd;
            fd.shape = &shape;
            fd.density = 20.0f;
            fd.friction = 0.2f;

            b2RevoluteJointDef jd;

            b2Body* prevBody = ground;
            for (int32_t i = 0; i < BRIDGE_COUNT; ++i)
            {
                b2BodyDef bd;
                bd.type = b2BodyType::DYNAMIC_BODY;
                bd.position = {{-14.5f + 1.0f * i, 5.0f}};
                b2Body* body = m_world->CreateBody(&bd);
                body->CreateFixture(&fd);

                b2Vec2 anchor{{-15.0f + 1.0f * i, 5.0f}};
                jd.Initialize(prevBody, body, anchor);
                m_world->CreateJoint(&jd);

                if (i == (BRIDGE_COUNT >> 1))
                {
                    m_middle = body;
                }
                prevBody = body;
            }

            b2Vec2 anchor{{-15.0f + 1.0f * BRIDGE_COUNT, 5.0f}};
            jd.Initialize(prevBody, ground, anchor);
            m_world->CreateJoint(&jd);
        }

        for (int32_t i = 0; i < 2; ++i)
        {
            b2Vec2 vertices[3];
            vertices[0] = {{-0.5f, 0.0f}};
            vertices[1] = {{0.5f, 0.0f}};
            vertices[2] = {{0.0f, 1.5f}};

            b2PolygonShape shape;
            shape.Set(vertices, 3);

            b2FixtureDef fd;
            fd.shape = &shape;
            fd.density = 1.0f;

            b2BodyDef bd;
            bd.type = b2BodyType::DYNAMIC_BODY;
            bd.position = {{-8.0f + 8.0f * i, 12.0f}};
            b2Body* body = m_world->CreateBody(&bd);
            body->CreateFixture(&fd);
        }

        for (int32_t i = 0; i < 3; ++i)
        {
            b2CircleShape shape;
            shape.SetRadius(0.5f);

            b2FixtureDef fd;
            fd.shape = &shape;
            fd.density = 1.0f;

            b2BodyDef bd;
            bd.type = b2BodyType::DYNAMIC_BODY;
            bd.position = {{-6.0f + 6.0f * i, 10.0f}};
            b2Body* body = m_world->CreateBody(&bd);
            body->CreateFixture(&fd);
        }
    }

    static Test* Create()
    {
        return new Bridge;
    }

    b2Body* m_middle;
};

#endif