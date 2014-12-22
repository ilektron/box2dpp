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

#ifndef POLY_SHAPES_H
#define POLY_SHAPES_H

/// This tests stacking. It also shows how to use b2World::Query
/// and b2TestOverlap.

/// This callback is called by b2World::QueryAABB. We find all the fixtures
/// that overlap an AABB. Of those, we use b2TestOverlap to determine which fixtures
/// overlap a circle. Up to 4 overlapped fixtures will be highlighted with a yellow border.

constexpr int POLY_SHAPES_CALLBACK_MAX_COUNT = 4;

class PolyShapesCallback : public b2QueryCallback
{
public:
    PolyShapesCallback()
    {
        m_count = 0;
    }

    void DrawFixture(b2Fixture* fixture)
    {
        b2Color color(0.95f, 0.95f, 0.6f);
        const b2Transform& xf = fixture->GetBody()->GetTransform();

        switch (fixture->GetType())
        {
            case b2Shape::e_circle:
            {
                b2CircleShape* circle = (b2CircleShape*)fixture->GetShape();

                b2Vec2 center = b2Mul(xf, circle->m_p);
                float32 radius = circle->GetRadius();

                g_debugDraw->DrawCircle(center, radius, color);
            }
            break;

            case b2Shape::e_polygon:
            {
                b2PolygonShape* poly = (b2PolygonShape*)fixture->GetShape();
                std::vector<b2Vec2> vertices;

                for (const auto& vec : poly->GetVertices())
                {
                    vertices.push_back(b2Mul(xf, vec));
                }

                g_debugDraw->DrawPolygon(vertices, color);
            }
            break;

            default:
                break;
        }
    }

    /// Called for each fixture found in the query AABB.
    /// @return false to terminate the query.
    bool ReportFixture(b2Fixture* fixture) override
    {
        if (m_count == POLY_SHAPES_CALLBACK_MAX_COUNT)
        {
            return false;
        }

        b2Body* body = fixture->GetBody();
        b2Shape* shape = fixture->GetShape();

        bool overlap = b2TestOverlap(shape, 0, &m_circle, 0, body->GetTransform(), m_transform);

        if (overlap)
        {
            DrawFixture(fixture);
            ++m_count;
        }

        return true;
    }

    b2CircleShape m_circle;
    b2Transform m_transform;
    b2Draw* g_debugDraw;
    int32_t m_count;
};

constexpr int POLY_SHAPES_MAX_BODIES = 256;

class PolyShapes : public Test
{
public:
    PolyShapes()
    {
        // Ground body
        {
            b2BodyDef bd;
            b2Body* ground = m_world->CreateBody(&bd);

            b2EdgeShape shape;
            shape.Set({{-40.0f, 0.0f}}, {{40.0f, 0.0f}});
            ground->CreateFixture(&shape, 0.0f);
        }

        {
            b2Vec2 vertices[3];
            vertices[0] = {{-0.5f, 0.0f}};
            vertices[1] = {{0.5f, 0.0f}};
            vertices[2] = {{0.0f, 1.5f}};
            m_polygons[0].Set(vertices, 3);
        }

        {
            b2Vec2 vertices[3];
            vertices[0] = {{-0.1f, 0.0f}};
            vertices[1] = {{0.1f, 0.0f}};
            vertices[2] = {{0.0f, 1.5f}};
            m_polygons[1].Set(vertices, 3);
        }

        {
            float32 w = 1.0f;
            float32 b = w / (2.0f + b2Sqrt(2.0f));
            float32 s = b2Sqrt(2.0f) * b;

            b2Vec2 vertices[8];
            vertices[0] = {{0.5f * s, 0.0f}};
            vertices[1] = {{0.5f * w, b}};
            vertices[2] = {{0.5f * w, b + s}};
            vertices[3] = {{0.5f * s, w}};
            vertices[4] = {{-0.5f * s, w}};
            vertices[5] = {{-0.5f * w, b + s}};
            vertices[6] = {{-0.5f * w, b}};
            vertices[7] = {{-0.5f * s, 0.0f}};

            m_polygons[2].Set(vertices, 8);
        }

        {
            m_polygons[3].SetAsBox(0.5f, 0.5f);
        }

        {
            m_circle.SetRadius( 0.5f);
        }

        m_bodyIndex = 0;
        memset(m_bodies, 0, sizeof(m_bodies));
    }

    void Create(int32_t index)
    {
        if (m_bodies[m_bodyIndex] != nullptr)
        {
            m_world->DestroyBody(m_bodies[m_bodyIndex]);
            m_bodies[m_bodyIndex] = nullptr;
        }

        b2BodyDef bd;
        bd.type = b2BodyType::DYNAMIC_BODY;

        float32 x = RandomFloat(-2.0f, 2.0f);
        bd.position = {{x, 10.0f}};
        bd.angle = RandomFloat(-B2_PI, B2_PI);

        if (index == 4)
        {
            bd.angularDamping = 0.02f;
        }

        m_bodies[m_bodyIndex] = m_world->CreateBody(&bd);

        if (index < 4)
        {
            b2FixtureDef fd;
            fd.shape = m_polygons + index;
            fd.density = 1.0f;
            fd.friction = 0.3f;
            m_bodies[m_bodyIndex]->CreateFixture(&fd);
        }
        else
        {
            b2FixtureDef fd;
            fd.shape = &m_circle;
            fd.density = 1.0f;
            fd.friction = 0.3f;

            m_bodies[m_bodyIndex]->CreateFixture(&fd);
        }

        m_bodyIndex = (m_bodyIndex + 1) % POLY_SHAPES_MAX_BODIES;
    }

    void DestroyBody()
    {
        for (auto& elem : m_bodies)
        {
            if (elem != nullptr)
            {
                m_world->DestroyBody(elem);
                elem = nullptr;
                return;
            }
        }
    }

    void Keyboard(int key) override
    {
        switch (key)
        {
            case '1':
            case '2':
            case '3':
            case '4':
            case '5':
                Create(key - '1');
                break;

            case 'a':
                for (int32_t i = 0; i < POLY_SHAPES_MAX_BODIES; i += 2)
                {
                    if (m_bodies[i])
                    {
                        bool active = m_bodies[i]->IsActive();
                        m_bodies[i]->SetActive(!active);
                    }
                }
                break;

            case 'd':
                DestroyBody();
                break;
        }
    }

    void Step(Settings* settings) override
    {
        Test::Step(settings);

        PolyShapesCallback callback;
        callback.m_circle.SetRadius( 2.0f);
        callback.m_circle.m_p = {{0.0f, 1.1f}};
        callback.m_transform.SetIdentity();
        callback.g_debugDraw = &g_debugDraw;

        b2AABB aabb;
        callback.m_circle.ComputeAABB(&aabb, callback.m_transform, 0);

        m_world->QueryAABB(&callback, aabb);

        b2Color color(0.4f, 0.7f, 0.8f);
        g_debugDraw.DrawCircle(callback.m_circle.m_p, callback.m_circle.GetRadius(), color);

        g_debugDraw.DrawString({{5.0f, static_cast<float>(m_textLine)}}, "Press 1-5 to drop stuff");
        m_textLine += DRAW_STRING_NEW_LINE;
        g_debugDraw.DrawString({{5.0f, static_cast<float>(m_textLine)}}, "Press 'a' to (de)activate some bodies");
        m_textLine += DRAW_STRING_NEW_LINE;
        g_debugDraw.DrawString({{5.0f, static_cast<float>(m_textLine)}}, "Press 'd' to destroy a body");
        m_textLine += DRAW_STRING_NEW_LINE;
    }

    static Test* Create()
    {
        return new PolyShapes;
    }

    int32_t m_bodyIndex;
    b2Body* m_bodies[POLY_SHAPES_MAX_BODIES];
    b2PolygonShape m_polygons[4];
    b2CircleShape m_circle;
};

#endif
