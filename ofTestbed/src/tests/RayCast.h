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

#ifndef RAY_CAST_H
#define RAY_CAST_H

// This test demonstrates how to use the world ray-cast feature.
// NOTE: we are intentionally filtering one of the polygons, therefore
// the ray will always miss one type of polygon.

// This callback finds the closest hit. Polygon 0 is filtered.
class RayCastClosestCallback : public b2RayCastCallback
{
public:
    RayCastClosestCallback()
    {
        m_hit = false;
    }

    float32 ReportFixture(b2Fixture* fixture, const b2Vec2& point, const b2Vec2& normal,
                          float32 fraction) override
    {
        b2Body* body = fixture->GetBody();
        void* userData = body->GetUserData();
        if (userData)
        {
            int32_t index = *(int32_t*)userData;
            if (index == 0)
            {
                // By returning -1, we instruct the calling code to ignore this fixture and
                // continue the ray-cast to the next fixture.
                return -1.0f;
            }
        }

        m_hit = true;
        m_point = point;
        m_normal = normal;

        // By returning the current fraction, we instruct the calling code to clip the ray and
        // continue the ray-cast to the next fixture. WARNING: do not assume that fixtures
        // are reported in order. However, by clipping, we can always get the closest fixture.
        return fraction;
    }

    bool m_hit;
    b2Vec2 m_point;
    b2Vec2 m_normal;
};

// This callback finds any hit. Polygon 0 is filtered. For this type of query we are usually
// just checking for obstruction, so the actual fixture and hit point are irrelevant.
class RayCastAnyCallback : public b2RayCastCallback
{
public:
    RayCastAnyCallback()
    {
        m_hit = false;
    }

    float32 ReportFixture(b2Fixture* fixture, const b2Vec2& point, const b2Vec2& normal,
                          float32 fraction) override
    {
        b2Body* body = fixture->GetBody();
        void* userData = body->GetUserData();
        if (userData)
        {
            int32_t index = *(int32_t*)userData;
            if (index == 0)
            {
                // By returning -1, we instruct the calling code to ignore this fixture
                // and continue the ray-cast to the next fixture.
                return -1.0f;
            }
        }

        m_hit = true;
        m_point = point;
        m_normal = normal;

        // At this point we have a hit, so we know the ray is obstructed.
        // By returning 0, we instruct the calling code to terminate the ray-cast.
        return 0.0f;
    }

    bool m_hit;
    b2Vec2 m_point;
    b2Vec2 m_normal;
};

// This ray cast collects multiple hits along the ray. Polygon 0 is filtered.
// The fixtures are not necessary reported in order, so we might not capture
// the closest fixture.1

constexpr int RAY_CAST_MULTIPLE_CALLBACK_MAX_COUNT = 3;

class RayCastMultipleCallback : public b2RayCastCallback
{
public:
    RayCastMultipleCallback()
    {
        m_count = 0;
    }

    float32 ReportFixture(b2Fixture* fixture, const b2Vec2& point, const b2Vec2& normal,
                          float32 fraction) override
    {
        b2Body* body = fixture->GetBody();
        void* userData = body->GetUserData();
        if (userData)
        {
            int32_t index = *(int32_t*)userData;
            if (index == 0)
            {
                // By returning -1, we instruct the calling code to ignore this fixture
                // and continue the ray-cast to the next fixture.
                return -1.0f;
            }
        }

        b2Assert(m_count < RAY_CAST_MULTIPLE_CALLBACK_MAX_COUNT);

        m_points[m_count] = point;
        m_normals[m_count] = normal;
        ++m_count;

        if (m_count == RAY_CAST_MULTIPLE_CALLBACK_MAX_COUNT)
        {
            // At this point the buffer is full.
            // By returning 0, we instruct the calling code to terminate the ray-cast.
            return 0.0f;
        }

        // By returning 1, we instruct the caller to continue without clipping the ray.
        return 1.0f;
    }

    b2Vec2 m_points[RAY_CAST_MULTIPLE_CALLBACK_MAX_COUNT];
    b2Vec2 m_normals[RAY_CAST_MULTIPLE_CALLBACK_MAX_COUNT];
    int32_t m_count;
};

constexpr int RAY_CAST_COUNT = 256;

class RayCast : public Test
{
public:
    enum Mode
    {
        e_closest,
        e_any,
        e_multiple
    };

    RayCast()
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

        {
            m_edge.Set({{-1.0f, 0.0f}}, {{1.0f, 0.0f}});
        }

        m_bodyIndex = 0;
        memset(m_bodies, 0, sizeof(m_bodies));

        m_angle = 0.0f;

        m_mode = e_closest;
    }

    void Create(int32_t index)
    {
        if (m_bodies[m_bodyIndex] != nullptr)
        {
            m_world->DestroyBody(m_bodies[m_bodyIndex]);
            m_bodies[m_bodyIndex] = nullptr;
        }

        b2BodyDef bd;

        float32 x = RandomFloat(-10.0f, 10.0f);
        float32 y = RandomFloat(0.0f, 20.0f);
        bd.position = {{x, y}};
        bd.angle = RandomFloat(-B2_PI, B2_PI);

        m_userData[m_bodyIndex] = index;
        bd.userData = m_userData + m_bodyIndex;

        if (index == 4)
        {
            bd.angularDamping = 0.02f;
        }

        m_bodies[m_bodyIndex] = m_world->CreateBody(&bd);

        if (index < 4)
        {
            b2FixtureDef fd;
            fd.shape = m_polygons + index;
            fd.friction = 0.3f;
            m_bodies[m_bodyIndex]->CreateFixture(&fd);
        }
        else if (index < 5)
        {
            b2FixtureDef fd;
            fd.shape = &m_circle;
            fd.friction = 0.3f;

            m_bodies[m_bodyIndex]->CreateFixture(&fd);
        }
        else
        {
            b2FixtureDef fd;
            fd.shape = &m_edge;
            fd.friction = 0.3f;

            m_bodies[m_bodyIndex]->CreateFixture(&fd);
        }

        m_bodyIndex = (m_bodyIndex + 1) % RAY_CAST_COUNT;
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
            case '6':
                Create(key - '1');
                break;

            case 'd':
                DestroyBody();
                break;

            case 'm':
                if (m_mode == e_closest)
                {
                    m_mode = e_any;
                }
                else if (m_mode == e_any)
                {
                    m_mode = e_multiple;
                }
                else if (m_mode == e_multiple)
                {
                    m_mode = e_closest;
                }
        }
    }

    void Step(Settings* settings) override
    {
        bool advanceRay = settings->pause == 0 || settings->singleStep;

        Test::Step(settings);
        g_debugDraw.DrawString({{5.0f, static_cast<float>(m_textLine)}}, "Press 1-6 to drop stuff, m to change the mode");
        m_textLine += DRAW_STRING_NEW_LINE;
        switch (m_mode)
        {
            case e_closest:
                g_debugDraw.DrawString(
                    {{5.0f, static_cast<float>(m_textLine)}}, "Ray-cast mode: closest - find closest fixture along the ray");
                break;

            case e_any:
                g_debugDraw.DrawString({{5.0f, static_cast<float>(m_textLine)}}, "Ray-cast mode: any - check for obstruction");
                break;

            case e_multiple:
                g_debugDraw.DrawString({{5.0f, static_cast<float>(m_textLine)}},
                                       "Ray-cast mode: multiple - gather multiple fixtures");
                break;
        }

        m_textLine += DRAW_STRING_NEW_LINE;

        float32 L = 11.0f;
        b2Vec2 point1{{0.0f, 10.0f}};
        b2Vec2 d{{L * cosf(m_angle), L * sinf(m_angle)}};
        b2Vec2 point2 = point1 + d;

        if (m_mode == e_closest)
        {
            RayCastClosestCallback callback;
            m_world->RayCast(&callback, point1, point2);

            if (callback.m_hit)
            {
                g_debugDraw.DrawPoint(callback.m_point, 5.0f, b2Color(0.4f, 0.9f, 0.4f));
                g_debugDraw.DrawSegment(point1, callback.m_point, b2Color(0.8f, 0.8f, 0.8f));
                b2Vec2 head = callback.m_point + 0.5f * callback.m_normal;
                g_debugDraw.DrawSegment(callback.m_point, head, b2Color(0.9f, 0.9f, 0.4f));
            }
            else
            {
                g_debugDraw.DrawSegment(point1, point2, b2Color(0.8f, 0.8f, 0.8f));
            }
        }
        else if (m_mode == e_any)
        {
            RayCastAnyCallback callback;
            m_world->RayCast(&callback, point1, point2);

            if (callback.m_hit)
            {
                g_debugDraw.DrawPoint(callback.m_point, 5.0f, b2Color(0.4f, 0.9f, 0.4f));
                g_debugDraw.DrawSegment(point1, callback.m_point, b2Color(0.8f, 0.8f, 0.8f));
                b2Vec2 head = callback.m_point + 0.5f * callback.m_normal;
                g_debugDraw.DrawSegment(callback.m_point, head, b2Color(0.9f, 0.9f, 0.4f));
            }
            else
            {
                g_debugDraw.DrawSegment(point1, point2, b2Color(0.8f, 0.8f, 0.8f));
            }
        }
        else if (m_mode == e_multiple)
        {
            RayCastMultipleCallback callback;
            m_world->RayCast(&callback, point1, point2);
            g_debugDraw.DrawSegment(point1, point2, b2Color(0.8f, 0.8f, 0.8f));

            for (int32_t i = 0; i < callback.m_count; ++i)
            {
                b2Vec2 p = callback.m_points[i];
                b2Vec2 n = callback.m_normals[i];
                g_debugDraw.DrawPoint(p, 5.0f, b2Color(0.4f, 0.9f, 0.4f));
                g_debugDraw.DrawSegment(point1, p, b2Color(0.8f, 0.8f, 0.8f));
                b2Vec2 head = p + 0.5f * n;
                g_debugDraw.DrawSegment(p, head, b2Color(0.9f, 0.9f, 0.4f));
            }
        }

        if (advanceRay)
        {
            m_angle += 0.25f * B2_PI / 180.0f;
        }

#if 0
		// This case was failing.
		{
			b2Vec2 vertices[4];
			//vertices[0] = {{-22.875f, -3.0f}};
			//vertices[1] = {{22.875f, -3.0f}};
			//vertices[2] = {{22.875f, 3.0f}};
			//vertices[3] = {{-22.875f, 3.0f}};

			b2PolygonShape shape;
			//shape.Set(vertices, 4);
			shape.SetAsBox(22.875f, 3.0f);

			b2RayCastInput input;
			input.p1 = {{10.2725f,1.71372f}};
			input.p2 = {{10.2353f,2.21807f}};
			//input.maxFraction = 0.567623f;
			input.maxFraction = 0.56762173f;

			b2Transform xf;
			xf.SetIdentity();
			xf.position = {{23.0f, 5.0f}};

			b2RayCastOutput output;
			bool hit;
			hit = shape.RayCast(&output, input, xf);
			hit = false;

			b2Color color(1.0f, 1.0f, 1.0f);
			b2Vec2 vs[4];
			for (int32_t i = 0; i < 4; ++i)
			{
				vs[i] = b2Mul(xf, shape.m_vertices[i]);
			}

			g_debugDraw.DrawPolygon(vs, 4, color);
			g_debugDraw.DrawSegment(input.p1, input.p2, color);
		}
#endif
    }

    static Test* Create()
    {
        return new RayCast;
    }

    int32_t m_bodyIndex;
    b2Body* m_bodies[RAY_CAST_COUNT];
    int32_t m_userData[RAY_CAST_COUNT];
    b2PolygonShape m_polygons[4];
    b2CircleShape m_circle;
    b2EdgeShape m_edge;

    float32 m_angle;

    Mode m_mode;
};

#endif