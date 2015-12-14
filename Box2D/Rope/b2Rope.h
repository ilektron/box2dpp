/*
* Copyright (c) 2011 Erin Catto http://www.box2d.org
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

#ifndef B2_ROPE_H
#define B2_ROPE_H

#include <Box2D/Common/b2Math.h>

namespace box2d
{
class b2Draw;

///
struct b2RopeDef
{
    b2RopeDef()
    {
        vertices = nullptr;
        count = 0;
        masses = nullptr;
        gravity = {{0.0f, 0.0f}};
        damping = 0.1f;
        k2 = 0.9f;
        k3 = 0.1f;
    }

    ///
    b2Vec<float, 2>* vertices;

    ///
    int32_t count;

    ///
    float* masses;

    ///
    b2Vec<float, 2> gravity;

    ///
    float damping;

    /// Stretching stiffness
    float k2;

    /// Bending stiffness. Values above 0.5 can make the simulation blow up.
    float k3;
};

///
class b2Rope
{
public:
    b2Rope();
    ~b2Rope();

    ///
    void Initialize(const b2RopeDef* def);

    ///
    void Step(float timeStep, int32_t iterations);

    ///
    int32_t GetVertexCount() const
    {
        return m_count;
    }

    ///
    const b2Vec<float, 2>* GetVertices() const
    {
        return m_ps;
    }

    ///
    void Draw(b2Draw* draw) const;

    ///
    void SetAngle(float angle);

private:
    void SolveC2();
    void SolveC3();

    int32_t m_count;
    b2Vec<float, 2>* m_ps;
    b2Vec<float, 2>* m_p0s;
    b2Vec<float, 2>* m_vs;

    float* m_ims;

    float* m_Ls;
    float* m_as;

    b2Vec<float, 2> m_gravity;
    float m_damping;

    float m_k2;
    float m_k3;
};
}

#endif
