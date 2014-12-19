/*
* Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
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

#include <Box2D/Dynamics/Joints/b2MouseJoint.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2TimeStep.h>

using namespace box2d;

// p = attached point, m = mouse point
// C = p - m
// Cdot = v
//      = v + cross(w, r)
// J = [I r_skew]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

b2MouseJoint::b2MouseJoint(const b2MouseJointDef* def) : b2Joint(def)
{
    b2Assert(IsValid(def->target));
    b2Assert(b2IsValid(def->maxForce) && def->maxForce >= 0.0f);
    b2Assert(b2IsValid(def->frequencyHz) && def->frequencyHz >= 0.0f);
    b2Assert(b2IsValid(def->dampingRatio) && def->dampingRatio >= 0.0f);

    m_targetA = def->target;
    m_localAnchorB = b2MulT(m_bodyB->GetTransform(), m_targetA);

    m_maxForce = def->maxForce;
    m_impulse = {{0.0f, 0.0f}};

    m_frequencyHz = def->frequencyHz;
    m_dampingRatio = def->dampingRatio;

    m_beta = 0.0f;
    m_gamma = 0.0f;
}

void b2MouseJoint::SetTarget(const b2Vec2& target)
{
    if (m_bodyB->IsAwake() == false)
    {
        m_bodyB->SetAwake(true);
    }
    m_targetA = target;
}

const b2Vec2& b2MouseJoint::GetTarget() const
{
    return m_targetA;
}

void b2MouseJoint::SetMaxForce(float32 force)
{
    m_maxForce = force;
}

float32 b2MouseJoint::GetMaxForce() const
{
    return m_maxForce;
}

void b2MouseJoint::SetFrequency(float32 hz)
{
    m_frequencyHz = hz;
}

float32 b2MouseJoint::GetFrequency() const
{
    return m_frequencyHz;
}

void b2MouseJoint::SetDampingRatio(float32 ratio)
{
    m_dampingRatio = ratio;
}

float32 b2MouseJoint::GetDampingRatio() const
{
    return m_dampingRatio;
}

void b2MouseJoint::InitVelocityConstraints(const b2SolverData& data)
{
    m_indexB = m_bodyB->m_islandIndex;
    m_localCenterB = m_bodyB->m_sweep.localCenter;
    m_invMassB = m_bodyB->m_invMass;
    m_invIB = m_bodyB->m_invI;

    b2Vec2 cB = data.positions[m_indexB].c;
    float32 aB = data.positions[m_indexB].a;
    b2Vec2 vB = data.velocities[m_indexB].v;
    float32 wB = data.velocities[m_indexB].w;

    b2Rot qB(aB);

    float32 mass = m_bodyB->GetMass();

    // Frequency
    float32 omega = 2.0f * B2_PI * m_frequencyHz;

    // Damping coefficient
    float32 d = 2.0f * mass * m_dampingRatio * omega;

    // Spring stiffness
    float32 k = mass * (omega * omega);

    // magic formulas
    // gamma has units of inverse mass.
    // beta has units of inverse time.
    float32 h = data.step.dt;
    b2Assert(d + h * k > EPSILON);
    m_gamma = h * (d + h * k);
    if (m_gamma != 0.0f)
    {
        m_gamma = 1.0f / m_gamma;
    }
    m_beta = h * k * m_gamma;

    // Compute the effective mass matrix.
    m_rB = b2Mul(qB, m_localAnchorB - m_localCenterB);

    // K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
    //      = [1/m1+1/m2     0    ] + invI1 * [r1[b2VecY]*r1[b2VecY] -r1[b2VecX]*r1[b2VecY]] + invI2 * [r1[b2VecY]*r1[b2VecY]
    //      -r1[b2VecX]*r1[b2VecY]]
    //        [    0     1/m1+1/m2]           [-r1[b2VecX]*r1[b2VecY] r1[b2VecX]*r1[b2VecX]]           [-r1[b2VecX]*r1[b2VecY]
    //        r1[b2VecX]*r1[b2VecX]]
    b2Mat22 K;
    K.ex[b2VecX] = m_invMassB + m_invIB * m_rB[b2VecY] * m_rB[b2VecY] + m_gamma;
    K.ex[b2VecY] = -m_invIB * m_rB[b2VecX] * m_rB[b2VecY];
    K.ey[b2VecX] = K.ex[b2VecY];
    K.ey[b2VecY] = m_invMassB + m_invIB * m_rB[b2VecX] * m_rB[b2VecX] + m_gamma;

    m_mass = K.GetInverse();

    m_C = cB + m_rB - m_targetA;
    m_C *= m_beta;

    // Cheat with some damping
    wB *= 0.98f;

    if (data.step.warmStarting)
    {
        m_impulse *= data.step.dtRatio;
        vB += m_invMassB * m_impulse;
        wB += m_invIB * b2Cross(m_rB, m_impulse);
    }
    else
    {
        m_impulse = {{0.0f, 0.0f}};
    }

    data.velocities[m_indexB].v = vB;
    data.velocities[m_indexB].w = wB;
}

void b2MouseJoint::SolveVelocityConstraints(const b2SolverData& data)
{
    b2Vec2 vB = data.velocities[m_indexB].v;
    float32 wB = data.velocities[m_indexB].w;

    // Cdot = v + cross(w, r)
    b2Vec2 Cdot = vB + b2Cross(wB, m_rB);
    b2Vec2 impulse = b2Mul(m_mass, -(Cdot + m_C + m_gamma * m_impulse));

    b2Vec2 oldImpulse = m_impulse;
    m_impulse += impulse;
    float32 maxImpulse = data.step.dt * m_maxForce;
    if (LengthSquared(m_impulse) > maxImpulse * maxImpulse)
    {
        m_impulse *= maxImpulse / Length(m_impulse);
    }
    impulse = m_impulse - oldImpulse;

    vB += m_invMassB * impulse;
    wB += m_invIB * b2Cross(m_rB, impulse);

    data.velocities[m_indexB].v = vB;
    data.velocities[m_indexB].w = wB;
}

bool b2MouseJoint::SolvePositionConstraints(const b2SolverData& data)
{
    B2_NOT_USED(data);
    return true;
}

b2Vec2 b2MouseJoint::GetAnchorA() const
{
    return m_targetA;
}

b2Vec2 b2MouseJoint::GetAnchorB() const
{
    return m_bodyB->GetWorldPoint(m_localAnchorB);
}

b2Vec2 b2MouseJoint::GetReactionForce(float32 inv_dt) const
{
    return inv_dt * m_impulse;
}

float32 b2MouseJoint::GetReactionTorque(float32 inv_dt) const
{
    return inv_dt * 0.0f;
}

void b2MouseJoint::ShiftOrigin(const b2Vec2& newOrigin)
{
    m_targetA -= newOrigin;
}
