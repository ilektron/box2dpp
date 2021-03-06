/*
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

#include <Box2D/Dynamics/Joints/b2PrismaticJoint.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2TimeStep.h>

using namespace box2d;

// Linear constraint (point-to-line)
// d = p2 - p1 = x2 + r2 - x1 - r1
// C = dot(perp, d)
// Cdot = dot(d, cross(w1, perp)) + dot(perp, v2 + cross(w2, r2) - v1 - cross(w1, r1))
//      = -dot(perp, v1) - dot(cross(d + r1, perp), w1) + dot(perp, v2) + dot(cross(r2, perp), v2)
// J = [-perp, -cross(d + r1, perp), perp, cross(r2,perp)]
//
// Angular constraint
// C = a2 - a1 + a_initial
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
//
// K = J * invM * JT
//
// J = [-a -s1 a s2]
//     [0  -1  0  1]
// a = perp
// s1 = cross(d + r1, a) = cross(p2 - x1, a)
// s2 = cross(r2, a) = cross(p2 - x2, a)

// Motor/Limit linear constraint
// C = dot(ax1, d)
// Cdot = = -dot(ax1, v1) - dot(cross(d + r1, ax1), w1) + dot(ax1, v2) + dot(cross(r2, ax1), v2)
// J = [-ax1 -cross(d+r1,ax1) ax1 cross(r2,ax1)]

// Block Solver
// We develop a block solver that includes the joint limit. This makes the limit stiff (inelastic)
// even
// when the mass has poor distribution (leading to large torques about the joint anchor points).
//
// The Jacobian has 3 rows:
// J = [-uT -s1 uT s2] // linear
//     [0   -1   0  1] // angular
//     [-vT -a1 vT a2] // limit
//
// u = perp
// v = axis
// s1 = cross(d + r1, u), s2 = cross(r2, u)
// a1 = cross(d + r1, v), a2 = cross(r2, v)

// M * (v2 - v1) = JT * df
// J * v2 = bias
//
// v2 = v1 + invM * JT * df
// J * (v1 + invM * JT * df) = bias
// K * df = bias - J * v1 = -Cdot
// K = J * invM * JT
// Cdot = J * v1 - bias
//
// Now solve for f2.
// df = f2 - f1
// K * (f2 - f1) = -Cdot
// f2 = invK * (-Cdot) + f1
//
// Clamp accumulated limit impulse.
// lower: f2(3) = max(f2(3), 0)
// upper: f2(3) = min(f2(3), 0)
//
// Solve for correct f2(1:2)
// K(1:2, 1:2) * f2(1:2) = -Cdot(1:2) - K(1:2,3) * f2(3) + K(1:2,1:3) * f1
//                       = -Cdot(1:2) - K(1:2,3) * f2(3) + K(1:2,1:2) * f1(1:2) + K(1:2,3) * f1(3)
// K(1:2, 1:2) * f2(1:2) = -Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3)) + K(1:2,1:2) * f1(1:2)
// f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
//
// Now compute impulse to be applied:
// df = f2 - f1

void b2PrismaticJointDef::Initialize(b2Body* bA, b2Body* bB, const b2Vec<float, 2>& anchor,
                                     const b2Vec<float, 2>& axis)
{
    bodyA = bA;
    bodyB = bB;
    localAnchorA = bodyA->GetLocalPoint(anchor);
    localAnchorB = bodyB->GetLocalPoint(anchor);
    localAxisA = bodyA->GetLocalVector(axis);
    referenceAngle = bodyB->GetAngle() - bodyA->GetAngle();
}

b2PrismaticJoint::b2PrismaticJoint(const b2PrismaticJointDef* def) : b2Joint(def)
{
    m_localAnchorA = def->localAnchorA;
    m_localAnchorB = def->localAnchorB;
    m_localXAxisA = def->localAxisA;
    m_localXAxisA.Normalize();
    m_localYAxisA = b2Cross(1.0f, m_localXAxisA);
    m_referenceAngle = def->referenceAngle;

    m_impulse = {0.0f, 0.0f, 0.0f};
    m_motorMass = 0.0f;
    m_motorImpulse = 0.0f;

    m_lowerTranslation = def->lowerTranslation;
    m_upperTranslation = def->upperTranslation;
    m_maxMotorForce = def->maxMotorForce;
    m_motorSpeed = def->motorSpeed;
    m_enableLimit = def->enableLimit;
    m_enableMotor = def->enableMotor;
    m_limitState = b2LimitState::INACTIVE_LIMIT;

    m_axis = {{0.0f, 0.0f}};
    m_perp = {{0.0f, 0.0f}};
}

void b2PrismaticJoint::InitVelocityConstraints(const b2SolverData& data)
{
    m_indexA = m_bodyA->m_islandIndex;
    m_indexB = m_bodyB->m_islandIndex;
    m_localCenterA = m_bodyA->m_sweep.localCenter;
    m_localCenterB = m_bodyB->m_sweep.localCenter;
    m_invMassA = m_bodyA->m_invMass;
    m_invMassB = m_bodyB->m_invMass;
    m_invIA = m_bodyA->m_invI;
    m_invIB = m_bodyB->m_invI;

    b2Vec<float, 2> cA = data.positions[m_indexA].c;
    float aA = data.positions[m_indexA].a;
    b2Vec<float, 2> vA = data.velocities[m_indexA].v;
    float wA = data.velocities[m_indexA].w;

    b2Vec<float, 2> cB = data.positions[m_indexB].c;
    float aB = data.positions[m_indexB].a;
    b2Vec<float, 2> vB = data.velocities[m_indexB].v;
    float wB = data.velocities[m_indexB].w;

    b2Rot qA(aA), qB(aB);

    // Compute the effective masses.
    b2Vec<float, 2> rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
    b2Vec<float, 2> rB = b2Mul(qB, m_localAnchorB - m_localCenterB);
    b2Vec<float, 2> d = (cB - cA) + rB - rA;

    float mA = m_invMassA, mB = m_invMassB;
    float iA = m_invIA, iB = m_invIB;

    // Compute motor Jacobian and effective mass.
    {
        m_axis = b2Mul(qA, m_localXAxisA);
        m_a1 = b2Cross(d + rA, m_axis);
        m_a2 = b2Cross(rB, m_axis);

        m_motorMass = mA + mB + iA * m_a1 * m_a1 + iB * m_a2 * m_a2;
        if (m_motorMass > 0.0f)
        {
            m_motorMass = 1.0f / m_motorMass;
        }
    }

    // Prismatic constraint.
    {
        m_perp = b2Mul(qA, m_localYAxisA);

        m_s1 = b2Cross(d + rA, m_perp);
        m_s2 = b2Cross(rB, m_perp);

        float k11 = mA + mB + iA * m_s1 * m_s1 + iB * m_s2 * m_s2;
        float k12 = iA * m_s1 + iB * m_s2;
        float k13 = iA * m_s1 * m_a1 + iB * m_s2 * m_a2;
        float k22 = iA + iB;
        if (k22 == 0.0f)
        {
            // For bodies with fixed rotation.
            k22 = 1.0f;
        }
        float k23 = iA * m_a1 + iB * m_a2;
        float k33 = mA + mB + iA * m_a1 * m_a1 + iB * m_a2 * m_a2;

        m_K.ex = {k11, k12, k13};
        m_K.ey = {k12, k22, k23};
        m_K.ez = {k13, k23, k33};
    }

    // Compute motor and limit terms.
    if (m_enableLimit)
    {
        float jointTranslation = b2Dot(m_axis, d);
        if (std::abs(m_upperTranslation - m_lowerTranslation) < 2.0f * LINEAR_SLOP)
        {
            m_limitState = b2LimitState::EQUAL_LIMITS;
        }
        else if (jointTranslation <= m_lowerTranslation)
        {
            if (m_limitState != b2LimitState::AT_LOWER_LIMIT)
            {
                m_limitState = b2LimitState::AT_LOWER_LIMIT;
                m_impulse[b2VecZ] = 0.0f;
            }
        }
        else if (jointTranslation >= m_upperTranslation)
        {
            if (m_limitState != b2LimitState::AT_UPPER_LIMIT)
            {
                m_limitState = b2LimitState::AT_UPPER_LIMIT;
                m_impulse[b2VecZ] = 0.0f;
            }
        }
        else
        {
            m_limitState = b2LimitState::INACTIVE_LIMIT;
            m_impulse[b2VecZ] = 0.0f;
        }
    }
    else
    {
        m_limitState = b2LimitState::INACTIVE_LIMIT;
        m_impulse[b2VecZ] = 0.0f;
    }

    if (m_enableMotor == false)
    {
        m_motorImpulse = 0.0f;
    }

    if (data.step.warmStarting)
    {
        // Account for variable time step.
        m_impulse *= data.step.dtRatio;
        m_motorImpulse *= data.step.dtRatio;

        b2Vec<float, 2> P = m_impulse[b2VecX] * m_perp + (m_motorImpulse + m_impulse[b2VecZ]) * m_axis;
        float LA = m_impulse[b2VecX] * m_s1 + m_impulse[b2VecY] + (m_motorImpulse + m_impulse[b2VecZ]) * m_a1;
        float LB = m_impulse[b2VecX] * m_s2 + m_impulse[b2VecY] + (m_motorImpulse + m_impulse[b2VecZ]) * m_a2;

        vA -= mA * P;
        wA -= iA * LA;

        vB += mB * P;
        wB += iB * LB;
    }
    else
    {
        m_impulse = {0.0f, 0.0f, 0.0f};
        m_motorImpulse = 0.0f;
    }

    data.velocities[m_indexA].v = vA;
    data.velocities[m_indexA].w = wA;
    data.velocities[m_indexB].v = vB;
    data.velocities[m_indexB].w = wB;
}

void b2PrismaticJoint::SolveVelocityConstraints(const b2SolverData& data)
{
    b2Vec<float, 2> vA = data.velocities[m_indexA].v;
    float wA = data.velocities[m_indexA].w;
    b2Vec<float, 2> vB = data.velocities[m_indexB].v;
    float wB = data.velocities[m_indexB].w;

    float mA = m_invMassA, mB = m_invMassB;
    float iA = m_invIA, iB = m_invIB;

    // Solve linear motor constraint.
    if (m_enableMotor && m_limitState != b2LimitState::EQUAL_LIMITS)
    {
        float Cdot = b2Dot(m_axis, vB - vA) + m_a2 * wB - m_a1 * wA;
        float impulse = m_motorMass * (m_motorSpeed - Cdot);
        float oldImpulse = m_motorImpulse;
        float maxImpulse = data.step.dt * m_maxMotorForce;
        m_motorImpulse = b2Clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse);
        impulse = m_motorImpulse - oldImpulse;

        b2Vec<float, 2> P = impulse * m_axis;
        float LA = impulse * m_a1;
        float LB = impulse * m_a2;

        vA -= mA * P;
        wA -= iA * LA;

        vB += mB * P;
        wB += iB * LB;
    }

    b2Vec<float, 2> Cdot1;
    Cdot1[b2VecX] = b2Dot(m_perp, vB - vA) + m_s2 * wB - m_s1 * wA;
    Cdot1[b2VecY] = wB - wA;

    if (m_enableLimit && m_limitState != b2LimitState::INACTIVE_LIMIT)
    {
        // Solve prismatic and limit constraint in block form.
        float Cdot2;
        Cdot2 = b2Dot(m_axis, vB - vA) + m_a2 * wB - m_a1 * wA;
        b2Vec<float, 3> Cdot(Cdot1[b2VecX], Cdot1[b2VecY], Cdot2);

        b2Vec<float, 3> f1 = m_impulse;
        b2Vec<float, 3> df = m_K.Solve33(-Cdot);
        m_impulse += df;

        if (m_limitState == b2LimitState::AT_LOWER_LIMIT)
        {
            m_impulse[b2VecZ] = b2Max(m_impulse[b2VecZ], 0.0f);
        }
        else if (m_limitState == b2LimitState::AT_UPPER_LIMIT)
        {
            m_impulse[b2VecZ] = b2Min(m_impulse[b2VecZ], 0.0f);
        }

        // f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
        b2Vec<float, 2> b = -Cdot1 - (m_impulse[b2VecZ] - f1[b2VecZ]) * b2Vec<float, 2>{{m_K.ez[b2VecX], m_K.ez[b2VecY]}};
        b2Vec<float, 2> f2r = m_K.Solve22(b) + b2Vec<float, 2>{{f1[b2VecX], f1[b2VecY]}};
        m_impulse[b2VecX] = f2r[b2VecX];
        m_impulse[b2VecY] = f2r[b2VecY];

        df = m_impulse - f1;

        b2Vec<float, 2> P = df[b2VecX] * m_perp + df[b2VecZ] * m_axis;
        float LA = df[b2VecX] * m_s1 + df[b2VecY] + df[b2VecZ] * m_a1;
        float LB = df[b2VecX] * m_s2 + df[b2VecY] + df[b2VecZ] * m_a2;

        vA -= mA * P;
        wA -= iA * LA;

        vB += mB * P;
        wB += iB * LB;
    }
    else
    {
        // Limit is inactive, just solve the prismatic constraint in block form.
        b2Vec<float, 2> df = m_K.Solve22(-Cdot1);
        m_impulse[b2VecX] += df[b2VecX];
        m_impulse[b2VecY] += df[b2VecY];

        b2Vec<float, 2> P = df[b2VecX] * m_perp;
        float LA = df[b2VecX] * m_s1 + df[b2VecY];
        float LB = df[b2VecX] * m_s2 + df[b2VecY];

        vA -= mA * P;
        wA -= iA * LA;

        vB += mB * P;
        wB += iB * LB;
    }

    data.velocities[m_indexA].v = vA;
    data.velocities[m_indexA].w = wA;
    data.velocities[m_indexB].v = vB;
    data.velocities[m_indexB].w = wB;
}

bool b2PrismaticJoint::SolvePositionConstraints(const b2SolverData& data)
{
    b2Vec<float, 2> cA = data.positions[m_indexA].c;
    float aA = data.positions[m_indexA].a;
    b2Vec<float, 2> cB = data.positions[m_indexB].c;
    float aB = data.positions[m_indexB].a;

    b2Rot qA(aA), qB(aB);

    float mA = m_invMassA, mB = m_invMassB;
    float iA = m_invIA, iB = m_invIB;

    // Compute fresh Jacobians
    b2Vec<float, 2> rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
    b2Vec<float, 2> rB = b2Mul(qB, m_localAnchorB - m_localCenterB);
    b2Vec<float, 2> d = cB + rB - cA - rA;

    b2Vec<float, 2> axis = b2Mul(qA, m_localXAxisA);
    float a1 = b2Cross(d + rA, axis);
    float a2 = b2Cross(rB, axis);
    b2Vec<float, 2> perp = b2Mul(qA, m_localYAxisA);

    float s1 = b2Cross(d + rA, perp);
    float s2 = b2Cross(rB, perp);

    b2Vec<float, 3> impulse;
    b2Vec<float, 2> C1;
    C1[b2VecX] = b2Dot(perp, d);
    C1[b2VecY] = aB - aA - m_referenceAngle;

    float linearError = std::abs(C1[b2VecX]);
    float angularError = std::abs(C1[b2VecY]);

    bool active = false;
    float C2 = 0.0f;
    if (m_enableLimit)
    {
        float translation = b2Dot(axis, d);
        if (std::abs(m_upperTranslation - m_lowerTranslation) < 2.0f * LINEAR_SLOP)
        {
            // Prevent large angular corrections
            C2 = b2Clamp(translation, -MAX_LINEAR_CORRECTION, MAX_LINEAR_CORRECTION);
            linearError = b2Max(linearError, std::abs(translation));
            active = true;
        }
        else if (translation <= m_lowerTranslation)
        {
            // Prevent large linear corrections and allow some slop.
            C2 = b2Clamp(translation - m_lowerTranslation + LINEAR_SLOP, -MAX_LINEAR_CORRECTION,
                         0.0f);
            linearError = b2Max(linearError, m_lowerTranslation - translation);
            active = true;
        }
        else if (translation >= m_upperTranslation)
        {
            // Prevent large linear corrections and allow some slop.
            C2 = b2Clamp(translation - m_upperTranslation - LINEAR_SLOP, 0.0f,
                         MAX_LINEAR_CORRECTION);
            linearError = b2Max(linearError, translation - m_upperTranslation);
            active = true;
        }
    }

    if (active)
    {
        float k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
        float k12 = iA * s1 + iB * s2;
        float k13 = iA * s1 * a1 + iB * s2 * a2;
        float k22 = iA + iB;
        if (k22 == 0.0f)
        {
            // For fixed rotation
            k22 = 1.0f;
        }
        float k23 = iA * a1 + iB * a2;
        float k33 = mA + mB + iA * a1 * a1 + iB * a2 * a2;

        b2Mat33 K;
        K.ex = {k11, k12, k13};
        K.ey = {k12, k22, k23};
        K.ez = {k13, k23, k33};

        b2Vec<float, 3> C;
        C[b2VecX] = C1[b2VecX];
        C[b2VecY] = C1[b2VecY];
        C[b2VecZ] = C2;

        impulse = K.Solve33(-C);
    }
    else
    {
        float k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
        float k12 = iA * s1 + iB * s2;
        float k22 = iA + iB;
        if (k22 == 0.0f)
        {
            k22 = 1.0f;
        }

        b2Mat22 K;
        K.ex = {{k11, k12}};
        K.ey = {{k12, k22}};

        b2Vec<float, 2> impulse1 = K.Solve(-C1);
        impulse[b2VecX] = impulse1[b2VecX];
        impulse[b2VecY] = impulse1[b2VecY];
        impulse[b2VecZ] = 0.0f;
    }

    b2Vec<float, 2> P = impulse[b2VecX] * perp + impulse[b2VecZ] * axis;
    float LA = impulse[b2VecX] * s1 + impulse[b2VecY] + impulse[b2VecZ] * a1;
    float LB = impulse[b2VecX] * s2 + impulse[b2VecY] + impulse[b2VecZ] * a2;

    cA -= mA * P;
    aA -= iA * LA;
    cB += mB * P;
    aB += iB * LB;

    data.positions[m_indexA].c = cA;
    data.positions[m_indexA].a = aA;
    data.positions[m_indexB].c = cB;
    data.positions[m_indexB].a = aB;

    return linearError <= LINEAR_SLOP && angularError <= ANGULAR_SLOP;
}

b2Vec<float, 2> b2PrismaticJoint::GetAnchorA() const
{
    return m_bodyA->GetWorldPoint(m_localAnchorA);
}

b2Vec<float, 2> b2PrismaticJoint::GetAnchorB() const
{
    return m_bodyB->GetWorldPoint(m_localAnchorB);
}

b2Vec<float, 2> b2PrismaticJoint::GetReactionForce(float inv_dt) const
{
    return inv_dt * (m_impulse[b2VecX] * m_perp + (m_motorImpulse + m_impulse[b2VecZ]) * m_axis);
}

float b2PrismaticJoint::GetReactionTorque(float inv_dt) const
{
    return inv_dt * m_impulse[b2VecY];
}

float b2PrismaticJoint::GetJointTranslation() const
{
    b2Vec<float, 2> pA = m_bodyA->GetWorldPoint(m_localAnchorA);
    b2Vec<float, 2> pB = m_bodyB->GetWorldPoint(m_localAnchorB);
    b2Vec<float, 2> d = pB - pA;
    b2Vec<float, 2> axis = m_bodyA->GetWorldVector(m_localXAxisA);

    float translation = b2Dot(d, axis);
    return translation;
}

float b2PrismaticJoint::GetJointSpeed() const
{
    b2Body* bA = m_bodyA;
    b2Body* bB = m_bodyB;

    b2Vec<float, 2> rA = b2Mul(bA->m_xf.q, m_localAnchorA - bA->m_sweep.localCenter);
    b2Vec<float, 2> rB = b2Mul(bB->m_xf.q, m_localAnchorB - bB->m_sweep.localCenter);
    b2Vec<float, 2> p1 = bA->m_sweep.c + rA;
    b2Vec<float, 2> p2 = bB->m_sweep.c + rB;
    b2Vec<float, 2> d = p2 - p1;
    b2Vec<float, 2> axis = b2Mul(bA->m_xf.q, m_localXAxisA);

    b2Vec<float, 2> vA = bA->m_linearVelocity;
    b2Vec<float, 2> vB = bB->m_linearVelocity;
    float wA = bA->m_angularVelocity;
    float wB = bB->m_angularVelocity;

    float speed =
        b2Dot(d, b2Cross(wA, axis)) + b2Dot(axis, vB + b2Cross(wB, rB) - vA - b2Cross(wA, rA));
    return speed;
}

bool b2PrismaticJoint::IsLimitEnabled() const
{
    return m_enableLimit;
}

void b2PrismaticJoint::EnableLimit(bool flag)
{
    if (flag != m_enableLimit)
    {
        m_bodyA->SetAwake(true);
        m_bodyB->SetAwake(true);
        m_enableLimit = flag;
        m_impulse[b2VecZ] = 0.0f;
    }
}

float b2PrismaticJoint::GetLowerLimit() const
{
    return m_lowerTranslation;
}

float b2PrismaticJoint::GetUpperLimit() const
{
    return m_upperTranslation;
}

void b2PrismaticJoint::SetLimits(float lower, float upper)
{
    b2Assert(lower <= upper);
    if (lower != m_lowerTranslation || upper != m_upperTranslation)
    {
        m_bodyA->SetAwake(true);
        m_bodyB->SetAwake(true);
        m_lowerTranslation = lower;
        m_upperTranslation = upper;
        m_impulse[b2VecZ] = 0.0f;
    }
}

bool b2PrismaticJoint::IsMotorEnabled() const
{
    return m_enableMotor;
}

void b2PrismaticJoint::EnableMotor(bool flag)
{
    m_bodyA->SetAwake(true);
    m_bodyB->SetAwake(true);
    m_enableMotor = flag;
}

void b2PrismaticJoint::SetMotorSpeed(float speed)
{
    m_bodyA->SetAwake(true);
    m_bodyB->SetAwake(true);
    m_motorSpeed = speed;
}

void b2PrismaticJoint::SetMaxMotorForce(float force)
{
    m_bodyA->SetAwake(true);
    m_bodyB->SetAwake(true);
    m_maxMotorForce = force;
}

float b2PrismaticJoint::GetMotorForce(float inv_dt) const
{
    return inv_dt * m_motorImpulse;
}

void b2PrismaticJoint::Dump()
{
    int32_t indexA = m_bodyA->m_islandIndex;
    int32_t indexB = m_bodyB->m_islandIndex;

    b2Log("  b2PrismaticJointDef jd;\n");
    b2Log("  jd.bodyA = bodies[%d];\n", indexA);
    b2Log("  jd.bodyB = bodies[%d];\n", indexB);
    b2Log("  jd.collideConnected = bool(%d);\n", m_collideConnected);
    b2Log("  jd.localAnchorA.Set(%.15lef, %.15lef);\n", m_localAnchorA[b2VecX], m_localAnchorA[b2VecY]);
    b2Log("  jd.localAnchorB.Set(%.15lef, %.15lef);\n", m_localAnchorB[b2VecX], m_localAnchorB[b2VecY]);
    b2Log("  jd.localAxisA.Set(%.15lef, %.15lef);\n", m_localXAxisA[b2VecX], m_localXAxisA[b2VecY]);
    b2Log("  jd.referenceAngle = %.15lef;\n", m_referenceAngle);
    b2Log("  jd.enableLimit = bool(%d);\n", m_enableLimit);
    b2Log("  jd.lowerTranslation = %.15lef;\n", m_lowerTranslation);
    b2Log("  jd.upperTranslation = %.15lef;\n", m_upperTranslation);
    b2Log("  jd.enableMotor = bool(%d);\n", m_enableMotor);
    b2Log("  jd.motorSpeed = %.15lef;\n", m_motorSpeed);
    b2Log("  jd.maxMotorForce = %.15lef;\n", m_maxMotorForce);
    b2Log("  joints[%d] = m_world->CreateJoint(&jd);\n", m_index);
}
