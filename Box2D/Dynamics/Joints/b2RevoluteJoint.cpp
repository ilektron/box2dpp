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

#include <Box2D/Dynamics/Joints/b2RevoluteJoint.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2TimeStep.h>

using namespace box2d;

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Motor constraint
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

void b2RevoluteJointDef::Initialize(b2Body* bA, b2Body* bB, const b2Vec<float, 2>& anchor)
{
    bodyA = bA;
    bodyB = bB;
    localAnchorA = bodyA->GetLocalPoint(anchor);
    localAnchorB = bodyB->GetLocalPoint(anchor);
    referenceAngle = bodyB->GetAngle() - bodyA->GetAngle();
}

b2RevoluteJoint::b2RevoluteJoint(const b2RevoluteJointDef* def) : b2Joint(def)
{
    m_localAnchorA = def->localAnchorA;
    m_localAnchorB = def->localAnchorB;
    m_referenceAngle = def->referenceAngle;

    m_impulse = {0.0f, 0.0f, 0.0f};
    m_motorImpulse = 0.0f;

    m_lowerAngle = def->lowerAngle;
    m_upperAngle = def->upperAngle;
    m_maxMotorTorque = def->maxMotorTorque;
    m_motorSpeed = def->motorSpeed;
    m_enableLimit = def->enableLimit;
    m_enableMotor = def->enableMotor;
    m_limitState = b2LimitState::INACTIVE_LIMIT;
}

void b2RevoluteJoint::InitVelocityConstraints(const b2SolverData& data)
{
    m_indexA = m_bodyA->m_islandIndex;
    m_indexB = m_bodyB->m_islandIndex;
    m_localCenterA = m_bodyA->m_sweep.localCenter;
    m_localCenterB = m_bodyB->m_sweep.localCenter;
    m_invMassA = m_bodyA->m_invMass;
    m_invMassB = m_bodyB->m_invMass;
    m_invIA = m_bodyA->m_invI;
    m_invIB = m_bodyB->m_invI;

    float aA = data.positions[m_indexA].a;
    b2Vec<float, 2> vA = data.velocities[m_indexA].v;
    float wA = data.velocities[m_indexA].w;

    float aB = data.positions[m_indexB].a;
    b2Vec<float, 2> vB = data.velocities[m_indexB].v;
    float wB = data.velocities[m_indexB].w;

    b2Rot qA(aA), qB(aB);

    m_rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
    m_rB = b2Mul(qB, m_localAnchorB - m_localCenterB);

    // J = [-I -r1_skew I r2_skew]
    //     [ 0       -1 0       1]
    // r_skew = [-ry; rx]

    // Matlab
    // K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
    //     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
    //     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

    float mA = m_invMassA, mB = m_invMassB;
    float iA = m_invIA, iB = m_invIB;

    bool fixedRotation = (iA + iB == 0.0f);

    m_mass.ex[b2VecX] = mA + mB + m_rA[b2VecY] * m_rA[b2VecY] * iA + m_rB[b2VecY] * m_rB[b2VecY] * iB;
    m_mass.ey[b2VecX] = -m_rA[b2VecY] * m_rA[b2VecX] * iA - m_rB[b2VecY] * m_rB[b2VecX] * iB;
    m_mass.ez[b2VecX] = -m_rA[b2VecY] * iA - m_rB[b2VecY] * iB;
    m_mass.ex[b2VecY] = m_mass.ey[b2VecX];
    m_mass.ey[b2VecY] = mA + mB + m_rA[b2VecX] * m_rA[b2VecX] * iA + m_rB[b2VecX] * m_rB[b2VecX] * iB;
    m_mass.ez[b2VecY] = m_rA[b2VecX] * iA + m_rB[b2VecX] * iB;
    m_mass.ex[b2VecZ] = m_mass.ez[b2VecX];
    m_mass.ey[b2VecZ] = m_mass.ez[b2VecY];
    m_mass.ez[b2VecZ] = iA + iB;

    m_motorMass = iA + iB;
    if (m_motorMass > 0.0f)
    {
        m_motorMass = 1.0f / m_motorMass;
    }

    if (m_enableMotor == false || fixedRotation)
    {
        m_motorImpulse = 0.0f;
    }

    if (m_enableLimit && fixedRotation == false)
    {
        float jointAngle = aB - aA - m_referenceAngle;
        if (std::abs(m_upperAngle - m_lowerAngle) < 2.0f * ANGULAR_SLOP)
        {
            m_limitState = b2LimitState::EQUAL_LIMITS;
        }
        else if (jointAngle <= m_lowerAngle)
        {
            if (m_limitState != b2LimitState::AT_LOWER_LIMIT)
            {
                m_impulse[b2VecZ] = 0.0f;
            }
            m_limitState = b2LimitState::AT_LOWER_LIMIT;
        }
        else if (jointAngle >= m_upperAngle)
        {
            if (m_limitState != b2LimitState::AT_UPPER_LIMIT)
            {
                m_impulse[b2VecZ] = 0.0f;
            }
            m_limitState = b2LimitState::AT_UPPER_LIMIT;
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
    }

    if (data.step.warmStarting)
    {
        // Scale impulses to support a variable time step.
        m_impulse *= data.step.dtRatio;
        m_motorImpulse *= data.step.dtRatio;

        b2Vec<float, 2> P = {{m_impulse[b2VecX], m_impulse[b2VecY]}};

        vA -= mA * P;
        wA -= iA * (b2Cross(m_rA, P) + m_motorImpulse + m_impulse[b2VecZ]);

        vB += mB * P;
        wB += iB * (b2Cross(m_rB, P) + m_motorImpulse + m_impulse[b2VecZ]);
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

void b2RevoluteJoint::SolveVelocityConstraints(const b2SolverData& data)
{
    b2Vec<float, 2> vA = data.velocities[m_indexA].v;
    float wA = data.velocities[m_indexA].w;
    b2Vec<float, 2> vB = data.velocities[m_indexB].v;
    float wB = data.velocities[m_indexB].w;

    float mA = m_invMassA, mB = m_invMassB;
    float iA = m_invIA, iB = m_invIB;

    bool fixedRotation = (iA + iB == 0.0f);

    // Solve motor constraint.
    if (m_enableMotor && m_limitState != b2LimitState::EQUAL_LIMITS && fixedRotation == false)
    {
        float Cdot = wB - wA - m_motorSpeed;
        float impulse = -m_motorMass * Cdot;
        float oldImpulse = m_motorImpulse;
        float maxImpulse = data.step.dt * m_maxMotorTorque;
        m_motorImpulse = b2Clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse);
        impulse = m_motorImpulse - oldImpulse;

        wA -= iA * impulse;
        wB += iB * impulse;
    }

    // Solve limit constraint.
    if (m_enableLimit && m_limitState != b2LimitState::INACTIVE_LIMIT && fixedRotation == false)
    {
        b2Vec<float, 2> Cdot1 = vB + b2Cross(wB, m_rB) - vA - b2Cross(wA, m_rA);
        float Cdot2 = wB - wA;
        b2Vec<float, 3> Cdot(Cdot1[b2VecX], Cdot1[b2VecY], Cdot2);

        b2Vec<float, 3> impulse = -m_mass.Solve33(Cdot);

        if (m_limitState == b2LimitState::EQUAL_LIMITS)
        {
            m_impulse += impulse;
        }
        else if (m_limitState == b2LimitState::AT_LOWER_LIMIT)
        {
            float newImpulse = m_impulse[b2VecZ] + impulse[b2VecZ];
            if (newImpulse < 0.0f)
            {
                b2Vec<float, 2> rhs = -Cdot1 + m_impulse[b2VecZ] * b2Vec<float, 2>{{m_mass.ez[b2VecX], m_mass.ez[b2VecY]}};
                b2Vec<float, 2> reduced = m_mass.Solve22(rhs);
                impulse[b2VecX] = reduced[b2VecX];
                impulse[b2VecY] = reduced[b2VecY];
                impulse[b2VecZ] = -m_impulse[b2VecZ];
                m_impulse[b2VecX] += reduced[b2VecX];
                m_impulse[b2VecY] += reduced[b2VecY];
                m_impulse[b2VecZ] = 0.0f;
            }
            else
            {
                m_impulse += impulse;
            }
        }
        else if (m_limitState == b2LimitState::AT_UPPER_LIMIT)
        {
            float newImpulse = m_impulse[b2VecZ] + impulse[b2VecZ];
            if (newImpulse > 0.0f)
            {
                b2Vec<float, 2> rhs = -Cdot1 + m_impulse[b2VecZ] * b2Vec<float, 2>{{m_mass.ez[b2VecX], m_mass.ez[b2VecY]}};
                b2Vec<float, 2> reduced = m_mass.Solve22(rhs);
                impulse[b2VecX] = reduced[b2VecX];
                impulse[b2VecY] = reduced[b2VecY];
                impulse[b2VecZ] = -m_impulse[b2VecZ];
                m_impulse[b2VecX] += reduced[b2VecX];
                m_impulse[b2VecY] += reduced[b2VecY];
                m_impulse[b2VecZ] = 0.0f;
            }
            else
            {
                m_impulse += impulse;
            }
        }

        b2Vec<float, 2> P{{impulse[b2VecX], impulse[b2VecY]}};

        vA -= mA * P;
        wA -= iA * (b2Cross(m_rA, P) + impulse[b2VecZ]);

        vB += mB * P;
        wB += iB * (b2Cross(m_rB, P) + impulse[b2VecZ]);
    }
    else
    {
        // Solve point-to-point constraint
        b2Vec<float, 2> Cdot = vB + b2Cross(wB, m_rB) - vA - b2Cross(wA, m_rA);
        b2Vec<float, 2> impulse = m_mass.Solve22(-Cdot);

        m_impulse[b2VecX] += impulse[b2VecX];
        m_impulse[b2VecY] += impulse[b2VecY];

        vA -= mA * impulse;
        wA -= iA * b2Cross(m_rA, impulse);

        vB += mB * impulse;
        wB += iB * b2Cross(m_rB, impulse);
    }

    data.velocities[m_indexA].v = vA;
    data.velocities[m_indexA].w = wA;
    data.velocities[m_indexB].v = vB;
    data.velocities[m_indexB].w = wB;
}

bool b2RevoluteJoint::SolvePositionConstraints(const b2SolverData& data)
{
    b2Vec<float, 2> cA = data.positions[m_indexA].c;
    float aA = data.positions[m_indexA].a;
    b2Vec<float, 2> cB = data.positions[m_indexB].c;
    float aB = data.positions[m_indexB].a;

    b2Rot qA(aA), qB(aB);

    float angularError = 0.0f;
    float positionError = 0.0f;

    bool fixedRotation = (m_invIA + m_invIB == 0.0f);

    // Solve angular limit constraint.
    if (m_enableLimit && m_limitState != b2LimitState::INACTIVE_LIMIT && fixedRotation == false)
    {
        float angle = aB - aA - m_referenceAngle;
        float limitImpulse = 0.0f;

        if (m_limitState == b2LimitState::EQUAL_LIMITS)
        {
            // Prevent large angular corrections
            float C =
                b2Clamp(angle - m_lowerAngle, -MAX_ANGULAR_CORRECTION, MAX_ANGULAR_CORRECTION);
            limitImpulse = -m_motorMass * C;
            angularError = std::abs(C);
        }
        else if (m_limitState == b2LimitState::AT_LOWER_LIMIT)
        {
            float C = angle - m_lowerAngle;
            angularError = -C;

            // Prevent large angular corrections and allow some slop.
            C = b2Clamp(C + ANGULAR_SLOP, -MAX_ANGULAR_CORRECTION, 0.0f);
            limitImpulse = -m_motorMass * C;
        }
        else if (m_limitState == b2LimitState::AT_UPPER_LIMIT)
        {
            float C = angle - m_upperAngle;
            angularError = C;

            // Prevent large angular corrections and allow some slop.
            C = b2Clamp(C - ANGULAR_SLOP, 0.0f, MAX_ANGULAR_CORRECTION);
            limitImpulse = -m_motorMass * C;
        }

        aA -= m_invIA * limitImpulse;
        aB += m_invIB * limitImpulse;
    }

    // Solve point-to-point constraint.
    {
        qA.Set(aA);
        qB.Set(aB);
        b2Vec<float, 2> rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
        b2Vec<float, 2> rB = b2Mul(qB, m_localAnchorB - m_localCenterB);

        b2Vec<float, 2> C = cB + rB - cA - rA;
        positionError = C.Length();

        float mA = m_invMassA, mB = m_invMassB;
        float iA = m_invIA, iB = m_invIB;

        b2Mat22 K;
        K.ex[b2VecX] = mA + mB + iA * rA[b2VecY] * rA[b2VecY] + iB * rB[b2VecY] * rB[b2VecY];
        K.ex[b2VecY] = -iA * rA[b2VecX] * rA[b2VecY] - iB * rB[b2VecX] * rB[b2VecY];
        K.ey[b2VecX] = K.ex[b2VecY];
        K.ey[b2VecY] = mA + mB + iA * rA[b2VecX] * rA[b2VecX] + iB * rB[b2VecX] * rB[b2VecX];

        b2Vec<float, 2> impulse = -K.Solve(C);

        cA -= mA * impulse;
        aA -= iA * b2Cross(rA, impulse);

        cB += mB * impulse;
        aB += iB * b2Cross(rB, impulse);
    }

    data.positions[m_indexA].c = cA;
    data.positions[m_indexA].a = aA;
    data.positions[m_indexB].c = cB;
    data.positions[m_indexB].a = aB;

    return positionError <= LINEAR_SLOP && angularError <= ANGULAR_SLOP;
}

b2Vec<float, 2> b2RevoluteJoint::GetAnchorA() const
{
    return m_bodyA->GetWorldPoint(m_localAnchorA);
}

b2Vec<float, 2> b2RevoluteJoint::GetAnchorB() const
{
    return m_bodyB->GetWorldPoint(m_localAnchorB);
}

b2Vec<float, 2> b2RevoluteJoint::GetReactionForce(float inv_dt) const
{
    b2Vec<float, 2> P{{m_impulse[b2VecX], m_impulse[b2VecY]}};
    return inv_dt * P;
}

float b2RevoluteJoint::GetReactionTorque(float inv_dt) const
{
    return inv_dt * m_impulse[b2VecZ];
}

float b2RevoluteJoint::GetJointAngle() const
{
    b2Body* bA = m_bodyA;
    b2Body* bB = m_bodyB;
    return bB->m_sweep.a - bA->m_sweep.a - m_referenceAngle;
}

float b2RevoluteJoint::GetJointSpeed() const
{
    b2Body* bA = m_bodyA;
    b2Body* bB = m_bodyB;
    return bB->m_angularVelocity - bA->m_angularVelocity;
}

bool b2RevoluteJoint::IsMotorEnabled() const
{
    return m_enableMotor;
}

void b2RevoluteJoint::EnableMotor(bool flag)
{
    m_bodyA->SetAwake(true);
    m_bodyB->SetAwake(true);
    m_enableMotor = flag;
}

float b2RevoluteJoint::GetMotorTorque(float inv_dt) const
{
    return inv_dt * m_motorImpulse;
}

void b2RevoluteJoint::SetMotorSpeed(float speed)
{
    m_bodyA->SetAwake(true);
    m_bodyB->SetAwake(true);
    m_motorSpeed = speed;
}

void b2RevoluteJoint::SetMaxMotorTorque(float torque)
{
    m_bodyA->SetAwake(true);
    m_bodyB->SetAwake(true);
    m_maxMotorTorque = torque;
}

bool b2RevoluteJoint::IsLimitEnabled() const
{
    return m_enableLimit;
}

void b2RevoluteJoint::EnableLimit(bool flag)
{
    if (flag != m_enableLimit)
    {
        m_bodyA->SetAwake(true);
        m_bodyB->SetAwake(true);
        m_enableLimit = flag;
        m_impulse[b2VecZ] = 0.0f;
    }
}

float b2RevoluteJoint::GetLowerLimit() const
{
    return m_lowerAngle;
}

float b2RevoluteJoint::GetUpperLimit() const
{
    return m_upperAngle;
}

void b2RevoluteJoint::SetLimits(float lower, float upper)
{
    b2Assert(lower <= upper);

    if (lower != m_lowerAngle || upper != m_upperAngle)
    {
        m_bodyA->SetAwake(true);
        m_bodyB->SetAwake(true);
        m_impulse[b2VecZ] = 0.0f;
        m_lowerAngle = lower;
        m_upperAngle = upper;
    }
}

void b2RevoluteJoint::Dump()
{
    int32_t indexA = m_bodyA->m_islandIndex;
    int32_t indexB = m_bodyB->m_islandIndex;

    b2Log("  b2RevoluteJointDef jd;\n");
    b2Log("  jd.bodyA = bodies[%d];\n", indexA);
    b2Log("  jd.bodyB = bodies[%d];\n", indexB);
    b2Log("  jd.collideConnected = bool(%d);\n", m_collideConnected);
    b2Log("  jd.localAnchorA.Set(%.15lef, %.15lef);\n", m_localAnchorA[b2VecX], m_localAnchorA[b2VecY]);
    b2Log("  jd.localAnchorB.Set(%.15lef, %.15lef);\n", m_localAnchorB[b2VecX], m_localAnchorB[b2VecY]);
    b2Log("  jd.referenceAngle = %.15lef;\n", m_referenceAngle);
    b2Log("  jd.enableLimit = bool(%d);\n", m_enableLimit);
    b2Log("  jd.lowerAngle = %.15lef;\n", m_lowerAngle);
    b2Log("  jd.upperAngle = %.15lef;\n", m_upperAngle);
    b2Log("  jd.enableMotor = bool(%d);\n", m_enableMotor);
    b2Log("  jd.motorSpeed = %.15lef;\n", m_motorSpeed);
    b2Log("  jd.maxMotorTorque = %.15lef;\n", m_maxMotorTorque);
    b2Log("  joints[%d] = m_world->CreateJoint(&jd);\n", m_index);
}
