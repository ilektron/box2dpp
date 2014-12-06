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

#ifndef B2_CONTACT_SOLVER_H
#define B2_CONTACT_SOLVER_H

#include <Box2D/Common/b2Math.h>
#include <Box2D/Collision/b2Collision.h>
#include <Box2D/Dynamics/b2TimeStep.h>

namespace b2d11
{

class Contact;
class Body;
class StackAllocator;

struct VelocityConstraintPoint
{
	Vec2 rA;
	Vec2 rB;
	float32 normalImpulse;
	float32 tangentImpulse;
	float32 normalMass;
	float32 tangentMass;
	float32 velocityBias;
};

struct ContactVelocityConstraint
{
	VelocityConstraintPoint points[MAX_MANIFOLD_POINTS];
	Vec2 normal;
	Mat22 normalMass;
	Mat22 K;
	int32 indexA;
	int32 indexB;
	float32 invMassA, invMassB;
	float32 invIA, invIB;
	float32 friction;
	float32 restitution;
	float32 tangentSpeed;
	int32 pointCount;
	int32 contactIndex;
};

struct ContactPositionConstraint
{
	Vec2 localPoints[MAX_MANIFOLD_POINTS];
	Vec2 localNormal;
	Vec2 localPoint;
	int32 indexA;
	int32 indexB;
	float32 invMassA, invMassB;
	Vec2 localCenterA, localCenterB;
	float32 invIA, invIB;
	Manifold::Type type;
	float32 radiusA, radiusB;
	int32 pointCount;
};

struct ContactSolverDef
{
	TimeStep step;
	Contact** contacts;
	int32 count;
	Position* positions;
	Velocity* velocities;
	StackAllocator* allocator;
};

class ContactSolver
{
public:
	ContactSolver(ContactSolverDef* def);
	~ContactSolver();

	void InitializeVelocityConstraints();

	void WarmStart();
	void SolveVelocityConstraints();
	void StoreImpulses();

	bool SolvePositionConstraints();
	bool SolveTOIPositionConstraints(int32 toiIndexA, int32 toiIndexB);
	
	ContactVelocityConstraint* m_velocityConstraints; // used outside of class
private:
	
	TimeStep m_step;
	Position* m_positions;
	Velocity* m_velocities;
	StackAllocator* m_allocator;
	ContactPositionConstraint* m_positionConstraints;
	Contact** m_contacts;
	int m_count;
};

} // End of namespace b2d11

#endif

