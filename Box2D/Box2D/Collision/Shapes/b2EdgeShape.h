/*
* Copyright (c) 2006-2010 Erin Catto http://www.box2d.org
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

#ifndef B2_EDGE_SHAPE_H
#define B2_EDGE_SHAPE_H

#include <Box2D/Collision/Shapes/b2Shape.h>

namespace b2d11
{

/// A line segment (edge) shape. These can be connected in chains or loops
/// to other edge shapes. The connectivity information is used to ensure
/// correct contact normals.
class EdgeShape : public Shape
{
public:
	EdgeShape();

	/// Set this as an isolated edge.
	void Set(const Vec2& v1, const Vec2& v2);

	/// Implement Shape.
	Shape* Clone(BlockAllocator* allocator) const;

	/// @see Shape::GetChildCount
	int32 GetChildCount() const;

	/// @see Shape::TestPoint
	bool TestPoint(const Transform& transform, const Vec2& p) const;

	/// Implement Shape.
	bool RayCast(RayCastOutput* output, const RayCastInput& input,
				const Transform& transform, int32 childIndex) const;

	/// @see Shape::ComputeAABB
	void ComputeAABB(AABB* aabb, const Transform& transform, int32 childIndex) const;

	/// @see Shape::ComputeMass
	void ComputeMass(MassData* massData, float32 density) const;
	
	/// These are the edge vertices
	Vec2 m_vertex1, m_vertex2;

	/// Optional adjacent vertices. These are used for smooth collision.
	Vec2 m_vertex0, m_vertex3;
	bool m_hasVertex0, m_hasVertex3;
};

inline EdgeShape::EdgeShape()
{
	m_type = e_edge;
	m_radius = POLYGON_RADIUS;
	m_vertex0.x = 0.0f;
	m_vertex0.y = 0.0f;
	m_vertex3.x = 0.0f;
	m_vertex3.y = 0.0f;
	m_hasVertex0 = false;
	m_hasVertex3 = false;
}

} // End of namespace b2d11

#endif
