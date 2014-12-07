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

#ifndef B2_SETTINGS_H
#define B2_SETTINGS_H

#include <stddef.h>
#include <assert.h>
#include <cfloat>
#include <cmath>
#include <cstdint>

namespace box2d
{
#define B2_NOT_USED(x) ((void)(x))
inline void b2Assert(bool a) { assert(a); }

typedef float float32;
typedef double float64;

constexpr float MAX_FLOAT = FLT_MAX;
constexpr float EPSILON = FLT_EPSILON;
constexpr float PI = M_PI;

/// @file
/// Global tuning constants based on meters-kilograms-seconds (MKS) units.
///

// Collision

/// The maximum number of contact points between two convex shapes. Do
/// not change this value.
constexpr int MAX_MANIFOLD_POINTS = 2;

/// The maximum number of vertices on a convex polygon. You cannot increase
/// this too much because b2BlockAllocator has a maximum object size.
constexpr int MAX_POLYGON_VERTICES = 8;

/// This is used to fatten AABBs in the dynamic tree. This allows proxies
/// to move by a small amount without triggering a tree adjustment.
/// This is in meters.
constexpr float AABB_EXTENSION = 0.1f;

/// This is used to fatten AABBs in the dynamic tree. This is used to predict
/// the future position based on the current displacement.
/// This is a dimensionless multiplier.
constexpr float AABB_MULTIPLIER = 2.0f;

/// A small length used as a collision and constraint tolerance. Usually it is
/// chosen to be numerically significant, but visually insignificant.
constexpr float LINEAR_SLOP = 0.005f;

/// A small angle used as a collision and constraint tolerance. Usually it is
/// chosen to be numerically significant, but visually insignificant.
constexpr float ANGULAR_SLOP = (2.0f / 180.0f * PI);

/// The radius of the polygon/edge shape skin. This should not be modified. Making
/// this smaller means polygons will have an insufficient buffer for continuous collision.
/// Making it larger may create artifacts for vertex collision.
constexpr float POLYGON_RADIUS = (2.0f * LINEAR_SLOP);

/// Maximum number of sub-steps per contact in continuous physics simulation.
constexpr int MAX_SUB_STEPS = 8;

// Dynamics

/// Maximum number of contacts to be handled to solve a TOI impact.
constexpr int MAX_TOI_CONTACTS = 32;

/// A velocity threshold for elastic collisions. Any collision with a relative linear
/// velocity below this threshold will be treated as inelastic.
constexpr float VELOCITY_THRESHOLD = 1.0f;

/// The maximum linear position correction used when solving constraints. This helps to
/// prevent overshoot.
constexpr float MAX_LINEAR_CORRECTION = 0.2f;

/// The maximum angular position correction used when solving constraints. This helps to
/// prevent overshoot.
constexpr float MAX_ANGULAR_CORRECTION = (8.0f / 180.0f * PI);

/// The maximum linear velocity of a body. This limit is very large and is used
/// to prevent numerical problems. You shouldn't need to adjust this.
constexpr float MAX_TRANSLATION = 2.0f;
constexpr float MAX_TRANSLATION_SQUARED = (MAX_TRANSLATION * MAX_TRANSLATION);

/// The maximum angular velocity of a body. This limit is very large and is used
/// to prevent numerical problems. You shouldn't need to adjust this.
constexpr float MAX_ROTATION = (0.5f * PI);
constexpr float MAX_ROTATION_SQUARED = (MAX_ROTATION * MAX_ROTATION);

/// This scale factor controls how fast overlap is resolved. Ideally this would be 1 so
/// that overlap is removed in one time step. However using values close to 1 often lead
/// to overshoot.
constexpr float BAUMGARTE = 0.2f;
constexpr float TOI_BAUMGARTE = 0.75f;

// Sleep

/// The time that a body must be still before it will go to sleep.
constexpr float TIME_TO_SLEEP = 0.5f;

/// A body cannot sleep if its linear velocity is above this tolerance.
constexpr float LINEAR_SLEEP_TOLERANCE = 0.01f;

/// A body cannot sleep if its angular velocity is above this tolerance.
constexpr float ANGULAR_SLEEP_TOLERANCE = (2.0f / 180.0f * PI);

// Memory Allocation

/// Implement this function to use your own memory allocator.
void* b2Alloc(int32_t size);

/// If you implement b2Alloc, you should also implement this function.
void b2Free(void* mem);

/// Logging function.
void b2Log(const char* string, ...);

/// Version numbering scheme.
/// See http://en.wikipedia.org/wiki/Software_versioning
struct b2Version
{
    int32_t major;     ///< significant changes
    int32_t minor;     ///< incremental changes
    int32_t revision;  ///< bug fixes
};

/// Current version.
extern b2Version b2_version;
}

#endif
