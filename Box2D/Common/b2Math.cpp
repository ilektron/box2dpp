/*
* Copyright (c) 2007-2009 Erin Catto http://www.box2d.org
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

#include <Box2D/Common/b2Math.h>

using namespace box2d;

const b2Vec<float, 2> box2d::b2Vec2_zero{{0.0f, 0.0f}};

/// Solve A * x = b, where b is a column vector. This is more efficient
/// than computing the inverse in one-shot cases.
b2Vec<float, 3> b2Mat33::Solve33(const b2Vec<float, 3>& b) const
{
    float det = b2Dot(ex, b2Cross(ey, ez));
    if (det != 0.0f)
    {
        det = 1.0f / det;
    }
    b2Vec<float, 3> x;
    x[b2VecX] = det * b2Dot(b, b2Cross(ey, ez));
    x[b2VecY] = det * b2Dot(ex, b2Cross(b, ez));
    x[b2VecZ] = det * b2Dot(ex, b2Cross(ey, b));
    return x;
}

/// Solve A * x = b, where b is a column vector. This is more efficient
/// than computing the inverse in one-shot cases.
b2Vec<float, 2> b2Mat33::Solve22(const b2Vec<float, 2>& b) const
{
    float a11 = ex[b2VecX], a12 = ey[b2VecX], a21 = ex[b2VecY], a22 = ey[b2VecY];
    float det = a11 * a22 - a12 * a21;
    if (det != 0.0f)
    {
        det = 1.0f / det;
    }
    b2Vec<float, 2> x;
    x[b2VecX] = det * (a22 * b[b2VecX] - a12 * b[b2VecY]);
    x[b2VecY] = det * (a11 * b[b2VecY] - a21 * b[b2VecX]);
    return x;
}

///
void b2Mat33::GetInverse22(b2Mat33* M) const
{
    float a = ex[b2VecX], b = ey[b2VecX], c = ex[b2VecY], d = ey[b2VecY];
    float det = a * d - b * c;
    if (det != 0.0f)
    {
        det = 1.0f / det;
    }

    M->ex[b2VecX] = det * d;
    M->ey[b2VecX] = -det * b;
    M->ex[b2VecZ] = 0.0f;
    M->ex[b2VecY] = -det * c;
    M->ey[b2VecY] = det * a;
    M->ey[b2VecZ] = 0.0f;
    M->ez[b2VecX] = 0.0f;
    M->ez[b2VecY] = 0.0f;
    M->ez[b2VecZ] = 0.0f;
}

/// Returns the zero matrix if singular.
void b2Mat33::GetSymInverse33(b2Mat33* M) const
{
    float det = b2Dot(ex, b2Cross(ey, ez));
    if (det != 0.0f)
    {
        det = 1.0f / det;
    }

    float a11 = ex[b2VecX], a12 = ey[b2VecX], a13 = ez[b2VecX];
    float a22 = ey[b2VecY], a23 = ez[b2VecY];
    float a33 = ez[b2VecZ];

    M->ex[b2VecX] = det * (a22 * a33 - a23 * a23);
    M->ex[b2VecY] = det * (a13 * a23 - a12 * a33);
    M->ex[b2VecZ] = det * (a12 * a23 - a13 * a22);

    M->ey[b2VecX] = M->ex[b2VecY];
    M->ey[b2VecY] = det * (a11 * a33 - a13 * a13);
    M->ey[b2VecZ] = det * (a13 * a12 - a11 * a23);

    M->ez[b2VecX] = M->ex[b2VecZ];
    M->ez[b2VecY] = M->ey[b2VecZ];
    M->ez[b2VecZ] = det * (a11 * a22 - a12 * a12);
}
