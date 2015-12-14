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

#ifndef B2_MATH_H
#define B2_MATH_H
#pragma once

#include <Box2D/Common/b2MathFunctions.h>
#include <Box2D/Common/b2Settings.h>
#include <Box2D/Common/b2Vec.h>
#include <utility>
#include <cmath>
#include <cstring>
#include <array>
#include <utility>
#include <initializer_list>

namespace box2d
{

/// A 2D column vector.
struct b2Vec2Old
{
    /// Default constructor does nothing (for performance).
    b2Vec2Old() = default;

    /// Construct using coordinates.
    b2Vec2Old(float x, float y) : x(x), y(y)
    {
    }

    /// Set this vector to all zeros.
    void SetZero()
    {
        x = 0.0f;
        y = 0.0f;
    }

    /// Set this vector to some specified coordinates.
    void Set(float x_, float y_)
    {
        x = x_;
        y = y_;
    }

    /// Negate this vector.
    b2Vec2Old operator-() const
    {
        b2Vec2Old v;
        v.Set(-x, -y);
        return v;
    }

    // Read from and indexed element.
    float operator[](int32_t i) const
    {
        switch (i)
        {
            case 0:
                return x;
                break;
            case 1:
                return y;
                break;
            default:
                assert(false);
                break;
        }
    }

    // Write to an indexed element.
    float& operator[](int32_t i)
    {
        switch (i)
        {
            case 0:
                return x;
                break;
            case 1:
                return y;
                break;
            default:
                assert(false);
                break;
        }
    }

    /// Add a vector to this vector.
    void operator+=(const b2Vec2Old& v)
    {
        x += v[b2VecX];
        y += v[b2VecY];
    }

    /// Subtract a vector from this vector.
    void operator-=(const b2Vec2Old& v)
    {
        x -= v[b2VecX];
        y -= v[b2VecY];
    }

    /// Multiply this vector by a scalar.
    void operator*=(float a)
    {
        x *= a;
        y *= a;
    }

    /// Get the length of this vector (the norm).
    float Length() const
    {
        return b2Sqrt(x * x + y * y);
    }

    /// Get the length squared. For performance, use this instead of
    /// b2Vec<float, 2>::Length (if possible).
    float LengthSquared() const
    {
        return x * x + y * y;
    }

    /// Convert this vector into a unit vector. Returns the length.
    float Normalize()
    {
        float length = Length();
        if (length < EPSILON)
        {
            return 0.0f;
        }
        float invLength = 1.0f / length;
        x *= invLength;
        y *= invLength;

        return length;
    }

    /// Does this vector contain finite coordinates?
    bool IsValid() const
    {
        return b2IsValid(x) && b2IsValid(y);
    }

    /// Get the skew vector such that dot(skew_vec, other) == cross(vec, other)
    b2Vec2Old Skew() const
    {
        return b2Vec2Old(-y, x);
    }

    // private:
    float x, y;
};

/// A 2-by-2 matrix. Stored in column-major order.
struct b2Mat22
{
    /// The default constructor does nothing (for performance).
    b2Mat22()
    {
    }

    /// Construct this matrix using columns.
    b2Mat22(const b2Vec<float, 2>& c1, const b2Vec<float, 2>& c2)
    {
        ex = c1;
        ey = c2;
    }

    /// Construct this matrix using scalars.
    b2Mat22(float a11, float a12, float a21, float a22)
    {
        ex = {{a11, a21}};
        ey = {{a12, a22}};
    }

    /// Initialize this matrix using columns.
    void Set(const b2Vec<float, 2>& c1, const b2Vec<float, 2>& c2)
    {
        ex = c1;
        ey = c2;
    }

    /// Set this to the identity matrix.
    void SetIdentity()
    {
        ex = {{1.0f, 0.0f}};
        ey = {{0.0f, 1.0f}};
    }

    /// Set this matrix to all zeros.
    void SetZero()
    {
        ex = {{0.0f, 0.0f}};
        ey = {{0.0f, 0.0f}};
    }

    b2Mat22 GetInverse() const
    {
        float a = ex[b2VecX], b = ey[b2VecX], c = ex[b2VecY], d = ey[b2VecY];
        b2Mat22 B;
        float det = a * d - b * c;
        if (det != 0.0f)
        {
            det = 1.0f / det;
        }
        B.ex[b2VecX] = det * d;
        B.ey[b2VecX] = -det * b;
        B.ex[b2VecY] = -det * c;
        B.ey[b2VecY] = det * a;
        return B;
    }

    /// Solve A * x = b, where b is a column vector. This is more efficient
    /// than computing the inverse in one-shot cases.
    b2Vec<float, 2> Solve(const b2Vec<float, 2>& b) const
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

    // private:
    b2Vec<float, 2> ex, ey;
};

/// A 3-by-3 matrix. Stored in column-major order.
struct b2Mat33
{
    /// The default constructor does nothing (for performance).
    b2Mat33()
    {
    }

    /// Construct this matrix using columns.
    b2Mat33(const b2Vec<float, 3>& c1, const b2Vec<float, 3>& c2, const b2Vec<float, 3>& c3)
    {
        ex = c1;
        ey = c2;
        ez = c3;
    }

    /// Set this matrix to all zeros.
    void SetZero()
    {
        ex = {};
        ey = {};
        ez = {};
    }

    /// Solve A * x = b, where b is a column vector. This is more efficient
    /// than computing the inverse in one-shot cases.
    b2Vec<float, 3> Solve33(const b2Vec<float, 3>& b) const;

    /// Solve A * x = b, where b is a column vector. This is more efficient
    /// than computing the inverse in one-shot cases. Solve only the upper
    /// 2-by-2 matrix equation.
    b2Vec<float, 2> Solve22(const b2Vec<float, 2>& b) const;

    /// Get the inverse of this matrix as a 2-by-2.
    /// Returns the zero matrix if singular.
    void GetInverse22(b2Mat33* M) const;

    /// Get the symmetric inverse of this matrix as a 3-by-3.
    /// Returns the zero matrix if singular.
    void GetSymInverse33(b2Mat33* M) const;

    b2Vec<float, 3> ex, ey, ez;
};

/// Rotation
struct b2Rot
{
    b2Rot()
    {
    }

    /// Initialize from an angle in radians
    explicit b2Rot(float angle)
    {
        /// TODO_ERIN optimize
        s = sinf(angle);
        c = cosf(angle);
    }

    /// Set using an angle in radians.
    void Set(float angle)
    {
        /// TODO_ERIN optimize
        s = sinf(angle);
        c = cosf(angle);
    }

    /// Set to the identity rotation
    void SetIdentity()
    {
        s = 0.0f;
        c = 1.0f;
    }

    /// Get the angle in radians
    float GetAngle() const
    {
        return b2Atan2(s, c);
    }

    /// Get the x-axis
    b2Vec<float, 2> GetXAxis() const
    {
        return b2Vec<float, 2>{{c, s}};
    }

    /// Get the u-axis
    b2Vec<float, 2> GetYAxis() const
    {
        return b2Vec<float, 2>{{-s, c}};
    }

    /// Sine and cosine
    float s, c;
};

/// A transform contains translation and rotation. It is used to represent
/// the position and orientation of rigid frames.
struct b2Transform
{
    /// The default constructor does nothing.
    b2Transform() :
        p{{0,0}},
        q(0.0f)
    {
    }

    /// Initialize using a position vector and a rotation.
    b2Transform(b2Vec<float, 2> position, b2Rot rotation) : p(std::move(position)), q(std::move(rotation))
    {
    }

    /// Set this to the identity transform.
    void SetIdentity()
    {
        p = {{0, 0}};
        q.SetIdentity();
    }

    /// Set this based on the position and angle.
    void Set(const b2Vec<float, 2>& position, float angle)
    {
        p = position;
        q.Set(angle);
    }

    b2Vec<float, 2> p;
    b2Rot q;
};

/// This describes the motion of a body/shape for TOI computation.
/// Shapes are defined with respect to the body origin, which may
/// no coincide with the center of mass. However, to support dynamics
/// we must interpolate the center of mass position.
struct b2Sweep
{
    /// Get the interpolated transform at a specific time.
    /// @param beta is a factor in [0,1], where 0 indicates alpha0.
    void GetTransform(b2Transform* xfb, float beta) const;

    /// Advance the sweep forward, yielding a new initial state.
    /// @param alpha the new initial time.
    void Advance(float alpha);

    /// Normalize the angles.
    void Normalize();

    b2Vec<float, 2> localCenter;  ///< local center of mass position
    b2Vec<float, 2> c0, c;        ///< center world positions
    float a0, a;       ///< world angles

    /// Fraction of the current time step in the range [0,1]
    /// c0 and a0 are the positions at alpha0.
    float alpha0;
};

/// Useful constant
extern const b2Vec<float, 2> b2Vec2_zero;

/// Perform the dot product on two vectors.
inline float b2Dot(const b2Vec<float, 2>& a, const b2Vec<float, 2>& b)
{
    return a[b2VecX] * b[b2VecX] + a[b2VecY] * b[b2VecY];
}

/// Perform the cross product on two vectors. In 2D this produces a scalar.
inline float b2Cross(const b2Vec<float, 2>& a, const b2Vec<float, 2>& b)
{
    return a[b2VecX] * b[b2VecY] - a[b2VecY] * b[b2VecX];
}

/// Perform the cross product on a vector and a scalar. In 2D this produces
/// a vector.
inline b2Vec<float, 2> b2Cross(const b2Vec<float, 2>& a, float s)
{
    return b2Vec<float, 2>{{s * a[b2VecY], -s * a[b2VecX]}};
}

/// Perform the cross product on a scalar and a vector. In 2D this produces
/// a vector.
inline b2Vec<float, 2> b2Cross(float s, const b2Vec<float, 2>& a)
{
    return b2Vec<float, 2>{{-s * a[b2VecY], s * a[b2VecX]}};
}

/// Multiply a matrix times a vector. If a rotation matrix is provided,
/// then this transforms the vector from one frame to another.
inline b2Vec<float, 2> b2Mul(const b2Mat22& A, const b2Vec<float, 2>& v)
{
    return b2Vec<float, 2>{{A.ex[b2VecX] * v[b2VecX] + A.ey[b2VecX] * v[b2VecY], A.ex[b2VecY] * v[b2VecX] + A.ey[b2VecY] * v[b2VecY]}};
}

/// Multiply a matrix transpose times a vector. If a rotation matrix is provided,
/// then this transforms the vector from one frame to another (inverse transform).
inline b2Vec<float, 2> b2MulT(const b2Mat22& A, const b2Vec<float, 2>& v)
{
    return b2Vec<float, 2>{{b2Dot(v, A.ex), b2Dot(v, A.ey)}};
}

inline float b2Distance(const b2Vec<float, 2>& a, const b2Vec<float, 2>& b)
{
    b2Vec<float, 2> c = a - b;
    return c.Length();
}

inline float b2DistanceSquared(const b2Vec<float, 2>& a, const b2Vec<float, 2>& b)
{
    b2Vec<float, 2> c = a - b;
    return b2Dot(c, c);
}

/// Perform the dot product on two vectors.
inline float b2Dot(const b2Vec<float, 3>& a, const b2Vec<float, 3>& b)
{
    return a[b2VecX] * b[b2VecX] + a[b2VecY] * b[b2VecY] + a[b2VecZ] * b[b2VecZ];
}

/// Perform the cross product on two vectors.
inline b2Vec<float, 3> b2Cross(const b2Vec<float, 3>& a, const b2Vec<float, 3>& b)
{
    return b2Vec<float, 3>(a[b2VecY] * b[b2VecZ] - a[b2VecZ] * b[b2VecY], a[b2VecZ] * b[b2VecX] - a[b2VecX] * b[b2VecZ], a[b2VecX] * b[b2VecY] - a[b2VecY] * b[b2VecX]);
}

inline b2Mat22 operator+(const b2Mat22& A, const b2Mat22& B)
{
    return b2Mat22(A.ex + B.ex, A.ey + B.ey);
}

// A * B
inline b2Mat22 b2Mul(const b2Mat22& A, const b2Mat22& B)
{
    return b2Mat22(b2Mul(A, B.ex), b2Mul(A, B.ey));
}

// A^CONTAINER_TYPE * B
inline b2Mat22 b2MulT(const b2Mat22& A, const b2Mat22& B)
{
    b2Vec<float, 2> c1{{b2Dot(A.ex, B.ex), b2Dot(A.ey, B.ex)}};
    b2Vec<float, 2> c2{{b2Dot(A.ex, B.ey), b2Dot(A.ey, B.ey)}};
    return b2Mat22(c1, c2);
}

/// Multiply a matrix times a vector.
inline b2Vec<float, 3> b2Mul(const b2Mat33& A, const b2Vec<float, 3>& v)
{
    return v[b2VecX] * A.ex + v[b2VecY] * A.ey + v[b2VecZ] * A.ez;
}

/// Multiply a matrix times a vector.
inline b2Vec<float, 2> b2Mul22(const b2Mat33& A, const b2Vec<float, 2>& v)
{
    return b2Vec<float, 2>{{A.ex[b2VecX] * v[b2VecX] + A.ey[b2VecX] * v[b2VecY], A.ex[b2VecY] * v[b2VecX] + A.ey[b2VecY] * v[b2VecY]}};
}

/// Multiply two rotations: q * r
inline b2Rot b2Mul(const b2Rot& q, const b2Rot& r)
{
    // [qc -qs] * [rc -rs] = [qc*rc-qs*rs -qc*rs-qs*rc]
    // [qs  qc]   [rs  rc]   [qs*rc+qc*rs -qs*rs+qc*rc]
    // s = qs * rc + qc * rs
    // c = qc * rc - qs * rs
    b2Rot qr;
    qr.s = q.s * r.c + q.c * r.s;
    qr.c = q.c * r.c - q.s * r.s;
    return qr;
}

/// Transpose multiply two rotations: qT * r
inline b2Rot b2MulT(const b2Rot& q, const b2Rot& r)
{
    // [ qc qs] * [rc -rs] = [qc*rc+qs*rs -qc*rs+qs*rc]
    // [-qs qc]   [rs  rc]   [-qs*rc+qc*rs qs*rs+qc*rc]
    // s = qc * rs - qs * rc
    // c = qc * rc + qs * rs
    b2Rot qr;
    qr.s = q.c * r.s - q.s * r.c;
    qr.c = q.c * r.c + q.s * r.s;
    return qr;
}

/// Rotate a vector
inline b2Vec<float, 2> b2Mul(const b2Rot& q, const b2Vec<float, 2>& v)
{
    return b2Vec<float, 2>{{q.c * v[b2VecX] - q.s * v[b2VecY], q.s * v[b2VecX] + q.c * v[b2VecY]}};
}

/// Inverse rotate a vector
inline b2Vec<float, 2> b2MulT(const b2Rot& q, const b2Vec<float, 2>& v)
{
    return b2Vec<float, 2>{{q.c * v[b2VecX] + q.s * v[b2VecY], -q.s * v[b2VecX] + q.c * v[b2VecY]}};
}

inline b2Vec<float, 2> b2Mul(const b2Transform& transform, const b2Vec<float, 2>& v)
{
    float x = (transform.q.c * v[b2VecX] - transform.q.s * v[b2VecY]) + transform.p[b2VecX];
    float y = (transform.q.s * v[b2VecX] + transform.q.c * v[b2VecY]) + transform.p[b2VecY];

    return b2Vec<float, 2>{{x, y}};
}

inline b2Vec<float, 2> b2MulT(const b2Transform& transform, const b2Vec<float, 2>& v)
{
    float px = v[b2VecX] - transform.p[b2VecX];
    float py = v[b2VecY] - transform.p[b2VecY];
    float x = (transform.q.c * px + transform.q.s * py);
    float y = (-transform.q.s * px + transform.q.c * py);

    return b2Vec<float, 2>{{x, y}};
}

// v2 = A.q.Rot(B.q.Rot(v1) + B.p) + A.p
//    = (A.q * B.q).Rot(v1) + A.q.Rot(B.p) + A.p
inline b2Transform b2Mul(const b2Transform& A, const b2Transform& B)
{
    b2Transform C;
    C.q = b2Mul(A.q, B.q);
    C.p = b2Mul(A.q, B.p) + A.p;
    return C;
}

// v2 = A.q' * (B.q * v1 + B.p - A.p)
//    = A.q' * B.q * v1 + A.q' * (B.p - A.p)
inline b2Transform b2MulT(const b2Transform& A, const b2Transform& B)
{
    b2Transform C;
    C.q = b2MulT(A.q, B.q);
    C.p = b2MulT(A.q, B.p - A.p);
    return C;
}
/*
template <typename CONTAINER_TYPE>
inline CONTAINER_TYPE b2Abs(CONTAINER_TYPE a)
{
	return std::abs(a);
//     return a > CONTAINER_TYPE(0) ? a : -a;
}*/

inline b2Vec<float, 2> b2Abs(const b2Vec<float, 2>& a)
{
    return b2Vec<float, 2>{{std::abs(a[b2VecX]), std::abs(a[b2VecY])}};
}

inline b2Mat22 b2Abs(const b2Mat22& A)
{
    return b2Mat22(b2Abs(A.ex), b2Abs(A.ey));
}

template <typename TYPE>
inline TYPE b2Min(TYPE a, TYPE b)
{
    return a < b ? a : b;
}

inline b2Vec<float, 2> b2Min(const b2Vec<float, 2>& a, const b2Vec<float, 2>& b)
{
    return b2Vec<float, 2>{{b2Min(a[b2VecX], b[b2VecX]), b2Min(a[b2VecY], b[b2VecY])}};
}

template <typename TYPE>
inline TYPE b2Max(TYPE a, TYPE b)
{
    return a > b ? a : b;
}

inline b2Vec<float, 2> b2Max(const b2Vec<float, 2>& a, const b2Vec<float, 2>& b)
{
    return b2Vec<float, 2>{{b2Max(a[b2VecX], b[b2VecX]), b2Max(a[b2VecY], b[b2VecY])}};
}

template <typename TYPE>
inline TYPE b2Clamp(TYPE a, TYPE low, TYPE high)
{
    return b2Max(low, b2Min(a, high));
}

inline b2Vec<float, 2> b2Clamp(const b2Vec<float, 2>& a, const b2Vec<float, 2>& low, const b2Vec<float, 2>& high)
{
    return b2Max(low, b2Min(a, high));
}

template <typename TYPE>
inline void b2Swap(TYPE& a, TYPE& b)
{
    TYPE tmp = a;
    a = b;
    b = tmp;
}

/// "Next Largest Power of 2
/// Given a binary integer value x, the next largest power of 2 can be computed by a SWAR algorithm
/// that recursively "folds" the upper bits into the lower bits. This process yields a bit vector
/// with
/// the same most significant 1 as x, but all 1's below it. Adding 1 to that value yields the next
/// largest power of 2. For a 32-bit value:"
inline uint32_t b2NextPowerOfTwo(uint32_t x)
{
    x |= (x >> 1);
    x |= (x >> 2);
    x |= (x >> 4);
    x |= (x >> 8);
    x |= (x >> 16);
    return x + 1;
}

inline bool b2IsPowerOfTwo(uint32_t x)
{
    bool result = x > 0 && (x & (x - 1)) == 0;
    return result;
}

inline void b2Sweep::GetTransform(b2Transform* xf, float beta) const
{
    xf->p = (1.0f - beta) * c0 + beta * c;
    float angle = (1.0f - beta) * a0 + beta * a;
    xf->q.Set(angle);

    // Shift to origin
    xf->p -= b2Mul(xf->q, localCenter);
}

inline void b2Sweep::Advance(float alpha)
{
    b2Assert(alpha0 < 1.0f);
    float beta = (alpha - alpha0) / (1.0f - alpha0);
    c0 += beta * (c - c0);
    a0 += beta * (a - a0);
    alpha0 = alpha;
}

/// Normalize an angle in radians to be between -pi and pi
inline void b2Sweep::Normalize()
{
    float twoPi = 2.0f * B2_PI;
    float d = twoPi * floorf(a0 / twoPi);
    a0 -= d;
    a -= d;
}
}

#endif
