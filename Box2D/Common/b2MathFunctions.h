#ifndef B2MATH_FUNCTIONS_H
#define B2MATH_FUNCTIONS_H

#include <algorithm>

namespace box2d
{

#define b2Sqrt(x) sqrtf(x)
#define b2Atan2(y, x) atan2f(y, x)

/// This function is used to ensure that a floating point number is not a NaN or infinity.
inline bool b2IsValid(float x)
{
    int32_t ix = 0;
    static_assert(sizeof(x) == sizeof(ix), "Mismatched storage type sizes");
    std::copy(reinterpret_cast<char*>(&x), reinterpret_cast<char*>(&x) + sizeof(x), reinterpret_cast<char*>(&ix));
    return (ix & 0x7f800000) != 0x7f800000;
}

/// This is a approximate yet fast inverse square-root.
inline float b2InvSqrt(float x)
{
    int32_t ix = 0;
    static_assert(sizeof(x) == sizeof(ix), "Mismatched storage type sizes");
    std::copy(reinterpret_cast<char*>(&x), reinterpret_cast<char*>(&x) + sizeof(x), reinterpret_cast<char*>(&ix));

    float xhalf = 0.5f * x;
    ix = 0x5f3759df - (ix >> 1);
    std::copy(reinterpret_cast<char*>(&ix), reinterpret_cast<char*>(&ix) + sizeof(ix), reinterpret_cast<char*>(&x));
    x = x * (1.5f - xhalf * x * x);
    return x;
}


}

#endif
