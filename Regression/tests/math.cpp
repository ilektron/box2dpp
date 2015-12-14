// Math tests

#include "gtest/gtest.h"
#include "../box2d-ref/Box2DRef/Box2D.h"
#include "test_helpers.hpp"
#include <iostream>
#include <vector>

#include <Box2D/Box2D.h>

TEST(Math, b2VecInvert)
{
    box2d::b2Vec<float,2> vec{{-1.0f, 0.12345f}};
    box2dref::b2Vec2 vecref(-1.0f, 0.12345f);

    vec = vec.Negate();
    compareB2Vec2(vec, -vecref);
}

TEST(Math, b2VecAdd)
{
    box2d::b2Vec<float,2> vec1{{-1.0f, 0.12345f}};
    box2d::b2Vec<float,2> vec2{{1.1f, -20.12345f}};
    box2dref::b2Vec2 vecref1(-1.0f, 0.12345f);
    box2dref::b2Vec2 vecref2(1.1f, -20.12345f);

    auto result = vec1 + vec2;
    auto resultref = vecref1 + vecref2;

    compareB2Vec2(result, resultref);
}

TEST(Math, b2VecSubtract)
{
    box2d::b2Vec<float,2> vec1{{-1.0f, 0.12345f}};
    box2d::b2Vec<float,2> vec2{{1.1f, -20.12345f}};
    box2dref::b2Vec2 vecref1(-1.0f, 0.12345f);
    box2dref::b2Vec2 vecref2(1.1f, -20.12345f);

    auto result = vec1 - vec2;
    auto resultref = vecref1 - vecref2;

    compareB2Vec2(result, resultref);
}

TEST(Math, b2VecMultiplyScalar)
{
    box2d::b2Vec<float,2> vec1{{-1.0f, 0.12345f}};
    box2d::b2Vec<float,2> vec2{{1.1f, -20.12345f}};
    box2dref::b2Vec2 vecref1(-1.0f, 0.12345f);
    box2dref::b2Vec2 vecref2(1.1f, -20.12345f);

    auto result1 = vec1 * 1.234f;
    auto result2 = 838.123f * vec2;
//     auto resultref1 = vecref1 * box2dref::float32(1.234f);
    auto resultref2 = 838.123f * vecref2;

//    compareB2Vec2(result1, resultref1);
    compareB2Vec2(result2, resultref2);
}

