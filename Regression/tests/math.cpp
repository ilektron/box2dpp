// Math tests

#include "gtest/gtest.h"
#include "../box2d-ref/Box2DRef/Box2D.h"
#include "test_helpers.hpp"
#include <iostream>
#include <vector>

#include <Box2D/Box2D.h>

TEST(Math, b2VecInitialize)
{
    {
        box2d::b2Vec<float, 2> vec{};
        EXPECT_FLOAT_EQ(0.0f, vec[0]);
        EXPECT_FLOAT_EQ(0.0f, vec[1]);
    }
    {
        box2d::b2Vec<float, 3> vec{};
        EXPECT_FLOAT_EQ(0.0f, vec[0]);
        EXPECT_FLOAT_EQ(0.0f, vec[1]);
        EXPECT_FLOAT_EQ(0.0f, vec[2]);
    }
    {
        box2d::b2Vec<float, 2> vec{1.0f, 2.0f};
        EXPECT_FLOAT_EQ(1.0f, vec[0]);
        EXPECT_FLOAT_EQ(2.0f, vec[1]);
    }
    {
        box2d::b2Vec<float, 2> vec{1.0f, 2.0f};
        EXPECT_FLOAT_EQ(1.0f, vec[0]);
        EXPECT_FLOAT_EQ(1.0f, vec[0]);
    }
    {
        box2d::b2Vec<float, 3> vec{1000.0f, 2000.0f, -3000.0f};
        EXPECT_FLOAT_EQ(1000.0f, vec[0]);
        EXPECT_FLOAT_EQ(2000.0f, vec[1]);
        EXPECT_FLOAT_EQ(-3000.0f, vec[2]);
    }
}

TEST(Math, b2VecAssignment)
{
    {
        box2d::b2Vec<float, 2> vec;
        vec = {0.0f, 0.0f};
        EXPECT_FLOAT_EQ(0.0f, vec[0]);
        EXPECT_FLOAT_EQ(0.0f, vec[1]);
    }
    {
        box2d::b2Vec<float, 3> vec{};
        vec = {0.0f, 0.0f, 0.0f};
        EXPECT_FLOAT_EQ(0.0f, vec[0]);
        EXPECT_FLOAT_EQ(0.0f, vec[1]);
        EXPECT_FLOAT_EQ(0.0f, vec[2]);
    }
    {
        box2d::b2Vec<float, 5> vec;
        box2d::b2Vec<float, 5> vec2{1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
        vec = vec2;
        EXPECT_FLOAT_EQ(1.0f, vec[0]);
        EXPECT_FLOAT_EQ(2.0f, vec[1]);
        EXPECT_FLOAT_EQ(3.0f, vec[2]);
        EXPECT_FLOAT_EQ(4.0f, vec[3]);
        EXPECT_FLOAT_EQ(5.0f, vec[4]);
    }
}

TEST(Math, b2VecInvert)
{
    box2d::b2Vec<float, 2> vec{{-1.0f, 0.12345f}};
    box2dref::b2Vec2 vecref(-1.0f, 0.12345f);

    vec = vec.Negate();
    compareB2Vec2(vec, -vecref);
}

TEST(Math, b2VecAdd)
{
    box2d::b2Vec<float, 2> vec1{{-1.0f, 0.12345f}};
    box2d::b2Vec<float, 2> vec2{{1.1f, -20.12345f}};
    box2dref::b2Vec2 vecref1(-1.0f, 0.12345f);
    box2dref::b2Vec2 vecref2(1.1f, -20.12345f);

    auto result = vec1 + vec2;
    auto resultref = vecref1 + vecref2;

    compareB2Vec2(result, resultref);
}

TEST(Math, b2VecSubtract)
{
    box2d::b2Vec<float, 2> vec1{{-1.0f, 0.12345f}};
    box2d::b2Vec<float, 2> vec2{{1.1f, -20.12345f}};
    box2dref::b2Vec2 vecref1(-1.0f, 0.12345f);
    box2dref::b2Vec2 vecref2(1.1f, -20.12345f);

    auto result = vec1 - vec2;
    auto resultref = vecref1 - vecref2;

    compareB2Vec2(result, resultref);
}

TEST(Math, b2VecMultiplyScalar)
{
    box2d::b2Vec<float, 2> vec1{{-1.0f, 0.12345f}};
    box2d::b2Vec<float, 2> vec2{{1.1f, -20.12345f}};
    box2d::b2Vec<float, 3> vec3{{1.1f, -20.12345f, 8888.0f}};
    box2dref::b2Vec2 vecref2(1.1f, -20.12345f);
    box2dref::b2Vec3 vecref3(1.1f, -20.12345f, 8888.0f);

    auto result1 = vec1 * 1.234f;
    box2dref::b2Vec2 resultref1(vec1[0] * 1.234f, vec1[1] * 1.234f);
    compareB2Vec2(result1, resultref1);

    auto result2 = 838.123f * vec2;
    auto resultref2 = 838.123f * vecref2;
    compareB2Vec2(result2, resultref2);

    vec3 *= 12333212.0f;
    vecref3 *= 12333212.0f;
    compareB2Vec3(vec3, vecref3);
}
