#ifndef TEST_HELPERS_HPP
#define TEST_HELPERS_HPP
#pragma once

#include "gtest/gtest.h"
#include <Box2D/Box2D.h>
#include "../box2d-ref/Box2DRef/Box2D.h"

static void compareB2Vec2(const box2d::b2Vec<float,2>& vec, const box2dref::b2Vec2& vecref)
{
    EXPECT_FLOAT_EQ(vec[0], vecref.x);
    EXPECT_FLOAT_EQ(vec[1], vecref.y);
}

static void compareB2Manifold(const box2d::b2Manifold& manifold, const box2dref::b2Manifold& manifoldref)
{
    compareB2Vec2(manifold.localNormal, manifoldref.localNormal);
    compareB2Vec2(manifold.localPoint, manifoldref.localPoint);
    ASSERT_EQ(manifold.pointCount, manifoldref.pointCount);
    for (int i = 0; i < manifold.pointCount; ++i)
    {
        compareB2Vec2(manifold.points[i].localPoint, manifoldref.points[i].localPoint);
//         EXPECT_FLOAT_EQ(manifold.points[i].normalImpulse, manifoldref.points[i].normalImpulse);
//         EXPECT_FLOAT_EQ(manifold.points[i].tangentImpulse, manifoldref.points[i].tangentImpulse);
    }
}

static bool compareShape(const box2d::b2Shape& shape, const box2dref::b2Shape& shaperef)
{
    bool ret = false;
    
    EXPECT_EQ((int)shape.GetType(), (int)shaperef.m_type);
    EXPECT_FLOAT_EQ(shape.GetRadius(), shaperef.m_radius);
    
    return ret;
}

static bool compareBody(const box2d::b2Body& body, const box2dref::b2Body& bodyref)
{
    bool ret = false;
    
    EXPECT_FLOAT_EQ(body.GetAngle(), bodyref.GetAngle());
    EXPECT_FLOAT_EQ(body.GetAngularDamping(), bodyref.GetAngularDamping());
    EXPECT_FLOAT_EQ(body.GetAngularVelocity(), bodyref.GetAngularVelocity());
    EXPECT_FLOAT_EQ(body.GetGravityScale(), bodyref.GetGravityScale());
    EXPECT_FLOAT_EQ(body.GetLinearDamping(), bodyref.GetLinearDamping());
    compareB2Vec2(body.GetPosition(), bodyref.GetPosition());
    EXPECT_FLOAT_EQ(body.GetAngularVelocity(), bodyref.GetAngularVelocity());
//     EXPECT_EQ(body.GetContactList(), bodyref.GetContactList());
    EXPECT_EQ((int)body.GetType(), bodyref.GetType());
    compareB2Vec2(body.GetWorldCenter(), bodyref.GetWorldCenter());
    EXPECT_EQ(body.IsActive(), bodyref.IsActive());
    EXPECT_EQ(body.IsAwake(), bodyref.IsAwake());
    EXPECT_EQ(body.IsBullet(), bodyref.IsBullet());
//     EXPECT_EQ(body.GetAngularDamping(), bodyref.GetAngularDamping());
//     EXPECT_EQ(body.GetAngularDamping(), bodyref.GetAngularDamping());
//     EXPECT_EQ(body.GetAngularDamping(), bodyref.GetAngularDamping());
//     EXPECT_EQ(body.GetAngularDamping(), bodyref.GetAngularDamping());
    
    return ret;
}

#endif
