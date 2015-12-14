#include "helloworld.h"
#include "gtest/gtest.h"
#include "test_helpers.hpp"
#include <iostream>
#include <vector>

#include <Box2D/Box2D.h>

void b2RegressionTest::SetUp()
{
}

void b2RegressionTest::TearDown()
{
}


std::vector<std::pair<box2dref::b2Vec2, float>> runHelloWorldRef()
{
    std::vector<std::pair<box2dref::b2Vec2, float>> points;
    
    // Define the gravity vector.
    box2dref::b2Vec2 gravity(0.0f, -10.0f);
    
    // Construct a world object, which will hold and simulate the rigid bodies.
    box2dref::b2World world(gravity);
    
    // Define the ground body.
    box2dref::b2BodyDef groundBodyDef;
    groundBodyDef.position.Set(0.0f, -10.0f);
    
    // Call the body factory which allocates memory for the ground body
    // from a pool and creates the ground box shape (also from a pool).
    // The body is also added to the world.
    box2dref::b2Body* groundBody = world.CreateBody(&groundBodyDef);
    
    // Define the ground box shape.
    box2dref::b2PolygonShape groundBox;
    
    // The extents are the half-widths of the box.
    groundBox.SetAsBox(50.0f, 10.0f);
    
    // Add the ground fixture to the ground body.
    groundBody->CreateFixture(&groundBox, 0.0f);
    
    // Define the dynamic body. We set its position and call the body factory.
    box2dref::b2BodyDef bodyDef;
    bodyDef.type = box2dref::b2_dynamicBody;
    bodyDef.position.Set(0.0f, 4.0f);
    box2dref::b2Body* body = world.CreateBody(&bodyDef);
    
    // Define another box shape for our dynamic body.
    box2dref::b2PolygonShape dynamicBox;
    dynamicBox.SetAsBox(1.0f, 1.0f);
    
    // Define the dynamic body fixture.
    box2dref::b2FixtureDef fixtureDef;
    fixtureDef.shape = &dynamicBox;
    
    // Set the box density to be non-zero, so it will be dynamic.
    fixtureDef.density = 1.0f;
    
    // Override the default friction.
    fixtureDef.friction = 0.3f;
    
    // Add the shape to the body.
    body->CreateFixture(&fixtureDef);
    
    // Prepare for simulation. Typically we use a time step of 1/60 of a
    // second (60Hz) and 10 iterations. This provides a high quality simulation
    // in most game scenarios.
    box2dref::float32 timeStep = 1.0f / 60.0f;
    int32_t velocityIterations = 6;
    int32_t positionIterations = 2;
    
    // This is our little game loop.
    for (int32_t i = 0; i < 60; ++i)
    {
        // Instruct the world to perform a single step of simulation.
        // It is generally best to keep the time step and iterations fixed.
        world.Step(timeStep, velocityIterations, positionIterations);
        
        points.push_back(std::make_pair(body->GetPosition(), body->GetAngle()));
    }
    
    return points;
}

std::vector<std::pair<box2d::b2Vec<float,2>, float>> runHelloWorld()
{
    
    std::vector<std::pair<box2d::b2Vec<float,2>, float>> points;
    
    // Define the gravity vector.
    box2d::b2Vec<float,2> gravity{{0.0f, -10.0f}};
    
    // Construct a world object, which will hold and simulate the rigid bodies.
    box2d::b2World world(gravity);
    
    // Define the ground body.
    box2d::b2BodyDef groundBodyDef;
    groundBodyDef.type = box2d::b2BodyType::STATIC_BODY;
    groundBodyDef.position = {{0.0f, -10.0f}};
    
    // Call the body factory which allocates memory for the ground body
    // from a pool and creates the ground box shape (also from a pool).
    // The body is also added to the world.
    box2d::b2Body* groundBody = world.CreateBody(&groundBodyDef);
    
    // Define the ground box shape.
    box2d::b2PolygonShape groundBox;
    
    // The extents are the half-widths of the box.
    groundBox.SetAsBox(50.0f, 10.0f);
    
    // Add the ground fixture to the ground body.
    groundBody->CreateFixture(&groundBox, 0.0f);
    
    // Define the dynamic body. We set its position and call the body factory.
    box2d::b2BodyDef bodyDef;
    bodyDef.type = box2d::b2BodyType::DYNAMIC_BODY;
    bodyDef.position = {{0.0f, 4.0f}};
    box2d::b2Body* body = world.CreateBody(&bodyDef);
    
    // Define another box shape for our dynamic body.
    box2d::b2PolygonShape dynamicBox;
    dynamicBox.SetAsBox(1.0f, 1.0f);
    
    // Define the dynamic body fixture.
    box2d::b2FixtureDef fixtureDef;
    fixtureDef.shape = &dynamicBox;
    
    // Set the box density to be non-zero, so it will be dynamic.
    fixtureDef.density = 1.0f;
    
    // Override the default friction.
    fixtureDef.friction = 0.3f;
    
    // Add the shape to the body.
    body->CreateFixture(&fixtureDef);
    
    // Prepare for simulation. Typically we use a time step of 1/60 of a
    // second (60Hz) and 10 iterations. This provides a high quality simulation
    // in most game scenarios.
    float timeStep = 1.0f / 60.0f;
    int32_t velocityIterations = 6;
    int32_t positionIterations = 2;
    
    // This is our little game loop.
    for (int32_t i = 0; i < 60; ++i)
    {
        // Instruct the world to perform a single step of simulation.
        // It is generally best to keep the time step and iterations fixed.
        world.Step(timeStep, velocityIterations, positionIterations);
        
        // Now print the position and angle of the body.
        box2d::b2Vec<float,2> position = body->GetPosition();
        float angle = body->GetAngle();
        points.push_back(std::make_pair(position, angle));
        
//         printf("%d %4.2f %4.2f %4.2f\n", i, position[box2d::b2VecX], position[box2d::b2VecY], angle);
//         printf("%d %4.2f %4.2f %4.2f\n", i, groundBody->GetPosition()[box2d::b2VecX], groundBody->GetPosition()[box2d::b2VecY], groundBody->GetAngle());
    }
    
    return points;
}

TEST(ChainShape, Construction)
{
    box2d::b2ChainShape chain;
    box2dref::b2ChainShape chainref;
    
    compareShape(chain, chainref);
    EXPECT_EQ(chain.GetChildCount(), chainref.GetChildCount());
}

TEST(EdgeShape, Construction)
{
    box2d::b2EdgeShape edge;
    box2dref::b2EdgeShape edgeref;
    
    compareShape(edge, edgeref);
}

TEST(EdgeShape, Body)
{
    box2d::b2Vec<float,2> gravity{{0, -9.8}};
    box2d::b2World world(gravity);
    box2d::b2BodyDef bd;
    box2d::b2Body* ground = world.CreateBody(&bd);
    
    box2d::b2EdgeShape shape;
    shape.Set({{-40.0f, 0.0f}}, {{40.0f, 0.0f}});
    ground->CreateFixture(&shape, 0.0f);
    
    shape.Set({{20.0f, 0.0f}}, {{20.0f, 20.0f}});
    ground->CreateFixture(&shape, 0.0f);
    
    box2dref::b2Vec2 gravityref(0, -9.8);
    box2dref::b2World worldref(gravityref);
    box2dref::b2BodyDef bdref;
    box2dref::b2Body* groundref = worldref.CreateBody(&bdref);
    
    box2dref::b2EdgeShape shaperef;
    shaperef.Set(box2dref::b2Vec2(-40.0f, 0.0f), box2dref::b2Vec2(40.0f, 0.0f));
    groundref->CreateFixture(&shaperef, 0.0f);
    
    shaperef.Set(box2dref::b2Vec2(20.0f, 0.0f), box2dref::b2Vec2(20.0f, 20.0f));
    groundref->CreateFixture(&shaperef, 0.0f);
    
    compareBody(*ground, *groundref);
}

TEST(PolygonShape, Construction)
{
    box2d::b2PolygonShape polygon;
    box2dref::b2PolygonShape polygonref;
    
    compareShape(polygon, polygonref);
}

TEST(CircleShape, Construction)
{
    box2d::b2CircleShape circle;
    box2dref::b2CircleShape circleref;
    
    compareShape(circle, circleref);
}

TEST(Body, Construction)
{
}
/*
/// Compute the collision manifold between two circles.
void b2CollideCircles(b2Manifold* manifold,
                      const b2CircleShape* circleA, const b2Transform& xfA,
                      const b2CircleShape* circleB, const b2Transform& xfB);

/// Compute the collision manifold between a polygon and a circle.
void b2CollidePolygonAndCircle(b2Manifold* manifold,
                               const b2PolygonShape* polygonA, const b2Transform& xfA,
                               const b2CircleShape* circleB, const b2Transform& xfB);

/// Compute the collision manifold between two polygons.
void b2CollidePolygons(b2Manifold* manifold,
                       const b2PolygonShape* polygonA, const b2Transform& xfA,
                       const b2PolygonShape* polygonB, const b2Transform& xfB);

/// Compute the collision manifold between an edge and a circle.
void b2CollideEdgeAndCircle(b2Manifold* manifold,
                            const b2EdgeShape* polygonA, const b2Transform& xfA,
                            const b2CircleShape* circleB, const b2Transform& xfB);

/// Compute the collision manifold between an edge and a circle.
void b2CollideEdgeAndPolygon(b2Manifold* manifold,
                             const b2EdgeShape* edgeA, const b2Transform& xfA,
                             const b2PolygonShape* circleB, const b2Transform& xfB);

/// Clipping for contact manifolds.
int32 b2ClipSegmentToLine(b2ClipVertex vOut[2], const b2ClipVertex vIn[2],
                          const b2Vec2& normal, float offset, int32 vertexIndexA);

/// Determine if two generic shapes overlap.
bool b2TestOverlap( const b2Shape* shapeA, int32 indexA,
                    const b2Shape* shapeB, int32 indexB,
                    const b2Transform& xfA, const b2Transform& xfB);
*/
TEST(Collision, Testb2CollideCirclesTouching)
{
    box2d::b2Manifold manifold;
    {
        box2d::b2CircleShape circle1, circle2;
        box2d::b2Transform xf1, xf2;
        circle1.SetRadius(1.0f);
        circle2.SetRadius(1.0f);
        xf1.Set({{1.0f, 0.0f}}, 1.0f);
        xf2.Set({{-1.0f, 0.0f}}, 1.0f);
        box2d::b2CollideCircles(&manifold, &circle1, xf1, &circle2, xf2);
    }
    
    box2dref::b2Manifold manifoldref;
    {
        box2dref::b2CircleShape circle1, circle2;
        box2dref::b2Transform xf1, xf2;
        circle1.m_radius = 1.0f;
        circle2.m_radius = 1.0f;
        xf1.Set(box2dref::b2Vec2(0.0f, 0.0f), 1.0f);
        xf2.Set(box2dref::b2Vec2(-1.0f, 0.0f), 1.0f);
        box2dref::b2CollideCircles(&manifoldref, &circle1, xf1, &circle2, xf2);
    }
    
    compareB2Manifold(manifold, manifoldref);
}

TEST(Collision, Testb2FindMaxSeparation)
{
    float distance = 0.0f;
    int32_t edge = 0;
    
    {
        box2d::b2PolygonShape poly1, poly2;
        box2d::b2Transform xf1, xf2;
        poly1.SetAsBox(1.0f, 1.0f);
        poly2.SetAsBox(1.0f, 1.0f);
        xf1.Set({{2.0f, 0.0f}}, 1.0f);
        xf2.Set({{-1.0f, 0.0f}}, 1.0f);
        distance = box2d::b2FindMaxSeparation(&edge, &poly1, xf1, &poly2, xf2);
    }
    
    float distanceref = 0.0f;
    int32_t edgeref = 0;
    {
        box2dref::b2PolygonShape poly1, poly2;
        box2dref::b2Transform xf1, xf2;
        poly1.SetAsBox(1.0f, 1.0f);
        poly2.SetAsBox(1.0f, 1.0f);
        xf1.Set(box2dref::b2Vec2(2.0f, 0.0f), 1.0f);
        xf2.Set(box2dref::b2Vec2(-1.0f, 0.0f), 1.0f);
        distanceref = box2dref::b2FindMaxSeparation(&edgeref, &poly1, xf1, &poly2, xf2);
    }
    
    EXPECT_EQ(edge, edgeref);
    EXPECT_FLOAT_EQ(distance, distanceref);
    
}

TEST(Collision, Testb2FindMaxSeparationOverlap)
{
    float distance = 0.0f;
    int32_t edge = 0;
    
    {
        box2d::b2PolygonShape poly1, poly2;
        box2d::b2Transform xf1, xf2;
        poly1.SetAsBox(1.0f, 1.0f);
        poly2.SetAsBox(1.0f, 1.0f);
        xf1.Set({{1.0f, 0.0f}}, 1.0f);
        xf2.Set({{-1.0f, 0.0f}}, 1.0f);
        distance = box2d::b2FindMaxSeparation(&edge, &poly1, xf1, &poly2, xf2);
    }
    
    float distanceref = 0.0f;
    int32_t edgeref = 0;
    {
        box2dref::b2PolygonShape poly1, poly2;
        box2dref::b2Transform xf1, xf2;
        poly1.SetAsBox(1.0f, 1.0f);
        poly2.SetAsBox(1.0f, 1.0f);
        xf1.Set(box2dref::b2Vec2(1.0f, 0.0f), 1.0f);
        xf2.Set(box2dref::b2Vec2(-1.0f, 0.0f), 1.0f);
        distanceref = box2dref::b2FindMaxSeparation(&edgeref, &poly1, xf1, &poly2, xf2);
    }
    
    EXPECT_EQ(edge, edgeref);
    EXPECT_FLOAT_EQ(distance, distanceref);
}

TEST(Collision, Testb2FindIncidentEdge)
{
    float separation{};
    std::array<box2d::b2ClipVertex, 2> incidentEdge;
    {
        box2d::b2PolygonShape poly1, poly2;
        box2d::b2Transform xf1, xf2;
        poly1.SetAsBox(1.0f, 1.0f);
        poly2.SetAsBox(1.0f, 1.0f);
        xf1.Set({{1.0f, 0.0f}}, 0.0f);
        xf2.Set({{-1.0f, 0.0f}}, 0.0f);
        int32_t edge = 0;
        separation = box2d::b2FindMaxSeparation(&edge, &poly1, xf1, &poly2, xf2);
        box2d::b2FindIncidentEdge(incidentEdge, &poly1, xf1, edge, &poly2, xf2);
    }

    float separationRef{};
    box2dref::b2ClipVertex incidentEdgeRef[2];
    {
        box2dref::b2PolygonShape poly1, poly2;
        box2dref::b2Transform xf1, xf2;
        poly1.SetAsBox(1.0f, 1.0f);
        poly2.SetAsBox(1.0f, 1.0f);
        xf1.Set(box2dref::b2Vec2(1.0f, 0.0f), 0.0f);
        xf2.Set(box2dref::b2Vec2(-1.0f, 0.0f), 0.0f);
        int32_t edge = 0;
        separationRef = box2dref::b2FindMaxSeparation(&edge, &poly1, xf1, &poly2, xf2);
        box2dref::b2FindIncidentEdge(incidentEdgeRef, &poly1, xf1, edge, &poly2, xf2);
    }
    
    compareB2Vec2(incidentEdge[0].v, incidentEdgeRef[0].v);
    compareB2Vec2(incidentEdge[1].v, incidentEdgeRef[1].v);
    EXPECT_FLOAT_EQ(separation, separationRef);
}

TEST(Collision, Testb2CollidePolygonsTouching)
{
    box2d::b2Manifold manifold;
    {
        box2d::b2PolygonShape poly1, poly2;
        box2d::b2Transform xf1, xf2;
        poly1.SetAsBox(1.0f, 1.0f);
        poly2.SetAsBox(1.0f, 1.0f);
        xf1.Set({{1.0f, 0.0f}}, 1.0f);
        xf2.Set({{-1.0f, 0.0f}}, 1.0f);
        box2d::b2CollidePolygons(&manifold, &poly1, xf1, &poly2, xf2);
    }
    
    box2dref::b2Manifold manifoldref;
    {
        box2dref::b2PolygonShape poly1, poly2;
        box2dref::b2Transform xf1, xf2;
        poly1.SetAsBox(1.0f, 1.0f);
        poly2.SetAsBox(1.0f, 1.0f);
        xf1.Set(box2dref::b2Vec2(1.0f, 0.0f), 1.0f);
        xf2.Set(box2dref::b2Vec2(-1.0f, 0.0f), 1.0f);
        box2dref::b2CollidePolygons(&manifoldref, &poly1, xf1, &poly2, xf2);
    }
    
    compareB2Manifold(manifold, manifoldref);
}

TEST_F(b2RegressionTest, TestTest)
{
    auto dataref = runHelloWorldRef();
    auto data = runHelloWorld();
    
    ASSERT_EQ(data.size(), dataref.size());
    for (std::size_t i = 0; i < dataref.size(); ++i)
    {
//         ASSERT_FLOAT_EQ(dataref[i].first.x, data[i].first[box2d::b2VecX]) << "For i:" << i;
        ASSERT_FLOAT_EQ(dataref[i].first.y, data[i].first[box2d::b2VecY]) << "For i:" << i;
        ASSERT_FLOAT_EQ(dataref[i].second, data[i].second) << "For i:" << i;
    }
}

