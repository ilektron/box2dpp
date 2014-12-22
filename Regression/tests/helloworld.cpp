#include "helloworld.h"
#include "gtest/gtest.h"
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
        
        // Now print the position and angle of the body.
        box2dref::b2Vec2 position = body->GetPosition();
        box2dref::float32 angle = body->GetAngle();
        
        printf("%d %4.2f %4.2f %4.2f\n", i, position.x, position.y, angle);
        points.push_back(std::make_pair(body->GetPosition(), body->GetAngle()));
        printf("%d %4.2f %4.2f %4.2f\n", i, groundBody->GetPosition().x, groundBody->GetPosition().y, groundBody->GetAngle());
    }
    
    return points;
}

std::vector<std::pair<box2d::b2Vec2, float>> runHelloWorld()
{
    
    std::vector<std::pair<box2d::b2Vec2, float>> points;
    
    // Define the gravity vector.
    box2d::b2Vec2 gravity{{0.0f, -10.0f}};
    
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
    box2d::float32 timeStep = 1.0f / 60.0f;
    int32_t velocityIterations = 6;
    int32_t positionIterations = 2;
    
    // This is our little game loop.
    for (int32_t i = 0; i < 60; ++i)
    {
        // Instruct the world to perform a single step of simulation.
        // It is generally best to keep the time step and iterations fixed.
        world.Step(timeStep, velocityIterations, positionIterations);
        
        // Now print the position and angle of the body.
        box2d::b2Vec2 position = body->GetPosition();
        box2d::float32 angle = body->GetAngle();
        points.push_back(std::make_pair(position, angle));
        
        printf("%d %4.2f %4.2f %4.2f\n", i, position[box2d::b2VecX], position[box2d::b2VecY], angle);
        printf("%d %4.2f %4.2f %4.2f\n", i, groundBody->GetPosition()[box2d::b2VecX], groundBody->GetPosition()[box2d::b2VecY], groundBody->GetAngle());
    }
    
    return points;
}

TEST_F(b2RegressionTest, TestTest)
{
    auto dataref = runHelloWorldRef();
    auto data = runHelloWorld();
    
    ASSERT_EQ(data.size(), dataref.size());
    for (int i = 0; i < dataref.size(); ++i)
    {
//         ASSERT_FLOAT_EQ(dataref[i].first.x, data[i].first[box2d::b2VecX]) << "For i:" << i;
        ASSERT_FLOAT_EQ(dataref[i].first.y, data[i].first[box2d::b2VecY]) << "For i:" << i;
        ASSERT_FLOAT_EQ(dataref[i].second, data[i].second) << "For i:" << i;
    }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
