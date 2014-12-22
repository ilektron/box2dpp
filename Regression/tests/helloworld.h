#ifndef _HELLOWORLD_TEST_H_
#define _HELLOWORLD_TEST_H_

#include "gtest/gtest.h"
#include "../box2d-ref/Box2DRef/Box2D.h"
#include <Box2D/Box2D.h>

class b2RegressionTest : public ::testing::Test
{
public:
    b2RegressionTest () = default;
    
    virtual ~b2RegressionTest() = default;
    
protected:
    virtual void SetUp() override;
    virtual void TearDown() override;
    
    ::b2World* refWorld;
//     box2d::b2World* world;
};

#endif
