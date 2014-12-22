#include "ofApp.h"

#include "Test.h"

#include <chrono>

using namespace box2d;

//
struct UIState
{
    bool showMenu;
    int scroll;
    int scrollarea1;
    bool mouseOverMenu;
    bool chooseTest;
};

//
namespace
{
    UIState ui;

    int32_t testIndex = 0;
    int32_t testSelection = 0;
    std::unique_ptr<Test> test;
    Settings settings;
    bool rightMouseDown;
    b2Vec2 lastp;
}

//
static void sCreateUI()
{
    ui.showMenu = true;
    ui.scroll = 0;
    ui.scrollarea1 = 0;
    ui.chooseTest = false;
    ui.mouseOverMenu = false;
}

//
static void sResizeWindow(int width, int height)
{
    g_debugDraw.SetDimensions({{static_cast<float>(width), static_cast<float>(height)}});
}


//
static void sRestart()
{
    test = std::unique_ptr<Test>(g_testEntries[testIndex].createFcn());
}

//
static void sSimulate()
{
    test->Step(&settings);
}

//
static void sInterface()
{
//     int menuWidth = 200;
//     ui.mouseOverMenu = false;
//     if (ui.showMenu)
//     {
//         bool over = imguiBeginScrollArea("Testbed Controls", g_camera.m_width - menuWidth - 10, 10,
//                                          menuWidth, g_camera.m_height - 20, &ui.scrollarea1);
//         if (over)
//             ui.mouseOverMenu = true;
// 
//         imguiSeparatorLine();
// 
//         imguiLabel("Test");
//         if (imguiButton(entry->name, true))
//         {
//             ui.chooseTest = !ui.chooseTest;
//         }
// 
//         imguiSeparatorLine();
// 
//         imguiSlider("Vel Iters", &settings.velocityIterations, 0, 50, 1, true);
//         imguiSlider("Pos Iters", &settings.positionIterations, 0, 50, 1, true);
//         imguiSlider("Hertz", &settings.hz, 5.0f, 120.0f, 5.0f, true);
// 
//         if (imguiCheck("Sleep", settings.enableSleep, true))
//             settings.enableSleep = !settings.enableSleep;
//         if (imguiCheck("Warm Starting", settings.enableWarmStarting, true))
//             settings.enableWarmStarting = !settings.enableWarmStarting;
//         if (imguiCheck("Time of Impact", settings.enableContinuous, true))
//             settings.enableContinuous = !settings.enableContinuous;
//         if (imguiCheck("Sub-Stepping", settings.enableSubStepping, true))
//             settings.enableSubStepping = !settings.enableSubStepping;
// 
//         imguiSeparatorLine();
// 
//         if (imguiCheck("Shapes", settings.drawShapes, true))
//             settings.drawShapes = !settings.drawShapes;
//         if (imguiCheck("Joints", settings.drawJoints, true))
//             settings.drawJoints = !settings.drawJoints;
//         if (imguiCheck("AABBs", settings.drawAABBs, true))
//             settings.drawAABBs = !settings.drawAABBs;
//         if (imguiCheck("Contact Points", settings.drawContactPoints, true))
//             settings.drawContactPoints = !settings.drawContactPoints;
//         if (imguiCheck("Contact Normals", settings.drawContactNormals, true))
//             settings.drawContactNormals = !settings.drawContactNormals;
//         if (imguiCheck("Contact Impulses", settings.drawContactImpulse, true))
//             settings.drawContactImpulse = !settings.drawContactImpulse;
//         if (imguiCheck("Friction Impulses", settings.drawFrictionImpulse, true))
//             settings.drawFrictionImpulse = !settings.drawFrictionImpulse;
//         if (imguiCheck("Center of Masses", settings.drawCOMs, true))
//             settings.drawCOMs = !settings.drawCOMs;
//         if (imguiCheck("Statistics", settings.drawStats, true))
//             settings.drawStats = !settings.drawStats;
//         if (imguiCheck("Profile", settings.drawProfile, true))
//             settings.drawProfile = !settings.drawProfile;
// 
//         if (imguiButton("Pause", true))
//             settings.pause = !settings.pause;
// 
//         if (imguiButton("Single Step", true))
//             settings.singleStep = !settings.singleStep;
// 
//         if (imguiButton("Restart", true))
//             sRestart();
// 
//         if (imguiButton("Quit", true))
//             glfwSetWindowShouldClose(mainWindow, GL_TRUE);
// 
//         imguiEndScrollArea();
//     }
// 
//     int testMenuWidth = 200;
//     if (ui.chooseTest)
//     {
//         static int testScroll = 0;
//         bool over =
//             imguiBeginScrollArea("Choose Sample", g_camera.m_width - menuWidth - testMenuWidth - 20,
//                                  10, testMenuWidth, g_camera.m_height - 20, &testScroll);
//         if (over)
//             ui.mouseOverMenu = true;
// 
//         for (int i = 0; i < testCount; ++i)
//         {
//             if (imguiItem(g_testEntries[i].name, true))
//             {
//                 delete test;
//                 entry = g_testEntries + i;
//                 test = entry->createFcn();
//                 ui.chooseTest = false;
//             }
//         }
// 
//         imguiEndScrollArea();
//     }
// 
//     imguiEndFrame();
}

ofApp::ofApp() : m_hide_gui(false), m_draw_box2d_debug(true)
{
}

//--------------------------------------------------------------
void ofApp::setup()
{
    sResizeWindow(APP_WIDTH, APP_HEIGHT);
    
    g_debugDraw.SetZoom(1.0f);
    g_debugDraw.SetCenter({{0.0f, 20.0f}});
    
    ofSetVerticalSync(true);
    
    //-------------------------------------------------------------------------------------------------------
    char title[64];
    sprintf(title, "Box2D Testbed Version %d.%d.%d", box2d::b2_version.major, box2d::b2_version.minor,
            box2d::b2_version.revision);
    
    testIndex = box2d::b2Clamp(testIndex, 0, static_cast<int>(g_testEntries.size()));
    testSelection = testIndex;
    
    test = std::unique_ptr<Test>(g_testEntries[testIndex].createFcn());
    
    std::cout << "Setup" << std::endl;
    
    //-------------------------------------------------------------------------------------------------------
    
    // we add this listener before setting up so the initial circle resolution is correct
    circleResolution.addListener(this, &ofApp::circleResolutionChanged);
    ringButton.addListener(this, &ofApp::ringButtonPressed);

    gui.setup();  // most of the time you don't need a name
    gui.add(filled.setup("fill", true));
    gui.add(radius.setup("radius", 140, 10, 300));
    gui.add(center.setup("center", ofVec2f(ofGetWidth() * .5, ofGetHeight() * .5), ofVec2f(0, 0),
                         ofVec2f(ofGetWidth(), ofGetHeight())));
    gui.add(color.setup("color", ofColor(100, 100, 140), ofColor(0, 0), ofColor(255, 255)));
    gui.add(circleResolution.setup("circle res", 5, 3, 90));
    gui.add(twoCircles.setup("two circles"));
    gui.add(ringButton.setup("ring"));
    gui.add(screenSize.setup("screen size", ""));

//     b2Vec2 gravity{{0, -9.8}};  // normal earth gravity, 9.8 m/s/s straight down!
//     m_world = make_shared<b2World>(gravity);
//     m_world->SetDebugDraw(&g_debugDraw);
// 
//     g_debugDraw.SetFlags(b2Draw::e_shapeBit | b2Draw::e_centerOfMassBit | b2Draw::e_aabbBit |
//                     b2Draw::e_jointBit | b2Draw::e_pairBit);
// 
//     b2BodyDef myBodyDef;
//     myBodyDef.type = b2DYNAMIC_BODY;  // this will be a dynamic body
//     myBodyDef.position = {{0, 20}};    // set the starting position
//     myBodyDef.angle = 0;              // set the starting angle
// 
//     b2Body* dynamicBody = m_world->CreateBody(&myBodyDef);
// 
//     b2PolygonShape boxShape;
//     boxShape.SetAsBox(1, 1, {{-2, 0}}, 0);
// 
//     b2FixtureDef boxFixtureDef;
//     boxFixtureDef.shape = &boxShape;
//     boxFixtureDef.density = 1;
// 
//     dynamicBody->CreateFixture(&boxFixtureDef);
// 
//     b2CircleShape circleShape;
//     circleShape.SetRadius(1);
//     circleShape.m_p = {{2, 0}};
// 
//     b2FixtureDef circleFixtureDef;
//     circleFixtureDef.shape = &circleShape;
//     circleFixtureDef.density = 1;
// 
//     dynamicBody->CreateFixture(&circleFixtureDef);
// 
//     dynamicBody->SetTransform({{0, 20}}, 1);
// 
//     b2BodyDef groundBodyDef;
//     groundBodyDef.position = {{0.0f, 0.0f}};
//     b2Body* groundBody = m_world->CreateBody(&groundBodyDef);
//     b2PolygonShape groundBox;
//     groundBox.SetAsBox(50.0f, 0.1f);
//     groundBody->CreateFixture(&groundBox, 0.0f);

    ofSetCircleResolution(30);
}

//--------------------------------------------------------------
void ofApp::exit()
{
    ringButton.removeListener(this, &ofApp::ringButtonPressed);
}

//--------------------------------------------------------------
void ofApp::circleResolutionChanged(int& circleResolution)
{
    ofSetCircleResolution(circleResolution);
}

//--------------------------------------------------------------
void ofApp::ringButtonPressed()
{
}

//--------------------------------------------------------------
void ofApp::update()
{
    static auto last_time = std::chrono::high_resolution_clock::now();
    auto current_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> time_step = current_time - last_time;
    last_time = current_time;
    
    sSimulate();

//     constexpr int32_t velocityIterations = 8;  // how strongly to correct velocity
//     constexpr int32_t positionIterations = 3;  // how strongly to correct position
    

//     m_world->Step(time_step.count(), velocityIterations, positionIterations);
}

//--------------------------------------------------------------
void ofApp::draw()
{
    ofBackgroundGradient(ofColor::white, ofColor::gray);
    
    //-------------------------------------------------------------------------------------------------------
    
    
//     sInterface();
    
    //-------------------------------------------------------------------------------------------------------

    // Arrange the view for a normal math based coordinate system with positive up and 0, 0 at the
    // bottom middle of the screen
    

    // auto draw?
    // should the gui control hiding?
    if (m_hide_gui)
    {
        gui.draw();
    }
    
    test->Draw(&settings);

    test->DrawTitle(g_testEntries[testIndex].name);

    if (testSelection != testIndex)
    {
        testIndex = testSelection;
        test = std::unique_ptr<Test>(g_testEntries[testIndex].createFcn());
        g_debugDraw.SetZoom(1.0f);
        g_debugDraw.SetCenter({{0.0f, 20.0f}});
    }
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key)
{
    switch (key)
    {
        case OF_KEY_CONTROL:
            m_modifiers.ctl = true;
            break;

        case OF_KEY_LEFT:
            // Pan left
            if (m_modifiers.ctl)
            {
                b2Vec2 newOrigin{{2.0f, 0.0f}};
                test->ShiftOrigin(newOrigin);
            }
            else
            {
                g_debugDraw.ShiftCenterX(-0.5f);
            }
            break;

        case OF_KEY_RIGHT:
            // Pan right
            if (m_modifiers.ctl)
            {
                b2Vec2 newOrigin{{-2.0f, 0.0f}};
                test->ShiftOrigin(newOrigin);
            }
            else
            {
                g_debugDraw.ShiftCenterX(0.5f);
            }
            break;

        case OF_KEY_DOWN:
            // Pan down
            if (m_modifiers.ctl)
            {
                b2Vec2 newOrigin{{0.0f, 2.0f}};
                test->ShiftOrigin(newOrigin);
            }
            else
            {
                g_debugDraw.ShiftCenterY(-0.5f);
            }
            break;

        case OF_KEY_UP:
            // Pan up
            if (m_modifiers.ctl)
            {
                b2Vec2 newOrigin{{0.0f, -2.0f}};
                test->ShiftOrigin(newOrigin);
            }
            else
            {
                g_debugDraw.ShiftCenterY(0.5f);
            }
            break;

        case OF_KEY_HOME:
            // Reset view
            g_debugDraw.SetZoom(1.0f);
            g_debugDraw.SetCenter({{0.0f, 20.0f}});
            break;

        case 'z':
            // Zoom out
            g_debugDraw.SetZoom(b2Min(1.1f * g_debugDraw.GetZoom(), 20.0f));
            break;

        case 'x':
            // Zoom in
            g_debugDraw.SetZoom(b2Max(0.9f * g_debugDraw.GetZoom(), 0.02f));
            break;

        case 'r':
            // Reset test
            test = std::unique_ptr<Test>(g_testEntries[testIndex].createFcn());
            break;

        case ' ':
            // Launch a bomb.
            if (test)
            {
                test->LaunchBomb();
            }
            break;

        case 'p':
            // Pause
            settings.pause = !settings.pause;
            break;

        case '[':
            // Switch to previous test
            --testSelection;
            if (testSelection < 0)
            {
                testSelection = g_testEntries.size() - 1;
            }
            break;

        case ']':
            // Switch to next test
            ++testSelection;
            if (testSelection == g_testEntries.size())
            {
                testSelection = 0;
            }
            break;

        case '\t':
            ui.showMenu = !ui.showMenu;

        default:
            if (test)
            {
                test->Keyboard(key);
            }
    }

    std::cout << std::hex << static_cast<int>(key) << std::endl;
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key)
{
    
    switch (key)
    {
        case OF_KEY_CONTROL:
            m_modifiers.ctl = false;
            break;
    }
    
    std::cout << std::hex << static_cast<int>(key) << std::endl;
    test->KeyboardUp(key);
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y)
{
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button)
{
 
    if (button == OF_MOUSE_BUTTON_1)
    {
        b2Vec2 ps{{static_cast<float>(x), static_cast<float>(y)}};
        
        b2Vec2 pw = g_debugDraw.ConvertScreenToWorld(ps);
        test->MouseMove(pw);
        
        if (rightMouseDown)
        {
            b2Vec2 diff = pw - lastp;
            g_debugDraw.ShiftCenterX(-diff[b2VecX]);
            g_debugDraw.ShiftCenterY(-diff[b2VecY]);
            lastp = g_debugDraw.ConvertScreenToWorld(ps);
        }
    }
    else if (button == OF_MOUSE_BUTTON_3 || button == OF_MOUSE_BUTTON_4)
    {
        if (y > 0)
        {
            g_debugDraw.SetZoom(g_debugDraw.GetZoom() / 1.1f);
        }
        else
        {
            g_debugDraw.SetZoom(g_debugDraw.GetZoom() * 1.1f);
        }
    }
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button)
{
    b2Vec2 ps = {{(float32)x, (float32)y}};
    
    // Use the mouse to move things around.
    if (button == OF_MOUSE_BUTTON_1)
    {
        //<##>
        // ps = {{0, 0}};
        b2Vec2 pw = g_debugDraw.ConvertScreenToWorld(ps);
        
        if (m_modifiers.shift)
        {
            test->ShiftMouseDown(pw);
        }
        else
        {
            test->MouseDown(pw);
        }
    }
    else if (button == OF_MOUSE_BUTTON_2)
    {
        lastp = g_debugDraw.ConvertScreenToWorld(ps);
        rightMouseDown = true;
    }
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button)
{
    b2Vec2 ps = {{(float32)x, (float32)y}};
    switch (button)
    {
        case OF_MOUSE_BUTTON_1:
            {
                b2Vec2 pw = g_debugDraw.ConvertScreenToWorld(ps);
                test->MouseUp(pw);
            }
            break;
        case OF_MOUSE_BUTTON_2:
            {
                rightMouseDown = false;   
            }
            break;
    }
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h)
{
    screenSize = ofToString(w) + "x" + ofToString(h);
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg)
{
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo)
{
}
