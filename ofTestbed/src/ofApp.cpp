#include "ofApp.h"

#include <chrono>

ofApp::ofApp() :
	m_hide_gui(false),
	m_draw_box2d_debug(true)
{

}

//--------------------------------------------------------------
void ofApp::setup() 
{
	ofSetVerticalSync(true);

	// we add this listener before setting up so the initial circle resolution is correct
	circleResolution.addListener(this, &ofApp::circleResolutionChanged);
	ringButton.addListener(this,&ofApp::ringButtonPressed);

	gui.setup(); // most of the time you don't need a name
	gui.add(filled.setup("fill", true));
	gui.add(radius.setup( "radius", 140, 10, 300 ));
	gui.add(center.setup("center",ofVec2f(ofGetWidth()*.5,ofGetHeight()*.5),ofVec2f(0,0),ofVec2f(ofGetWidth(),ofGetHeight())));
	gui.add(color.setup("color",ofColor(100,100,140),ofColor(0,0),ofColor(255,255)));
	gui.add(circleResolution.setup("circle res", 5, 3, 90));
	gui.add(twoCircles.setup("two circles"));
	gui.add(ringButton.setup("ring"));
	gui.add(screenSize.setup("screen size", ""));

	b2Vec2 gravity(0, -9.8); //normal earth gravity, 9.8 m/s/s straight down!
	m_world = make_shared<b2World>(gravity);
	m_world->SetDebugDraw(&m_draw);
	
	m_draw.SetFlags(b2Draw::e_shapeBit | b2Draw::e_centerOfMassBit | b2Draw::e_aabbBit | b2Draw::e_jointBit | b2Draw::e_pairBit);
	
	b2BodyDef myBodyDef;
	myBodyDef.type = b2DYNAMIC_BODY; //this will be a dynamic body
	myBodyDef.position.Set(0, 20); //set the starting position
	myBodyDef.angle = 0; //set the starting angle
	
	b2Body* dynamicBody = m_world->CreateBody(&myBodyDef);
	
	b2PolygonShape boxShape;
	boxShape.SetAsBox(1,1, b2Vec2(-2, 0), 0);
	
	b2FixtureDef boxFixtureDef;
	boxFixtureDef.shape = &boxShape;
	boxFixtureDef.density = 1;
	
	dynamicBody->CreateFixture(&boxFixtureDef);
	
	b2CircleShape circleShape;
	circleShape.m_radius = 1;
	circleShape.m_p.Set(2,0);
	
	b2FixtureDef circleFixtureDef;
	circleFixtureDef.shape = &circleShape;
	circleFixtureDef.density = 1;
	
	dynamicBody->CreateFixture(&circleFixtureDef);
	
	dynamicBody->SetTransform(b2Vec2(0, 20), 1);
	
	
	b2BodyDef groundBodyDef;
	groundBodyDef.position.Set(0.0f, 0.0f);
	b2Body* groundBody = m_world->CreateBody(&groundBodyDef);
	b2PolygonShape groundBox;
	groundBox.SetAsBox(50.0f, 0.1f);
	groundBody->CreateFixture(&groundBox, 0.0f);
	
	ofSetCircleResolution(30);
}

//--------------------------------------------------------------
void ofApp::exit()
{
	ringButton.removeListener(this,&ofApp::ringButtonPressed);
}

//--------------------------------------------------------------
void ofApp::circleResolutionChanged(int & circleResolution)
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
	
	int32 velocityIterations = 8;   //how strongly to correct velocity
	int32 positionIterations = 3;   //how strongly to correct position

	m_world->Step( time_step.count(), velocityIterations, positionIterations);
}

//--------------------------------------------------------------
void ofApp::draw()
{
    ofBackgroundGradient(ofColor::white, ofColor::gray);
	
	// Arrange the view for a normal math based coordinate system with positive up and 0, 0 at the bottom middle of the screen
	ofTranslate(APP_WIDTH/2.0f, APP_HEIGHT);
	ofScale(1, -1);  // Flip Y
	
	ofPushStyle();
	ofSetColor(255, 0, 0, 255);
	ofFill();
	ofDrawCircle(0, 0, 1);
	ofPopStyle();
    
	ofScale(OFXB2DBASIC_WORLD_SCALE, OFXB2DBASIC_WORLD_SCALE, 1.0f);
	if (filled)
	{
		ofFill();
	}
	else
	{
		ofNoFill();
	}

// 	ofSetColor(color);
// 	if(twoCircles)
// 	{
// 		ofDrawCircle(center->x-radius*.5, center->y, radius );
// 		ofDrawCircle(center->x+radius*.5, center->y, radius );
// 	}
// 	else
// 	{
// 		ofDrawCircle((ofVec2f)center, radius );
// 	}
	
	// auto draw?
	// should the gui control hiding?
	if(m_hide_gui)
	{
		gui.draw();
	}
	
	glPushMatrix();
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	m_world->DrawDebugData();
	glPopMatrix();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	if( key == 'h' )
	{
		m_hide_gui = !m_hide_gui;
	}
	
	if(key == 's')
	{
		gui.saveToFile("settings.xml");
	}
	
	if(key == 'l')
	{
		gui.loadFromFile("settings.xml");
	}
	
	if(key == ' ')
	{
		color = ofColor(255);
	}
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key)
{
	std::cout << std::hex << static_cast<int>(key) << std::endl;
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y )
{
	
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button)
{
	
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button)
{
	
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button)
{
	
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
