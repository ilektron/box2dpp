#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "Box2D/Box2D.h"
#include "ofxB2Draw.h"


constexpr int APP_WIDTH = 1024;
constexpr int APP_HEIGHT = 728;

class ofApp : public ofBaseApp
{
	
public:
	
	ofApp();
	virtual ~ofApp() = default;
	
	void setup();
	void update();
	void draw();
	
	void exit();

	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y );
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);		

	void circleResolutionChanged(int & circleResolution);
	void ringButtonPressed();

private:
	bool m_hide_gui;
	bool m_draw_box2d_debug;

	ofxFloatSlider radius;
	ofxColorSlider color;
	ofxVec2Slider center;
	ofxIntSlider circleResolution;
	ofxToggle filled;
	ofxButton twoCircles;
	ofxButton ringButton;
	ofxLabel screenSize;

	ofxPanel gui;
	
	std::shared_ptr<b2World> m_world;
	ofxB2Draw m_draw;
	
};

