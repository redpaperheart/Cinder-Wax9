#include "cinder/app/AppNative.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/Utilities.h"
#include "cinder/gl/Fbo.h"

#include "Wax9.h"

using namespace ci;
using namespace ci::app;
using namespace std;

class Wax9SampleApp : public AppNative {
  public:
	void setup();
	void update();
	void draw();
    
    void keyDown(KeyEvent event);

    float mFlash;
    quat flatPosition;
    CameraPersp mCam;
    Wax9 mWax9;
};

void Wax9SampleApp::setup()
{
    mFlash = 0.0f;
    flatPosition = angleAxis(toRadians(90.0f), vec3(0, 0, 1));// *
//                   angleAxis(toRadians(180.0f), vec3(0, 1, 0));
    
    // Initialize Wax3 receiver with its port name
    // (type this in terminal to find the connected devices: ls /dev/tty.*)
    try {
        mWax9.setup("WAX9");
        mWax9.setDebug(false );
        mWax9.start();
        mWax9.resetOrientation(flatPosition);
    }
    catch (Exception e) {
    }
    
    mCam = CameraPersp(getWindowWidth(), getWindowHeight(), 45.0);
    mCam.setEyePoint(vec3(0, 0, 100));
    mCam.setCenterOfInterestPoint(vec3(0));
}

void Wax9SampleApp::update()
{
    // Update the receiver to get the data
    // (we only need this if we tell it to not be threaded)
    mWax9.update();
    
    // Process new data to detect spikes
    if (mWax9.hasReadings() && mFlash == 0 &&
        mWax9.getAccelerationLength() > 5.0) {
        mFlash = 1.0;
    }
    
    if(mFlash > 0.0f) mFlash -= 0.01f;
    else mFlash = 0.0f;
}

void Wax9SampleApp::draw()
{
	gl::clear(Color::gray(mFlash));
    gl::enableAlphaBlending();

    gl::drawString(to_string((int)getAverageFps()), vec2(20, 20));
    
    if (mWax9.isConnected() && mWax9.getReadings()->size() > 0) {
    
        // graph acceleration history per axis
        float scaleX = 6.0f;
        float scaleY = 10.0f;
        Shape2d red;
        Shape2d green;
        Shape2d blue;
        
        red.moveTo(0, 125 - mWax9.getReading().acc.x * scaleY);
        green.moveTo(0, 250 - mWax9.getReading().acc.y * scaleY);
        blue.moveTo(0, 375 - mWax9.getReading().acc.z * scaleY);

        for (int i = 1; i < mWax9.getNumReadings(); i++) {
            Wax9Sample sample = mWax9.getReading(i);
            red.lineTo(i * scaleX, 125 - sample.acc.x * scaleY);
            green.lineTo(i * scaleX, 250 - sample.acc.y * scaleY);
            blue.lineTo(i * scaleX, 375 - sample.acc.z * scaleY);
        }

        gl::color(1, 0, 0);
        gl::draw(red);
        gl::color(0, 1, 0);
        gl::draw(green);
        gl::color(0, 0, 1);
        gl::draw(blue);
        
        gl::enableDepthRead();
        gl::enableDepthWrite();
        
        gl::pushMatrices();{
            gl::setMatrices(mCam);
            gl::rotate(mWax9.getOrientation());
            gl::drawColorCube(vec3(0.0f), vec3(15, 30, 5));
//            gl::drawStringCentered("TEST", vec2(0), ColorA::white(), Font("Arial", 14));
            gl::drawCoordinateFrame(25.0, 2.0, 1.0);
            
        }gl::popMatrices();
        
        gl::disableDepthRead();
        gl::disableDepthWrite();
    }
    else {
        gl::drawStringCentered("Wax9 not found. Check Bluetooth pairing and port name", getWindowCenter());
    }
}

void Wax9SampleApp::keyDown(KeyEvent event)
{
    if (event.getChar() == ' ') {
        mWax9.resetOrientation(flatPosition);        // calibration
    }
}


CINDER_APP_NATIVE( Wax9SampleApp, RendererGl )
