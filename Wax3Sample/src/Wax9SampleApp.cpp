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

    float mFlash;
    Wax9 mWax9;
};

void Wax9SampleApp::setup()
{
    // Initialize Wax3 receiver with its port name
    // (type this in terminal to find the connected devices: ls /dev/tty.*)
    try {
        mWax9.setup("WAX9");
        mWax9.setDebug(true);
        mWax9.start();
    }
    catch (Exception e) {
    }
    
    mFlash = 0.0f;
}


void Wax9SampleApp::update()
{
    // Update the receiver to get the data
    // (we only need this if we tell it to not be threaded)
    mWax9.update();
    
//    // Process new data to detect spikes
//    int newReadings = mAccel.getNumNewReadings();
//    for (int i=0; i<newReadings; i++) {
//        float acc = mAccel.getAccelMagnitude(i);
//        if (acc > 15.0 && mFlash == 0.0f) mFlash = 1.0f;
//    }
    
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

        for (int i=1; i<mWax9.getHistoryLength(); i++) {
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
        
        // show pitch and roll extracted from acceleration
        gl::enableDepthRead();
        gl::enableDepthWrite();
        
        gl::pushMatrices();{
            gl::setMatrices(CameraPersp(getWindowWidth(), getWindowHeight(), 45.));
            gl::translate(getWindowCenter());
            
//            app::console() << "pitch: " << mWax9.getPitch() << " roll: " << mWax9.getRoll() << std::endl;
//            gl::rotate(mWax9.getPitch(), vec3(1, 0, 0));
//            gl::rotate(mWax9.getRoll(),  vec3(0, 0, 1));
            gl::rotate(mWax9.getOrientation());

            gl::drawColorCube(vec3(0.0f), vec3(150, 50, 80));
        }gl::popMatrices();
        
        gl::disableDepthRead();
        gl::disableDepthWrite();
    }
    else {
        gl::drawStringCentered("Wax9 not found. Check Bluetooth pairing and port name", getWindowCenter());
    }
}


CINDER_APP_NATIVE( Wax9SampleApp, RendererGl )
