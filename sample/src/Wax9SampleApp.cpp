#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"

#include "Wax9.h"

// To test this sample place the sensor
// with the arrow pointing up and looking at you
// and hit space to calibrate

using namespace ci;
using namespace ci::app;
using namespace std;

class Wax9SampleApp : public App {
public:
    void setup();
    void update();
    void draw();
    void keyDown(KeyEvent event);
    
    void drawGraph();
    void drawOrientation();
    
    float mFlash;
    quat mStartRotationAHRS;
    CameraPersp mCam;
    
    Wax9 mWax9;
    
    // test
    deque<vec3> mMag;
};

void Wax9SampleApp::setup()
{
    mFlash = 0.0f;
    
    // Let's define the starting position of the sensor.
    // The zero rotation is the sensor flat, with the serial number up
    // so you can read it. On the other side, the arrow should point to the left.
    //
    // We want to define a zero rotation different than that. So look at the
    // sensor coordinate system from the readme and write down the rotations
    // you need in order to get to the starting rotation you want.
    // In this case the starting position will be the one with the arrow pointing up
    // and looking at us. So:
    
    mat4 startRotMat;
    startRotMat *= glm::rotate(toRadians(-90.0f), vec3(1, 0, 0));  // x
    startRotMat *= glm::rotate(toRadians(-90.0f), vec3(0, 1, 0));  // y
    startRotMat *= glm::rotate(0.0f, vec3(0, 0, 1));               // z
    
    mStartRotationAHRS = quat(startRotMat);  // save it in a quaternion
    
    // Initialize Wax9 with its port name
    try {
#ifdef CINDER_MSW
		// for windows you'll need to use the COM name of the serial port
		// which you can find in the device manager
		// (see this bug https://github.com/cinder/Cinder/issues/1064)
		mWax9.setup("COM7");
#else
		// for osx you can just use a part of the device name and it will be found
		// (type this in terminal to find the connected devices: ls /dev/tty.*)
		mWax9.setup("2B");
#endif
        mWax9.setDebug(false );
        mWax9.start();
        mWax9.resetOrientation(mStartRotationAHRS);
    }
    catch (Exception e) {
    }
    
    // setup camera
    mCam.setPerspective(45.0f, getWindowAspectRatio(), 0.1f, 1000.0f);
    mCam.lookAt(vec3(0, 0, 100), vec3(0));
}

void Wax9SampleApp::update()
{
    // Update the receiver to get the data
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
    
    gl::drawString(to_string((int)getAverageFps()), vec2(20, 20));
    
    if (mWax9.isConnected() && mWax9.hasReadings()) {
        drawGraph();
        drawOrientation();
        
        {
            // test - draw magnetometer data
            mMag.push_back(mWax9.getReading().mag);
            if (mMag.size() > 5000) mMag.pop_front();
            
            gl::ScopedColor c(1, 1, 1);
            gl::ScopedDepth depth(true);
            gl::ScopedMatrices cameraMatrices;
            gl::setMatrices(mCam);
            
            for (vec3 &v : mMag) {
                gl::drawCube(v * 0.25f, vec3(0.25));
//                gl::drawLine(vec3(0), v * 0.5f);
            }
        }
    }
    else {
        gl::drawStringCentered("Wax9 not found. Check Bluetooth pairing and port name", getWindowCenter());
    }
}

void Wax9SampleApp::drawGraph()
{
    // graph acceleration history per axis
    float scaleX = 6.0f;
    float scaleY = 10.0f;
    Shape2d aRed, aGreen, aBlue;
    Shape2d gRed, gGreen, gBlue;
    
    vec2 aStart(0, 125 - mWax9.getReading().acc.x * scaleY);
    aRed.moveTo(aStart);
    aGreen.moveTo(aStart);
    aBlue.moveTo(aStart);
    
    vec2 gStart(0, 375 - mWax9.getReading().acc.x * scaleY);
    gRed.moveTo(gStart);
    gGreen.moveTo(gStart);
    gBlue.moveTo(gStart);
    
    for (int i = 1; i < mWax9.getNumReadings(); i++) {
        Wax9Sample sample = mWax9.getReading(i);
        aRed.lineTo(i * scaleX, 125 - sample.acc.x * scaleY * 2);
        aGreen.lineTo(i * scaleX, 125 - sample.acc.y * scaleY * 2);
        aBlue.lineTo(i * scaleX, 125 - sample.acc.z * scaleY * 2);
        
        gRed.lineTo(i * scaleX, 375 - sample.gyr.x * scaleY);
        gGreen.lineTo(i * scaleX, 375 - sample.gyr.y * scaleY);
        gBlue.lineTo(i * scaleX, 375 - sample.gyr.z * scaleY);
    }
    
    gl::color(1, 0, 0);
    gl::draw(aRed);
    gl::draw(gRed);
    gl::color(0, 1, 0);
    gl::draw(aGreen);
    gl::draw(gGreen);
    gl::color(0, 0, 1);
    gl::draw(aBlue);
    gl::draw(gBlue);
    
    gl::drawString("Accelerometer", vec2(10, 100));
    gl::drawString("Gryoscope", vec2(10, 350));
}

void Wax9SampleApp::drawOrientation()
{
    if (mWax9.isConnected() && mWax9.hasReadings()) {

        quat sensorRotOGL = mWax9.getOrientation();
        
        gl::ScopedDepth depth(true);
        gl::ScopedMatrices cameraMatrices;
        gl::setMatrices(mCam);
        gl::rotate(sensorRotOGL);
        
        // draw sensor cube
        gl::drawColorCube(vec3(0.0f), vec3(30, 5, 15));
        gl::drawCoordinateFrame(25.0, 2.0, 1.0);    // frame coords
   
        // draw text and arrow
		gl::rotate( M_PI / 2.0f, 1.0f, 0.0f, 0.0f); //M_PI_2
        gl::scale(vec3(0.25, -0.25, 1.0));
        gl::translate(vec3(0, -8, 2.52));
        gl::drawStringCentered("◀︎Axivity", vec2(0, 0), Color::white(), Font("Arial", 24));
    }
}

void Wax9SampleApp::keyDown(KeyEvent event)
{
    if (event.getChar() == ' ') {
        mWax9.resetOrientation(mStartRotationAHRS);        // calibration
    }
}


CINDER_APP( Wax9SampleApp, RendererGl(RendererGl::Options().msaa(8)) )
