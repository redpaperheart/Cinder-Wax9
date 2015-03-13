/*
 Created by Adrià Navarro at Red Paper Heart
 
 Copyright (c) 2015, Red Paper Heart
 All rights reserved.
 
 This code is designed for use with the Cinder C++ library, http://libcinder.org
 
 To contact Red Paper Heart, email hello@redpaperheart.com or tweet @redpaperhearts
 
 Redistribution and use in source and binary forms, with or without modification, are permitted provided that
 the following conditions are met:
 
 * Redistributions of source code must retain the above copyright notice, this list of conditions and
 the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
 the following disclaimer in the documentation and/or other materials provided with the distribution.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
 */

#include "Wax9.h"
#include "cinder/Serial.h"

/* -------------------------------------------------------------------------------------------------- */
#pragma mark constructors and setup
/* -------------------------------------------------------------------------------------------------- */

Wax9::Wax9()
{
    // state
    bEnabled = true;
    bConnected = false;
    bDebug = false;
    bSmooth  = false;
    mSmoothFactor = 0.8;
    mNewReadings = 0;
    mHistoryLength = 120;
    mTimeout = 5.0f;
    mLastReadingTime = std::numeric_limits<float>::infinity();
    
    bBatteryLow = false;
    mBattery = 0xffff;
    mPressure = 0xfffffffful;
    mTemperature = 0xffff;
    
    // device settings
    bAccOn = true;
    bGyrOn = true;
    bMagOn = true;
    mOutputRate = 120;
    mAccRate = 200;
    mGyrRate = 200;
    mMagRate = 80;
    mAccRange = 8;
    mGyrRange = 2000;
    mDataMode = 1;
    
    mGyroDelta = vec3(0);
}

Wax9::~Wax9()
{
//    stop();
}

bool Wax9::setup(string portName, int historyLength)
{
    bConnected = false;
    mHistoryLength = historyLength;
    mSamples = new SampleBuffer(mHistoryLength);
    
    app::console() << "Available serial ports: " << std::endl;
    for( auto device : Serial::getDevices()) app::console() << device.getName() << ", " << device.getPath() << std::endl;
    
    try {
        Serial::Device device = Serial::findDeviceByNameContains(portName);
        mSerial = Serial(device, 115200);
        app::console() << "Receiver sucessfully connected to " << device.getName() << std::endl;
    }
    catch(SerialExc e) {
        app::console() << "Receiver unable to connect to " << portName << ": " << e.what() << std::endl;
        bConnected = false;
        return false;
    }
    
    AhrsInit(&mAhrs, 0, mOutputRate, 0.1f);
    
    bConnected = true;
    return true;
}

bool Wax9::start()
{
    if (bConnected) {

        // construct settings string - we're not using range, just leaving defaults
        std::string settings = "\r\n";
        settings += "RATE X 1 " + toString(mOutputRate) + "\r\n";                       // output rate in Hz (table 7 in dev guide)
        settings += "RATE A " + toString(bAccOn) + " " + toString(mAccRate) + "\r\n";   // accel rate in Hz (table 7)
        settings += "RATE G " + toString(bGyrOn) + " " + toString(mGyrRate) + "\r\n";   // gyro rate in Hz (table 7)
        settings += "RATE M " + toString(bMagOn) + " " + toString(mMagRate) + "\r\n";   // magnetometer rate Hz (table 7)
        settings += "DATAMODE " + toString(mDataMode) + "\r\n";                         // binary data mode (table 10)
        
        app::console() << settings;
        
        // send wait for reply from device
        mSerial.writeString(settings);
        app::console() << mSerial.readStringUntil('\n') << std::endl;
        
        // start streaming
        std::string init = "\r\nSTREAM\r\n";       // start streaming
        mSerial.writeString(init);
        
        return true;
    }
    return false;
}

/* Close input and thread */
bool Wax9::stop()
{
    // send termination string (this disconnects the device)
    if (bConnected) {
//        mSerial.writeString("\\r\nRESET\r\n");
//        app::console() << "Resetting and disconnecting WAX9" << std::endl;
    }
    bConnected = false;
    bEnabled = false;

    return true;
}

/* -------------------------------------------------------------------------------------------------- */
#pragma mark public interface
/* -------------------------------------------------------------------------------------------------- */

int Wax9::update()
{
    if (bConnected) {
        mNewReadings = readPackets(mBuffer);

        // If first run - not sure if this does anything
        if (mNewReadings > 0 && getNumReadings() == 0) {
            calculateOrientation(vec3(0), vec3(0), vec3(0), 0);
        }
        
        int numNewReadings = getNumNewReadings();
    
        // make sure we're not disconnected
        if (numNewReadings > 0)
            mLastReadingTime = app::getElapsedSeconds();
        else if (getNumReadings() > 0 && ((app::getElapsedSeconds() - mLastReadingTime) > mTimeout))
            bConnected = false;
        
        return numNewReadings;
    }
    return 0;
}

void Wax9::resetOrientation(quat q)
{
    float quat[4] = {q.w, q.x, q.y, q.z};
    AhrsReset(&mAhrs, quat);
}

/* -------------------------------------------------------------------------------------------------- */
#pragma mark input thread
/* -------------------------------------------------------------------------------------------------- */

int Wax9::readPackets(char* buffer)
{
    int packetsRead = 0;
    while(mSerial.getNumBytesAvailable() > 0)
    {
        // Read data
        size_t bytesRead = lineread(buffer, BUFFER_SIZE);
        
        if (bytesRead == (size_t) - 1)
        {
            bytesRead = slipread(buffer, BUFFER_SIZE);
        }
        if (bytesRead == 0) { return 0; }
        
        
        // Get time now
        unsigned long long now = ticksNow();
        
        // If it appears to be a binary WAX9 packet...
        if (bytesRead > 1 && buffer[0] == '9')
        {
            Wax9Packet *wax9Packet = parseWax9Packet(buffer, bytesRead, now);
            
            if (wax9Packet != NULL)
            {
                if(bDebug) printWax9(wax9Packet);
                
                // process packet and save it
                mSamples->push_front(processPacket(wax9Packet));  //todo: check if we should store packet pointers in the buffer
                packetsRead++;
            }
        }
    }
//    if (packetsRead > 0) app::console() << "packets read: " << packetsRead << std::endl;
    return packetsRead;
}

Wax9Sample Wax9::processPacket(Wax9Packet *p)
{
    Wax9Sample s;
    s.timestamp = p->timestamp;
    s.sampleNumber = p->sampleNumber;
    s.acc = vec3(p->accel.x, p->accel.y, p->accel.z) / 4096.0f;         // table 19 - in g
    s.gyr = vec3(p->gyro.x, p->gyro.y, p->gyro.z) * toRadians(0.07f);   // table 20 + convert deg/s to rad/s
    s.mag = vec3(p->mag.x, p->mag.y, -p->mag.z) * 0.1f;                 // in μT
    s.accLen = length(s.acc);
    s.rotAHRS = calculateOrientation(s.acc, s.gyr - mGyroDelta , s.mag, s.timestamp);
    s.rotOGL = AHRStoOpenGL(s.rotAHRS);
    
    // the sensor metadata only comes in every once in a while
    if (p->temperature != -1) {
        mTemperature = (float)p->temperature * 0.1f;
        if (bDebug) app::console() << "WAX9 - temperature: " << mTemperature << " celsius/n";
    }
    if (p->pressure != 0xfffffffful) {
        mPressure = p->pressure;
        if (bDebug)app::console() << "WAX9 - pressure: " << mPressure << " pascals/n";
    }
    if (p->battery != 0xffff){
        mBattery = p->battery;
        bBatteryLow = p->battery < 3500;   // according to dev guide, battery dies under 3300 mV
        if (bDebug)app::console() << "WAX9 - Battery: " << p->battery << " millivolts/n";
    }
    
    return s;
}

quat Wax9::calculateOrientation(const vec3 &acc, const vec3 &gyr, const vec3 &mag, uint32_t timestamp)
{
    // set sample frequency for AHRS algorithm
//    if (!mSamples->empty()) {
//        // compare timestamp between previous sample and this one
//        uint32_t prevTimestamp = mSamples->front().timestamp;
//        uint32_t diff = timestamp - prevTimestamp;
//        float diffSeconds = (float) diff / 65536.0f; // timestamps are in 1/65536 of a second
//        mAhrs.sampleFreq = 1.0f / diffSeconds;
//    }
//    else mAhrs.sampleFreq = mOutputRate;
    
    // Call AHRS algorithm update
    // we're not using the accelerometer yet
    float gyro[3]   = {gyr.x, gyr.y, gyr.z};
    float accel[3]  = {acc.x, acc.y, acc.z};
    AhrsUpdate(&mAhrs, gyro, accel, NULL);
    
    return quat(mAhrs.q[0], mAhrs.q[1], mAhrs.q[2], mAhrs.q[3]);
}


/* -------------------------------------------------------------------------------------------------- */
#pragma mark packet parsing
/* -------------------------------------------------------------------------------------------------- */

#define SLIP_END     0xC0                   /* End of packet indicator */
#define SLIP_ESC     0xDB                   /* Escape character, next character will be a substitution */
#define SLIP_ESC_END 0xDC                   /* Escaped substitution for the END data byte */
#define SLIP_ESC_ESC 0xDD                   /* Escaped substitution for the ESC data byte */

/* Read a line from the device */
size_t Wax9::lineread(void *inBuffer, size_t len)
{
    unsigned char *p = (unsigned char *)inBuffer;
    unsigned char c;
    size_t bytesRead = 0;
    
    if (inBuffer == NULL) { return 0; }
    *p = '\0';
    
//    while(!bCloseThread)
    while(bEnabled)
    {
        c = '\0';
        
        try{
            c = mSerial.readByte();
        }
        catch(...) {
            return bytesRead;
        }
        
        if (c == SLIP_END) { // A SLIP_END means the reader should switch to slip reading.
            return (size_t)-1;
        }
        if (c == '\r' || c == '\n')
        {
            if (bytesRead) return bytesRead;
        }
        else
        {
            if (bytesRead < len - 1) {
                p[bytesRead++] = (char)c;
                p[bytesRead] = 0;
            }
        }
    }
    return 0;
}

/* Read a SLIP-encoded packet from the device */
size_t Wax9::slipread(void *inBuffer, size_t len)
{
    unsigned char *p = (unsigned char *)inBuffer;
    unsigned char c = '\0';
    size_t bytesRead = 0;
    
    if (inBuffer == NULL) return 0;
    
    //    while(!bCloseThread)
    while(bEnabled)    //not sure if this is going to give problems without threaded
    {
        c = '\0';
        
        try{
            c = mSerial.readByte();
        }
        catch(...) {
            return bytesRead;
        }
        switch (c)
        {
            case SLIP_END:
                if (bytesRead) return bytesRead;
                break;
                
            case SLIP_ESC:
                c = '\0';
                
                try{
                    c = mSerial.readByte();
                }
                catch(...) {
                    return bytesRead;
                }
                
                switch (c){
                    case SLIP_ESC_END:
                        c = SLIP_END;
                        break;
                    case SLIP_ESC_ESC:
                        c = SLIP_ESC;
                        break;
                    default:
                        fprintf(stderr, "<Unexpected escaped value: %02x>", c);
                        break;
                }
                
                /* ... fall through to default case with our replaced character ... */
            default:
                if (bytesRead < len) {
                    p[bytesRead++] = c;
                }
                break;
        }
    }
    return 0;
}

Wax9Packet* Wax9::parseWax9Packet(const void *inputBuffer, size_t len, unsigned long long now)
{
    const unsigned char *buffer = (const unsigned char *)inputBuffer;
    static Wax9Packet wax9Packet;
    
    if (buffer == NULL || len <= 0) { return 0; }
    
    if (buffer[0] != '9')
    {
        fprintf(stderr, "WARNING: Unrecognized packet -- ignoring.\n");
    }
    else if (len >= 20)
    {
        wax9Packet.packetType = buffer[0];
        wax9Packet.packetVersion = buffer[1];
        wax9Packet.sampleNumber = buffer[2] | ((unsigned short)buffer[3] << 8);
        wax9Packet.timestamp = buffer[4] | ((unsigned int)buffer[5] << 8) | ((unsigned int)buffer[6] << 16) | ((unsigned int)buffer[7] << 24);
        
        wax9Packet.accel.x = (short)((unsigned short)(buffer[ 8] | (((unsigned short)buffer[ 9]) << 8)));
        wax9Packet.accel.y = (short)((unsigned short)(buffer[10] | (((unsigned short)buffer[11]) << 8)));
        wax9Packet.accel.z = (short)((unsigned short)(buffer[12] | (((unsigned short)buffer[13]) << 8)));
        
        if (len >= 20)
        {
            wax9Packet.gyro.x  = (short)((unsigned short)(buffer[14] | (((unsigned short)buffer[15]) << 8)));
            wax9Packet.gyro.y  = (short)((unsigned short)(buffer[16] | (((unsigned short)buffer[17]) << 8)));
            wax9Packet.gyro.z  = (short)((unsigned short)(buffer[18] | (((unsigned short)buffer[19]) << 8)));
        }
        else
        {
            wax9Packet.gyro.x   = 0;
            wax9Packet.gyro.y   = 0;
            wax9Packet.gyro.z   = 0;
        }
        
        if (len >= 26)
        {
            wax9Packet.mag.x   = (short)((unsigned short)(buffer[20] | (((unsigned short)buffer[21]) << 8)));
            wax9Packet.mag.y   = (short)((unsigned short)(buffer[22] | (((unsigned short)buffer[23]) << 8)));
            wax9Packet.mag.z   = (short)((unsigned short)(buffer[24] | (((unsigned short)buffer[25]) << 8)));
        }
        else
        {
            wax9Packet.mag.x   = 0;
            wax9Packet.mag.y   = 0;
            wax9Packet.mag.z   = 0;
        }
        
        if (len >= 28)
        {
            wax9Packet.battery = (unsigned short)(buffer[26] | (((unsigned short)buffer[27]) << 8));
        }
        else
        {
            wax9Packet.battery = 0xffff;
        }
        
        if (len >= 30)
        {
            wax9Packet.temperature = (short)((unsigned short)(buffer[28] | (((unsigned short)buffer[29]) << 8)));
        }
        else
        {
            wax9Packet.temperature = 0xffff;
        }
        
        if (len >= 34)
        {
            wax9Packet.pressure = buffer[30] | ((unsigned int)buffer[31] << 8) | ((unsigned int)buffer[32] << 16) | ((unsigned int)buffer[33] << 24);
        }
        else
        {
            wax9Packet.pressure = 0xfffffffful;
        }
        
        return &wax9Packet;
    }
    else
    {
        fprintf(stderr, "WARNING: Unrecognized WAX9 packet -- ignoring.\n");
    }
    return NULL;
}

/* -------------------------------------------------------------------------------------------------- */
#pragma mark utils
/* -------------------------------------------------------------------------------------------------- */

void Wax9::printWax9(Wax9Packet *wax9Packet)
{
    printf( "\nWAX9\ntimestring:\t%s\ntimestamp:\t%f\npacket num:\t%u\naccel\t[%f %f %f]\ngyro\t[%f %f %f]\nmagnet\t[%f %f %f]\n",
            timestamp(wax9Packet->timestamp),
            wax9Packet->timestamp / 65536.0,
            wax9Packet->sampleNumber,
            wax9Packet->accel.x / 4096.0f, wax9Packet->accel.y / 4096.0f, wax9Packet->accel.z / 4096.0f,	// 'G' (9.81 m/s/s)
            wax9Packet->gyro.x * 0.07f,    wax9Packet->gyro.y * 0.07f,    wax9Packet->gyro.z * 0.07f,		// degrees/sec
            wax9Packet->mag.x * 0.10f, wax9Packet->mag.y * 0.10f, wax9Packet->mag.z * 0.10f * -1			// uT (magnetic field ranges between 25-65 uT)
            );
}

/* Returns the number of milliseconds since the epoch */
unsigned long long Wax9::ticksNow(void)
{
    struct timeb tp;
    ftime(&tp);
    return (unsigned long long)tp.time * 1000 + tp.millitm;
}

/* Returns a date/time string for the specific number of milliseconds since the epoch */
const char* Wax9::timestamp(unsigned long long ticks)
{
    static char output[] = "YYYY-MM-DD HH:MM:SS.fff";
    output[0] = '\0';
    
    struct tm *today;
    struct timeb tp = {0};
    tp.time = (time_t)(ticks / 1000);
    tp.millitm = (unsigned short)(ticks % 1000);
    tzset();
    today = localtime(&(tp.time));
    if (strlen(output) != 0) { strcat(output, ","); }
    sprintf(output + strlen(output), "%04d-%02d-%02d %02d:%02d:%02d.%03d", 1900 + today->tm_year, today->tm_mon + 1, today->tm_mday, today->tm_hour, today->tm_min, today->tm_sec, tp.millitm);
    
    return output;
}

// Gets the Euler angles in radians defined with the Aerospace sequence (psi, theta, phi).
// See Sebastian O.H. Madwick report "An efficient orientation filter for inertial
// and inertial/magnetic sensor arrays" Chapter 2 Quaternion representation

vec3 Wax9::QuaternionToEuler(const quat &q)
{
    return vec3( (float)atan2(2 * q.x * q.y - 2 * q.w * q.z, 2 * q.w*q.w + 2 * q.x * q.x - 1),      // psi
                -(float)asin(2 * q.x * q.z + 2 * q.w * q.y),                                        // theta
                 (float)atan2(2 * q.y * q.z - 2 * q.w * q.x, 2 * q.w * q.w + 2 * q.z * q.z - 1) );  // phi

}

// Conversion between coordinate systems
// order as in: http://www.varesano.net/blog/fabio/ahrs-sensor-fusion-orientation-filter-3d-graphical-rotating-cube

quat Wax9::AHRStoOpenGL(const quat &q)
{
    vec3 eul = QuaternionToEuler(q);
    
    mat4 sensorRotMat = glm::rotate(-eul.z, vec3(0, 0, 1));	// Z: phi (roll)
    sensorRotMat *= glm::rotate(-eul.y, vec3(1, 0, 0));        // X: theta (pitch)
    sensorRotMat *= glm::rotate(-eul.x, vec3(0, 1, 0));        // Y: psi (yaw)
    
    return quat(sensorRotMat);
}

