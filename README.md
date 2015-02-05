Cinder-Wax9
========================

The [WAX9](http://axivity.com/v2/index.php?page=product.php&product=wax9) is a bluetooth 3-axis IMU built by [Axivity](http://axivity.com) that includes an accelerometer, a gyroscope, a magnetometer and a temperature sensor.

This Block provides an interface to obtain data from the sensor in real time from within a Cinder app, without the need to run any external software. It also provides a set of commonly needed functions to extract data from an IMU, which and can be extended or reused for other models.

It's tested only on Mac OS X and was built by [Adrià Navarro](http://adrianavarro.net) at [Red Paper Heart](http://redpaperheart.com).

Setting up the hardware
------------------------

0. Although it might no be necessary, it's better to download the [WAX9 Utils](http://axivity.com/v2/index.php?page=product.php&product=wax9) in a Windows machine and upgrade the firmware of the device first.

1. Charge de device and pair it in the Bluetooth settings panel. It should appear as WAX9-xxx and it won't need a password.

2. Connect to the device with [CoolTerm](http://freeware.the-meiers.org/) or another serial communication app.
    - The port name should include the word "WAX9". Baudrate is irrelevant for Bluetooth. Data bits: 8, Parity: none and stop bits: 1.
    - Type "settings" to see the current device configuration.
    - Read the developer guide for a detailed list of commands.

Basic usage
----------------

1. Clone the block into your blocks folder
2. Create a new project with Tinderbox, drag the src/ and include/ subfolders to your project or just duplicate the example.
3. Instantiate a Wax9 object for every device you have and call setup with the its port name. 
4. On every frame, call update() in all your devices.
5. Acces their data and process it with the getAcceleration(), getOrientation() and getAccelerationLength().

Advanced
--------
This block is based on the [Waxrec command line app](https://code.google.com/p/openmovement/source/browse/trunk/Software/WAX3/waxrec/waxrec.c) written in C by Axivity. Waxrec provides a lot more functionality, such as logging, UDP input, OSC output, etc, that hasn't been ported to the block. While this covers most of the general cases needed in a realtime Cinder application, for some situations you might find the need to use waxrec instead.

The IMU provides raw linear and angular acceleration. Obtaining the orientation from this data is not trivial. In this block I've used the [IMU and AHRS algorithm](http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/) open sourced by Sebastian Madgwick.

The WAX9 is also prepared to run as a BLE device (no pairing required). This block doesn't implement this functionality but you can find reference implementations [here](https://github.com/digitalinteraction/openmovement/tree/master/Software/WAX9).

Reading the developers guide is strongly encouraged to understand all the possible configurations of the WAX9.

License
-------
Copyright (c) 2015, Red Paper Heart, All rights reserved. To contact Red Paper Heart, email hello@redpaperheart.com or tweet @redpaperhearts

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
