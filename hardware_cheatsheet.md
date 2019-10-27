## Hardware

This cheat sheet will focus on different commonly used hardware devices. There are a lot of devices out there, so we won't be able to cover everything!

### Microcontrollers

Microcontrollers (MCUs) are the brain of our robot. They control the other devices, but don't tend to do much on their own. We'll look at two common microcontrollers (Arduino and mBed) as well as Raspberry Pi - which technically isn't a microcontroller.
A good robotics MCU will have a real-time operating system (RTOS), which means that the OS will _not_ schedule your tasks - they will run exactly how you've written them. This is especially important when it comes to reading rapidly changing sensors as interrupts can ensure that you never miss a reading.

#### Arduino

Arduino is pretty famous, so you've probably heard of it. It has lots of support and is programmed in C++. There are lots of fantastic tutorials pretty much everywhere, and there's a library (somewhere) for pretty much every component you want to hook up to it. It has fewer interrupt pins than mBed though, so bear that in mind.
Typically we end up using Arduino Nanos due to their size, but Unos and Megas can be good for different things too. Can interface with RPi via communication methods such as serial, i2c, Rosserial etc. (see below)

#### mBed

mBed is less well known than Arduino. It is also programmed in C++, with many different boards such as the ST-Nucleo series or the LPC series. There is an online compiler ([documentation here](https://os.mbed.com/handbook/mbed-Compiler)) which requires a free account to use. Alternatively, [gcc4mbed](https://github.com/adamgreen/gcc4mbed) allows for command line compiling via gcc.
mBed boards typically have more interrupt pins than Arduinos of similar sizes, so I personally prefer them, but there is _considerably_ less support for them, so be prepared to have to figure out everything out yourself. You can still usually find a library for whichever cool component you want to use.

#### Raspberry Pi

RPis are **not** microcontrollers. They do **not** have an RTOS. They are a full computer, capable of running Ubuntu, or Rasbian or other OSes. I have put them here as we tend to use them like MCUs, due to their portability and GPIO pins.
Useful libraries for GPIO stuff are [WiringPi](http://wiringpi.com/) (C++) and [RPi.GPIO](https://pypi.org/project/RPi.GPIO/) (Python).

Typically we'll use a RPi for high level control (ROS, computer vision, communication between several devices etc.), and use dedicated MCUs (above) for low-level realtime processing (e.g. controlling motors, reading encoders).

##### ROS on RPi

So ROS supposedly _can_ run on Raspbian, but it's a real pain to setup. I really recommend just installing Ubuntu, Ubuntu Core or Ubuntu Mate and installing ROS on that.

### Motor Types

There are lots of different types of motor out there! Hopefully this should help you pick which one is right for your robot.

#### Servos

Cheap servos (e.g. Tower Pro sg90s) can be bought for less than £10. They are great for position-controlled robots (e.g. robot arms), as they contain built-in position controllers and gearing. There are typically 3 wires - power, ground and PWM. Servos are usually controlled via a PWM pulse which controls the position. The [Wikipedia page](https://en.wikipedia.org/wiki/Servo_control) isn't too complicated!

Important characteristics:
 - Deadband width (amount the PWM signal needs to change by to move the servo) which corresponds to angular resolution. 
 - Voltage is the recommended voltage to drive the servos. They will probably break if you use a different one!
 - Torque is the amount of rotational force the servos can drive. Always check the units! Typical units are kg-cm. If you're worried about this, then bear in mind that it's the _maximum_ so don't forget to account for acceleration as well as just the weight of whatever it's lifting.

 Expensive servos often have built in controllers for, torque, velocity and/or position. An example would by Dynamixels. These may have positional limits but might be continuous rotation. They'll typically have proprietry communication protocols, so check the documentation ;) 
 Ultimately these are just DC motors with fancy controllers built in so you don't have to do the hard work, but they're pretty nifty.

#### DC Motors

These are the most common motors out there. Continuous rotation, usually controlled via PWM. That's about it really.

Important characteristics:
 - Stall torque - the maximum torque that the motor can provide before it stalls (at rated voltage)
 - Stall current - the current the motor will draw when stalling at the rated voltage
 - Rated voltage - this is usually the highest voltage the motor can safely run at. Running it at higher voltages will probably reduce the life of the motor, but that might not matter. Take the voltage too high at stall though, and the current draw might cause the motor to overheat and permanently break, so be wary of this before over driving your motors!
 - No load speed - The theoretical speed the motor will turn at when given the rated voltage with no load applied.

 All of these specs will already account for whatever gear ratio is already on the motor. If you have your own added to the end, don't forget to account for that!

 ##### Motor curves
 From the parameters above you can plot some nice curves, which let you easily see stuff like the efficiency at a particular loading, or the speed at a particular torque. Obviously these are max values when at 100% PWM - assume PWM is linear to find out what the results are at 50% PWM ;)

[This](https://medium.com/luosrobotics/how-to-read-a-dc-motors-datasheet-f70fa440452b) website has a decent tutorial about how to plot the curves.

##### BDCM

Brushless DC Motors are pretty similar to DC motors _really_. They are usually controlled pretty differently though and can take enormous voltages. Usually one will use ESCs or something like an O-drive to control them (see further down).

Important characteristics:
 - kv - this is a bit of a weird parameter, but usually means RPM per volt. It's not technically true, but if you care about that look it up yourself.

These don't tend to have many characteristics, but the others should be similar to DC motors.

#### Stepper Motors

These are the motors used in our 3D printers!. Steppers are motors that count individual steps (or microsteps), counting these allows accurate position control. These are _not_ good for drive motors, as they usually have mediocre speed and torque for their weight and price. Quite good for 3d-printer-esque things, but bear in mind that missed steps will not be noticed, so don't apply more torque than they can take!

Steppers can be controlled via a H-bridge, this depends on the number of wires they have though, [here](https://itp.nyu.edu/physcomp/labs/motors-and-transistors/lab-controlling-a-stepper-motor-with-an-h-bridge/) is an example for a 4 wire stepper. Alternatively dedicated stepper drivers can be used (e.g. [Polulu A4988](https://www.pololu.com/product/1182)).

Important Characteristics:
 - Holding Torque - the torque before the stepper begins to skip steps
 - Voltage - This is usually quite low. With a good stepper controller you can go up to 20x this, but with just a H-bridge try not to go too much above this, and the current chopping isn't there. 
 - Step Angle - the angle the motor turns per full step. Microstepping can often allow a motor to step smaller amounts than this, but it's basically the stepper resolution.
 - Current rating - This is the current the stepper can run at safely. When tuning a controller, you often need to set the current limit to slightly below this for safety.

 Note that steppers often get **hot** (in the region of 60-80 Celcius) when running - they're supposed to, but be careful and don't burn yourself!

#### Linear Actuators

These are really just one of the above motors with a transmission on them to convert the rotational actuation to linear actuation. There are a couple of different methods.

##### Lead Screws

Basically a massive threaded rod with a nut at the end. Turning the rod/nut while keeping the other stationary moves the nut up and down the rod. Try not to put an off-center load on this (i.e. use linear bearings and support rods to take this load), as it won't work very well if you do.

You need to find the distance per full rotation of the thread to calculate the linear velocity of the rod relative to the rotational velocity of the motor.

##### Ball Screws

These are basically expensive lead screws with no backlash. Slightly more efficient too.

##### Rack and Pinion

A pinion gear meshes with a rack (a straight rod with teeth). This has the same problems as gears (backlash) but is relatively simple to create. Trying to constrain this properly can be a challenge though, and make sure to use linear bearings :)

You can also get helical versions of these to go with helical gears, but if you care about this sort of thing do a Mech Eng degree. (Lol even we barely cover this).

### Motor Drivers

This is about generic DC motor drivers. Servos and Steppers are covered in their respective sections.

There are a couple of different options depending on which type of motor you're using.

#### H-bridges

These are the ones we use in Robotics 101 ;) Ultimately it's a fancy layout of transistors allowing you to turn on and off different outputs, allowing the motor to spin in different directions with different PWM signals. See [this](https://howtomechatronics.com/tutorials/arduino/arduino-dc-motor-control-tutorial-l298n-pwm-h-bridge/) for a more in depth explanation.

A good starting point for most motors you care about is the L298N h-bridge.

Keep in mind maximum current assumes you have a good enough heat-sink to dissipate the power. Remember that if you're running a motor at 15v 2A that's 30 watts, it's going to get hot!

#### ESCs

Electronic Speed Controllers are usually used to control BDCMs. Oddly enough they control the speed of the motor. Check the max current is greater than what you want to supply to the motor, but bear in mind that motors draw what they need, so if you stall the motor at a high voltage it may draw a LOT of current.

For super accurate speed control you'll probably still want encoders and to run your own PID loop for instance.

#### O-drives

O-drive was created by an Ex-ICRS committee member (Oskar Weigl). They're really, really good for the price (but are still pretty expensive!). ICRS might have some lying around if you ask, but equally we might not. The [website](https://odriverobotics.com/) is great and basically covers everything you need to know!

They can provide stupidly large amounts of peak current and have a custom USB interface, with some really cool software to interface with it. Fantastic for controlling motors via RPi for example.

### Encoders

Motor encoders are used to measure the position or speed of spinning shafts. There are two main types in usage terms and several different ways for them to work.

#### Absolute encoders

These encoders will measure the absolute position of the motor. A pattern of output pulses allows the MCU to read the exact angle of the shaft. By reading the rate of change of position, speed can be calculated. Note that only 1 revolution can be measured before the "position" returns to 0. Good for e.g. position control of a robot arm.

Important characteristics:
 - Max RPM - the highest velocity this can operate at safely. It's usually much higher than our wheels can spin
 - Resolution - the number of distinct positions the encoder can report. Divide 360 by this to find the angular resolution

Make sure to keep the datasheet nearby to work out how to read the position from the encoder.

#### Incremental/Relative encoders

These output a signal either high or low every time the shaft moves more than a certain amount. Some encoders may have 2 or more channels offset to allow for higher resolutions. [This](https://www.pc-control.co.uk/incremental_encoders.htm) is a good explanation of how to count the pulses. There are good libraries for quadrature (2 channel) encoders for both Arduino and mBed.

Important characteristics:
 - Max RPM - the same as absolute encoders
 - PPR - Pulses per revolution. Multiply this by 2 (high and low) and the number of channels to see the full number of state changes (rising and falling edges) that can be detected. This gives you the full resolution of the encoder. For example a 24 PPR quadrature encoder has 96 (24x2x2) state changes per revolution, corresponding to 3.75 degrees per state change.
 - Detent pulses/CPR - the number of clicks it makes per rotation. This doesn't really matter, but when you turn the encoder by hand you can feel or hear the steps. Not always the same as PPR.

#### Encoder methods

There are basically 4 ways that encoders can work. Optical, infrared, mechanical or magnetic.

##### Optical

These will have a disk with holes in it. A laser will get blocked every time a new hole is found, causing a sensor inside to detect the change and output a pulse.

##### Infrared

These have a disk with alternating black and white around the edge. Infrared sensors and LEDs detect the changing colour by detecting the change of infrared reflection/transmission.

##### Mechanical

Literally a switch gets flicked every now and then. These are the cheapest encoders but typically have low PPR.

##### Magnetic

A magnet spins on the end of a shaft, and a hall effect sensor detects the change of position. Often used in absolute encoders as the sensor can easily tell which hall effect sensor detected the magnet.

Alternatively, the magnet can have lots of poles and there is only 1 hall-effect sensor. Ultimately, buy the magnetic encoder and it will come with a working magnet to go with it.

### Laser Scanners

Phew, now we've finally made it to the exciting sensors! Laser scanners like the [hokuyo](https://www.robotshop.com/en/hokuyo-urg-04lx-ug01-scanning-laser-rangefinder.html) can be quite expensive, but they are fantastic for map localisation, surface scanning or SLAM. The basic idea is that a spinning disk shoots a laser at known angles and uses a Time-of-Flight (ToF) method to calculate distances. This data can then be processed how you like, e.g. with [PCL](http://pointclouds.org/). There are some fancy solid state sensors beginning to come out too.

The Hokuyo is the scanner we usually borrow from Prof. Yiannis Demiris, and there is a [ROS node](http://wiki.ros.org/urg_node) that will interface with the device and publish data via a `/laser_scan` topic.

The general idea is to take the data from the ROS [LaserScan message](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/LaserScan.html), discard any values greater than `range_max` or less than `range_min` and then calculate the x and y position of the data relative to the origin co-ordinate frame:

C++ Code
```cpp
// Standard C++ libraries
#include <cmath> // sin, cos
#include <vector> // std::vector

// Ros library so we can have the callback
#include <ros/ros.h>
// The laser scan message is in this header
#include <sensor_msgs/LaserScan.h>

// We're using Eigen for vectors.
// We're probably going to need to for some funky maths later anyway if
// this node process laser scan data.
#include <Eigen/Geometry> 

// Ros callback function
void laser_scan_cb(sensor_msgs::LaserScan::ConstPtr msg) {
    float angle = msg->angle_min;
    unsigned int scans = (msg->angle_max - msg->angle_min)
                       / msg->angle_increment;
    std::vector<Eigen::Vector3f> points;
    for(unsigned int i = 0; i < scans; i++) {
        float r = msg->ranges[i];
        if(msg->range_min <= r && r <= msg->range_max) {
            // Assume that x is forwards for the laser scanner
            points.push_back(Eigen::Vector3f(r*cos(angle),
                                             r*sin(angle),
                                             0));
        }
        // Even if we're ignoring the data we still need to 
        // increment the angle
        angle += msg->angle_increment;
    }

    // Now process the data, maybe use TF to convert to another coordinate
    // frame, or PCL to generate point cloud etc...
}
```

### Cameras

Ooh boy you want to do some computer vision? Then you're gonna need a camera. Specs are pretty easy to understand, just remember that a RPi can only connect to one [PiCam](https://projects.raspberrypi.org/en/projects/getting-started-with-picamera), but can have lots of [USB cameras](https://www.amazon.co.uk/gp/product/B01C2PIBB0/)!

In terms of funky things like un-distorting cameras, there's a [ROS Package](http://wiki.ros.org/camera_calibration) which can find your callibration parameters. Otherwise [OpenCV](https://opencv.org/) also has some [good stuff](https://docs.opencv.org/2.4/doc/tutorials/calib3d/camera_calibration/camera_calibration.html).

You'll probably want to use OpenCV with this, this cheat sheet is long enough as it is, but pro tip: use [HSV not RGB](https://docs.opencv.org/trunk/df/d9d/tutorial_py_colorspaces.html) ;)

### IMUs

Inertial Measurement Units are great for integrating with odometry in an EKF (more later). They contain an accelerometer, gyroscope and magnetometer usually, and some fancy ones will even do the sensor fusion on board. Ultimately this means you can measure your orientation and acceleration.

These guys are _noisy_ so don't try and integrate acceleration twice to get position or something. It won't work. Guaranteed.

Sparkfun has a good guide on [buying IMUs](https://www.sparkfun.com/pages/accel_gyro_guide).

## Communication Protocols

So you've got all these sensors and MCUs and other things, but how do you get them to talk to each other? This section will outline some existing protocols for talking to things at a **low** level. This does **not** cover networking stuff like TCP, UDP or IP. Go learn that yourself.

### Serial

This is a two-wire full-duplex communication method. That means that both sides can talk to each other at the same time, without interrupting each other (get it? Coz it often implements hardware interrupts? Eh? Eh? I'll see myself out).

Serial requires you to select a baud rate (bits per second) - common values are 9600, 19200, 57600 and 115200. The higher the faster. If the baud rate is mis-matched then data will be interpreted incorrectly. The nitty-gritty details can be found [here](https://learn.sparkfun.com/tutorials/serial-communication/all).

Arduino and mBeds can talk to computers via USB Serial communication. They can also talk to each other using this protocol. It's also great for debugging - you can print messages from your MCU to your PC to find out where it's crashing.

[Arduino Software Serial Tutorial](https://www.arduino.cc/en/tutorial/SoftwareSerialExample) (bad for high baud rates)
[Arduino Hardware Serial Tutorial](http://www.martyncurrey.com/arduino-serial-part-1/#Hardware-Serial)

[mBed Serial Tutorial](https://os.mbed.com/handbook/SerialPC)

The main problem with Serial is that it's a 1 to 1 communication protocol. It's no good if you need to talk to 3 different devices at the same time (or at different times, unless re-wiring in between is ok).

### I2C

I2C is a half-duplex two-wire protocol designed to allow one master device to communicate with lots of slave devices. There is a clock wire, which outputs a regular square wave and a data wire. The master will send an address, corresponding to a particular slave device. Then it will send commands to the slave (e.g. READ REGISTER 5, WRITE 0x13B4 TO REGISTER 2), which the slave will either do, or respond to. The master can then talk to a different slave. Typical baud rates are 100000 or 400000. Half-duplex means that only one device can talk at a time.

While I2C allows 1 to many communication, most devices have fixed addresses, and no two devices may have the same address for the protocol to work. If you have multiple devices with the same address, you'll need to use a multiplexer to open and close different channels - read its datasheet to see how it works, they're all different. To get you started the [TI PCA9548A](https://www.ti.com/lit/ds/symlink/pca9548a.pdf) is quite good, but don't get it confused with the [NXP PCA9548A](https://www.nxp.com/docs/en/data-sheet/PCA9548A.pdf?) which does the same thing with the same pinout but is controlled differently.

Arduino has an I2C library called [Wire](https://www.arduino.cc/en/reference/wire)
mBed has an I2C library called (unimaginatively) [I2C](https://os.mbed.com/handbook/I2C)

You can configure Arduinos or mBeds to be slaves instead if you want: [Arduino](https://www.arduino.cc/en/Tutorial/MasterWriter) [mBed](https://os.mbed.com/docs/mbed-os/v5.9/reference/i2cslave.html).

**Important**: I2C _needs_ pull-up resistors (usually about 10K is fine) on both wires. Also check that devices run at the same logic level - although 3v3 usually works with 5v devices as high on 3v3 is still high enough for 5v. If in doubt there are some converters out there.

### SPI

SPI is a four-wire full-duplex protocol. It has a clock (like I2C), a Master out/Slave in, Master in/Slave out and a slave select. Each slave is connected to a different slave select pin on the master, allowing the master to turn on communication with one slave at a time. After that communication proceeds in a similar manner to I2C (but without the addressing at the start), but full duplex.

I've not really used this protocol much, but the [Wikipedia page](https://en.wikipedia.org/wiki/Serial_Peripheral_Interface) explains it pretty well.
