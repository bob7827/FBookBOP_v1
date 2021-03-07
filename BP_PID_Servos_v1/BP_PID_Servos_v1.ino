//*******************************************
//  PID and Servo controller for Ball-on-Plate
//
//  Hardware:  MKR1010 (but nearly any fast Arduino or ESP32 will work)
//  
//  b walker, March 2021
//*******************************************


// included for math stuff in some set point patterns
#include <math.h>


// i2c to read touchscreen controller (the other MKR1010)
#include <Wire.h>


// I'm using the Adafruit PCA9685 to drive the x and y servos
// Adafruit 16-Channel 12-bit PWM/Servo Driver - I2C interface
// This simplified things and doesn't tie up the Arduino with PWM
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// the PCA9685 is being used to drive servos with 20ms wavelength
#define SERVO_FREQ 50 // 50 Hz PWM freq


// PID library
// https://github.com/gelraen/Arduino-PID-Library
// by Brett Beauregard and Max Ignatenko
#include <PID_v2.h>

// instantiate and set initial x and y PID constants
// high I constant to help center the ball more quickly
// These PID constants will be changed after the ball is on the plate
// for a few seconds.  It just works better that way.
double Kpx = 0.20, Kix = 0.05, Kdx = 0.0815;
PID_v2 xPID(Kpx, Kix, Kdx, PID::Direct, PID::P_On::Error);

double Kpy = 0.112, Kiy = 0.05, Kdy = 0.0435;
PID_v2 yPID(Kpy, Kiy, Kdy, PID::Reverse, PID::P_On::Error);



// set the sceen center
// You'll need to tweak these numbers.  All screens are different.
// screen is scaled so that (0,0) is lower left and (1000, 1000) is upper right
double centerX = 489;  // start with 500 and adjust for your touchscreen
double setX = centerX; // initial x set point

double centerY = 452;  // start with 500 and adjust for your touchscreen
double setY = centerY; // initial y set point



void setup() {
  Serial.begin(115200);
  delay(100);  // on my host computer, I need to set a short delay, don't ask

  // set i2c clock for touchscreen and PWM devices
  Wire.begin();
  Wire.setClock(400000);

  // set up Adafruit PWM to drive the x and y servos
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(100);


  // xPID setup
  xPID.Start(centerX,  // input
             286,    // current output-- level servo angle, yours will be different
            centerX);   // setpoint
  xPID.SetMode(PID::Mode::Manual);  // start with PID off  
  xPID.SetOutputLimits(150.0, 400.0); // x servo output limits, will vary with mechanics
  xPID.SetSampleTime(45);  // 40-50 ms works well


  // yPID setup
  yPID.Start(centerY,  // input
             342,    // current output-- level servo angle, yours will be different
             centerY);   // setpoint
  yPID.SetMode(PID::Mode::Manual);  // start with PID off
  yPID.SetOutputLimits(205.0, 439.0); // y servo output limits, will vary with mechanics
  yPID.SetSampleTime(50); // 40-50 ms works well.

  delay(100);

}


// PID output  
double outputX;
double outputY;

// averaged touchscreen location
double ax=centerX;
double ay=centerY;

// loop and pattern control
bool firstTime=true;
long int count=0;

// touchscreen location and pressure
uint16_t x, y, z;


void loop() {

  // *** READ SCREEN
  Wire.requestFrom(2, 6); // request x,y,z screen data from other Arduino
  
  x=  256*Wire.read();
  x+= Wire.read();
  
  y=  256*Wire.read();
  y+= Wire.read();
  
  z=  256*Wire.read();
  z+= Wire.read();


  // Once the ball is placed on the plate, turn on PID.
  // After a short delay, readjust the PID values for better response.
  // This if-block only gets called once.
  if (firstTime && count>200){
    
    // readjust PID constants, as necessary
    xPID.SetTunings(0.20, 0.020, 0.079, PID::P_On::Error);
    yPID.SetTunings(0.14, 0.016, 0.0495, PID::P_On::Error);

    // set ball in the center
    xPID.Setpoint(centerX);
    yPID.Setpoint(centerY);

    // Ball set on touchscreen for first time, so fire up PIDs.
    yPID.SetMode(PID::Mode::Automatic);
    xPID.SetMode(PID::Mode::Automatic);

    firstTime=false;
  }


  // if z!=0, then ball is on the plate
  if (z!=0){
    count++;  // count gets used to sequence set points for patterns

    // diagnostic output, your choice
    // Serial.print(x-centerX);  Serial.print(",");
    // Serial.print(y-centerY);  Serial.println();

    
    // optional: additional exponential smoothing to dampen screen jitter
    // I found that additional smoothing helps reduce some jitter.
    ax = 0.2*ax + 0.8*double(x);
    ay = 0.2*ay + 0.8*double(y); 
       
  
    // *** UPDATE X PID CONTROLLER and UPDATE SERVO
    // x servo output is smoothed  
    outputX = 0.2*outputX + 0.8*xPID.Run(ax); 
    pwm.setPWM(0, 0, uint16_t(outputX));
    
    // *** UPDATE Y PID CONTROLLER and UPDATE SERVO
    // y servo output is smoothed
    outputY = 0.2*outputY + 0.8*yPID.Run(ay); 
    pwm.setPWM(1, 0, uint16_t(outputY));


    // pick a single pattern
    
    pattern_center(count);  // ball remains in the center, start here for tuning
    //pattern_rectangle(count);
    //pattern_zigzag(count);
    //pattern_ellipse(count);
    //pattern_figure8(count);

  }
  
  delay(7);  // 5 to 10 ms seems to work well

}


// ball remains in center of plate
void pattern_center(long int &count){
  xPID.Setpoint(centerX);
  yPID.Setpoint(centerY);
}


// ball makes a rectangular pattern
void pattern_rectangle(long int &count){
  if (count==2000){
    xPID.Setpoint(225);
    yPID.Setpoint(250);
  }

  if (count==2300){
    xPID.Setpoint(225);
    yPID.Setpoint(705);
  }

  if (count==2600){
    xPID.Setpoint(775);
    yPID.Setpoint(705);
  }

  if (count==2900){
    xPID.Setpoint(775);
    yPID.Setpoint(250);
  }

  // set points
  if (count==3200){
    xPID.Setpoint(225);
    yPID.Setpoint(250);
  }
  
  if (count==3500){
    xPID.Setpoint(775);
    yPID.Setpoint(250);
  }
  if (count==3800){
    xPID.Setpoint(775);
    yPID.Setpoint(705);
  }

  if (count==4000){
    xPID.Setpoint(225);
    yPID.Setpoint(705);
    count=1700;
  }
}


// ball makes a zig-zag pattern along the x-axis
void pattern_zigzag(long int &count){
  xPID.Setpoint(((count/600)%2)*500+250);
  yPID.Setpoint(((count/600)%9)*60+250);
}


// ball makes a counter clockwise ellipse
void pattern_ellipse(long int &count){
  float theta = 0.01256*count;  // one complete revolution per 500 count
  if (theta>2*PI){
    count=0;
    theta=0.0;
  }
  float x=250*cos(theta)+centerX;
  float y=250*sin(theta)+centerY;
  
  xPID.Setpoint(x);
  yPID.Setpoint(y);
}


// ball makes a figure-8, actually its an infinity symbol
void pattern_figure8(long int &count){
  float theta = 0.01256*count;  // one complete revolution per 500 count
  if (theta>2*PI){
    count=0;
    theta=0.0;
  }
  float x=250*cos(theta)+centerX;
  float y=250*sin(2.0*theta)+centerY;
  
  xPID.Setpoint(x);
  yPID.Setpoint(y);
}
