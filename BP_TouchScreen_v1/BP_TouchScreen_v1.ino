//*******************************************
//  Touchscreen Reader for Ball on Plate
//  x,y,z coordinates are sent via i2c bus
//  i2c address 0x02
//
//  Hardware:  MKR1010 (but nearly any Arduino or ESP32 will work)

//  b walker, March 2021
//*******************************************


#include <Wire.h>
#include "TouchScreen.h"  // thank you for library, Lady Ada at Adafruit


#define YP A2  // must be an analog pin
#define XM A3  // must be an analog pin
#define YM 8   // digital pin
#define XP 9   // digital pin


// the usable limits of my touchscreen
// 12-bit ADC conversion

#define MINX 160
#define MAXX 915

#define MINY 296
#define MAXY 862

TouchScreen ts = TouchScreen(XP, YP, XM, YM, 800);

void setup(void) {
  Serial.begin(115200);

  // set up this MKR1010 as an i2c device
  Wire.begin(2);                // join i2c bus with address #2
  Wire.onRequest(requestEvent); // register event
}


uint16_t x;
uint16_t y;
uint16_t z;

unsigned long t = millis();
unsigned long deltat;
unsigned long count;

void loop(void) {
  // a point object holds x y and z coordinates
  TSPoint p = ts.getPoint();

  // check that pressure is within the readable range of the plate
  if ((p.x<MAXX && p.x>MINX) && (p.y<MAXY && p.y>MINY)){
    // pressure of 0 means no ball on surface of touchscreen!
    if (p.z > ts.pressureThreshhold) {

      // rescale and exponentially smooth x
      x=0.2*x+0.8*rescale(p.x, MINX, MAXX);

      // rescale and exponentially smooth y
      y=0.2*y+0.8*rescale(p.y, MINY, MAXY);
      
      z=p.z;
      
    } else {
      z = 0;
    }
  }
  
  delay(10);


  // leftover debug stuff
  count++;
  deltat = millis()-t;
  if (deltat>5000){
    //Serial.println(1000*count/double(deltat));
    count=0;
    t=millis();
  }
  
}


// request function that executes whenever data is requested by BP_PID_Servos_v1
// this function is registered as an event, see setup()
void requestEvent()
{
  Wire.write(byte(x/256)); // respond with message of 2 bytes
  Wire.write(byte(x%256));
  
  Wire.write(byte(y/256)); // respond with message of 2 bytes
  Wire.write(byte(y%256));
  
  Wire.write(byte(z/256)); // respond with message of 2 bytes
  Wire.write(byte(z%256));
}


// rescale so that touchscreen has xy(0,0) in lower left
// and xy(1000,1000) in upper right
// Notice that the direction is flipped for x and y axes
uint16_t rescale(uint16_t a, uint16_t mina, uint16_t maxa){

  // handle edge cases
  if (a<mina){
    return 1000;
  }
  
  if (a>maxa){
    return 0;
  }

  // beware, some sloppy math, here (mixing uints and floats)
  // rescale the usable area of the screen
  // This is done to make for simple screen coordinates.
  // xy(0,0) is lower left
  // xy(1000,1000) is upper right
  return 1000*(1-(float(a)-mina)/(maxa-mina));
  
}
