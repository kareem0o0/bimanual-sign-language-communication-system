#include <Adafruit_PWMServoDriver.h>

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver board2 = Adafruit_PWMServoDriver(0x41);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
// Watch video V1 to understand the two lines below: http://youtu.be/y8X9X10Tn1k
#define SERVOMIN  125 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  575 // this is the 'maximum' pulse length count (out of 4096)




void setup() {
  Serial.begin(9600);
  Serial.println("16 channel Servo test!");

  board1.begin();
  board2.begin();  
  board1.setPWMFreq(50);  // Analog servos run at ~60 Hz updates
  board2.setPWMFreq(50);
  //yield();
}

// the code inside loop() has been updated by Robojax
void loop() {

  
  for (int angle = 0; angle < 181; angle += 10) {
    board1.setPWM(15, 0, angleToPulse(random(0, 180)));
    board1.setPWM(14, 0, angleToPulse(random(0, 180)));
    board1.setPWM(13, 0, angleToPulse(random(0, 180)));
    board1.setPWM(12, 0, angleToPulse(random(0, 180)));
    board1.setPWM(11, 0, angleToPulse(random(0, 180)));
    board1.setPWM(10, 0, angleToPulse(random(0, 180)));
    board1.setPWM(9, 0, angleToPulse(random(0, 180)));
    delay(200);
  }
  
// robojax PCA9865 16 channel Servo control
  delay(100);
 
}

/*
 * angleToPulse(int ang)
 * gets angle in degree and returns the pulse width
 * also prints the value on seial monitor
 * written by Ahmad Nejrabi for Robojax, Robojax.com
 */
int angleToPulse(int ang){
   int pulse = map(ang,0, 180, SERVOMIN,SERVOMAX);// map angle of 0 to 180 to Servo min and Servo max 
   Serial.print("Angle: ");Serial.print(ang);
   Serial.print(" pulse: ");Serial.println(pulse);
   return pulse;
}