#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int16.h>

ros::NodeHandle nh;
std_msgs::Int16 input;
//ros::Publisher chatter("chatter1", &input);

Servo myservo;  // create servo object to control a servo

void arduino_sub(const std_msgs::Int16& msg) {
  int angle = msg.data;

  // Ensure the input is within valid servo angle range (0 to 180)
  if (angle >= 0 && angle <= 360) {
    moveServo(angle);
  } else {
    // Handle invalid input if needed
  }
}

ros::Subscriber<std_msgs::Int16> sub("chatter1", &arduino_sub);

void setup() {
  myservo.attach(9);  
  nh.initNode();
  //nh.advertise(chatter);
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  delay(10);
}

void moveServo(int angle) {
  myservo.write(angle);  // move the servo to the specified angle
  delay(15);             // optional delay to make the movement smoother
}
