#include <ros.h>
#include <std_msgs/Int8.h>
#include <Servo.h>

ros::NodeHandle nh;
int SERVO_PIN = 9;
Servo servo;

void moveCallback(const std_msgs::Int8& move_msg) {
    int angle = 0;
    
    if (move_msg.data == 10) { // Move right
        angle += 20;
    } else if (move_msg.data == -10) { // Move left
        angle -= 20;
    }
    
    // Ensure the angle stays within valid limits (usually 0 to 180 degrees)
    angle = constrain(angle, 0, 180);
    
    servo.write(angle);
}

ros::Subscriber<std_msgs::Int8> sub_move("move", &moveCallback);

void setup() {
    servo.attach(SERVO_PIN); // Replace SERVO_PIN with the actual pin connected to the servo
    nh.initNode();
    nh.subscribe(sub_move);
}

void loop() {
    nh.spinOnce();
    delay(10);
}
