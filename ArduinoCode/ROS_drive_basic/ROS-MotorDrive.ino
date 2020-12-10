#include <ros.h>
#include "Arduino.h"
#include <std_msgs/Float32.h>

ros::NodeHandle nh;


#define LED_BUILTIN 13


#define L_MOTOR_PWM 13
#define L_MOTOR_DIR1 11
#define L_MOTOR_DIR2 12
#define R_MOTOR_PWM 8
#define R_MOTOR_DIR1 9
#define R_MOTOR_DIR2 10

void turnWheel( const std_msgs::Float32 &wheel_power,
                unsigned int pwm_pin,
                unsigned int dir1_pin,
                unsigned int dir2_pin) {
    float factor = max(min(wheel_power.data, 1.0f), -1.0f);
    Serial.print(wheel_power.data);
    if( factor >= 0 ) {
        digitalWrite(dir1_pin, LOW);
        digitalWrite(dir2_pin, HIGH);  
        analogWrite(pwm_pin, (unsigned int)(255 * factor));
    } else {
        digitalWrite(dir1_pin, HIGH);
        digitalWrite(dir2_pin, LOW);
        analogWrite(pwm_pin, (unsigned int)(-255 * (1.0f * factor)));
    }   
}

void rightWheelCb( const std_msgs::Float32 &wheel_power ) {
    //nh.loginfo("Wheel Power - Right");
    //char result[8];
    //dtostrf(wheel_power.data, 6, 2, result); 
    //nh.loginfo(result);
    turnWheel( wheel_power, R_MOTOR_PWM, R_MOTOR_DIR1, R_MOTOR_DIR2 );
}
void leftWheelCb( const std_msgs::Float32 &wheel_power ) {
    //nh.loginfo("Wheel Power - Left");
    turnWheel( wheel_power, L_MOTOR_PWM, L_MOTOR_DIR1, L_MOTOR_DIR2 );
}

ros::Subscriber<std_msgs::Float32> sub_right("wheel_power_right",
                                            &rightWheelCb );
ros::Subscriber<std_msgs::Float32> sub_left("wheel_power_left",
                                           &leftWheelCb );


void setup()
{
  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  pinMode(L_MOTOR_PWM, OUTPUT);
  pinMode(L_MOTOR_DIR1, OUTPUT);
  pinMode(L_MOTOR_DIR2, OUTPUT);
  
  pinMode(R_MOTOR_PWM, OUTPUT);
  pinMode(R_MOTOR_DIR1, OUTPUT);
  pinMode(R_MOTOR_DIR2, OUTPUT);
  
  // Init motors to stop
  digitalWrite(L_MOTOR_DIR1, LOW);
  digitalWrite(L_MOTOR_DIR2, LOW);
  digitalWrite(R_MOTOR_DIR1, LOW);
  digitalWrite(R_MOTOR_DIR2, LOW);
  analogWrite(L_MOTOR_PWM, 0);
  analogWrite(R_MOTOR_PWM, 0);
  
  //ROS setup
  nh.initNode();
  Serial.begin(57600);
  nh.subscribe(sub_right);
  nh.subscribe(sub_left);
}

void loop()
{
  
  //nh.loginfo("Log Me");
  nh.spinOnce();
  
  // wait for a second
  //delay(1000);
}
