#include <ros.h>
#include "Arduino.h"
#include <std_msgs/Float32.h>
#include <std_msgs/UInt32.h>

ros::NodeHandle nh;



#define LED_BUILTIN 13


#define L_MOTOR_PWM 10
#define L_MOTOR_DIR1 8
#define L_MOTOR_DIR2 9
#define R_MOTOR_PWM 5
#define R_MOTOR_DIR1 6
#define R_MOTOR_DIR2 7

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

int l_encoder_pin = 3; //Interrupt1
int r_encoder_pin = 2; //Interrupt0
unsigned int rpm_l=0;
unsigned int rpm_r=0;
volatile byte pulse_l=0;
volatile byte pulse_r=0;
unsigned long timeold=0;
unsigned int pulse_per_rev=20; //Pulse per revolustion of index disc
volatile unsigned long rdebounce=0;
volatile unsigned long ldebounce=0;
void counter_l()
{
  unsigned long m = micros();
  if(m-ldebounce > 1500){
    //Update Count
    pulse_l=pulse_l+1;
    }
  ldebounce = m;
}
void counter_r()
{
  unsigned long m = micros();
  if(m-rdebounce > 1500){
    //Update Count
    pulse_r=pulse_r+1;
    }
   rdebounce = m;
}

//std_msgs::UInt32 rpm_left_msg;
//std_msgs::UInt32 rpm_right_msg; 
//ros::Publisher rpm_left_pub("left_wheel_rpm", &rpm_left_msg);
//ros::Publisher rpm_right_pub("right_wheel_rpm", &rpm_right_msg);

std_msgs::UInt32 rticks_msg;
std_msgs::UInt32 lticks_msg;
ros::Publisher rticks_pub("tick_wheel_right", &rticks_msg);
ros::Publisher lticks_pub("tick_wheel_left", &lticks_msg);

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

  //Encoder setup
  pinMode(l_encoder_pin, INPUT);
  pinMode(r_encoder_pin, INPUT);

  //Triggers on FALLING (change from HIGH to LOW)
  attachInterrupt(digitalPinToInterrupt(2), counter_r, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), counter_l, CHANGE); 

  //Initialize Value
  pulse_l = 0;
  pulse_r = 0;
  timeold = 0;
  
  //ROS setup
  nh.initNode();
  Serial.begin(57600);
  nh.subscribe(sub_right);
  nh.subscribe(sub_left);

  //nh.advertise(rpm_left_pub);
  //nh.advertise(rpm_right_pub);
  nh.advertise(rticks_pub);
  nh.advertise(lticks_pub);
  delay(200);
}

void loop()
{
  if (millis() - timeold >= 1000){  /*Uptade every one second, this will be equal to reading frecuency (Hz).*/
  detachInterrupt(0);
  detachInterrupt(1);
 
   //send data to ros messages
 
   rticks_msg.data = pulse_r;
   lticks_msg.data = pulse_l;

   //Publish data
   //rpm_left_pub.publish(&rpm_left_msg);
   //rpm_right_pub.publish(&rpm_right_msg);
   rticks_pub.publish(&rticks_msg);
   lticks_pub.publish(&lticks_msg);

   //reset parameter
   timeold = millis();
   pulse_r = 0;
   pulse_l = 0;
    
   //Restart the interrupt processing
   attachInterrupt(0, counter_r, FALLING);
   attachInterrupt(1, counter_l, FALLING);
   }

  
   //nh.loginfo("Log Me");
   nh.spinOnce();
  
   // wait for a second
   delay(200);
}
