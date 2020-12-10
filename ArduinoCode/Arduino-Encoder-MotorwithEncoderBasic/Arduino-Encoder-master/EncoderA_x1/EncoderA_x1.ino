int pulses;                              //Output pulses.
int deg = 0;
int encoderA = 3;
int encoderB = 2;
const int pwm = 5;                      //Power of motor.
const int dir = 4;                       //Direction of the motor.
int pulsesChanged = 0;
#define total 245                        //x1 pulses per rotation.
#define motorSpeed 180 
//Change speed of the motor.
void setup(){
  Serial.begin(115200);
  Serial.println("ready");
  pinMode(encoderA, INPUT);
  pinMode(encoderB, INPUT);
 // pinMode(pwm, OUTPUT);
 // pinMode(dir, OUTPUT);  
 // analogWrite(pwm,0);
 // digitalWrite(dir,HIGH);
  //waitAnyKeyPress();
  //attachInterrupt(0, A_CHANGE, CHANGE);
  attachInterrupt(digitalPinToInterrupt(2), A_CHANGE,CHANGE);


}//setup

void loop(){
// if (pulses == total) {
//    analogWrite(pwm, 0);
//    digitalWrite(dir, HIGH);
//  }
//  else {
//    if (pulses > total) {
//      analogWrite(pwm, motorSpeed);
//      digitalWrite(dir, LOW);
//
//
////    }
//    else if (pulses < total) {
//      analogWrite(pwm, motorSpeed);
//      digitalWrite(dir, HIGH);
//    }
//  }
//  
  if (pulsesChanged != 0) {
    pulsesChanged = 0;
    Serial.println(pulses);
  }
}

void A_CHANGE(){                                  //Function that to read the pulses in x1.
  if( digitalRead(encoderB) == 0 ) {
    if ( digitalRead(encoderA) == 0 ) {
      // A fell, B is low
      pulses--; // moving reverse
    } else {
      // A rose, B is low
      pulses++; // moving forward
    }
  }
  pulsesChanged = 1;
 
}
