#include "ArduPID.h"

double kp = 5; //proportional gain
double ki = 0.1; //integral gain
double kd = 0; //derivative gain
double N = 100; //derivative filter constant D(s)=s/(1+s/N)
//a good rule is: N>10*Kd/Kp (also avoid too large values)
double kb = 1; //back-calculation constant (README.md) to be
//set only if PID_BC used
double U1 = 0; //output variable 1
double U2 = 0; //output variable 2
double U3 = 0; //output variable 3
uint32_t T = 8; //20ms => 50Hz cycle frequency
PID PID1(&U1, kp, ki, kd, N, T); //standard PID, no anti-windup
PID_IC PID2(&U2, kp, ki, kd, N, T);  //PID with integrator clamping anti-windup
PID_BC PID3(&U3, kp, ki, kd, N, T, kb); //PID with back-calculation anti-windup

long lasttime = 0;

double Ref = 0.10;
double curr = 0;

void setup() {
  PID2.SetSaturation(-1000, 1000); //sets lower and upper limit to the PID output;
  //advisable to use an an anti-windup PID if sat needed
  PID3.SetSaturation(-1000, 1000);
  Serial.begin(38400);
  lasttime = millis();
}

void loop() {

  long now = millis();
  
  double e1 = 0; //this is an example: the Compute function receives as input the error
  double e2 = Ref-curr;
  double e3 = 0;
  PID1.AutoCompute(e1); //if possible use the standard PID: this way you have more control on what is going on
  PID2.AutoCompute(e2); 
  PID3.AutoCompute(e3);
  curr = curr+U2*double(now-lasttime)/1000.0;
  Serial.println(millis());
  Serial.println(curr);
  Serial.println(U2);
  lasttime = now;
}
