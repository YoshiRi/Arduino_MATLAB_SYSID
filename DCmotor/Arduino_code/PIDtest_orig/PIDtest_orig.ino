#include <PID_v1.h>
double kp = 2; //proportional gain
double ki = 0; //integral gain
double kd = 0; //derivative gain
double N = 100; //derivative filter constant D(s)=s/(1+s/N)
//a good rule is: N>10*Kd/Kp (also avoid too large values)
double kb = 1; //back-calculation constant (README.md) to be

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, kp,ki,kd, DIRECT);

double curr = 80;

void setup() {
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255,255);
  Serial.begin(38400);
  Setpoint = 100;
}

int count = 0;

void loop() {
  Input = curr;
  if(myPID.Compute()){
    curr = curr+Output;    
  }

  Serial.println(millis());
  Serial.println(curr);
  Serial.println(Output);

}
