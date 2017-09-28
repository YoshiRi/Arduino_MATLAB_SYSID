// Yoshi Ri @ Univ Tokyo
// Sep 27 2017
// Check speed control
// using ArduPID lib

#include "ArduPID.h"
#include <Encoder.h>
#include <Servo.h> 

// encoder count per rotation 
//#define COUNTS_PER_ROTATION 18934
#define COUNTS_PER_ROTATION 17952
#define WHEEL_DIAMETER 0.11
#define METERS_PER_COUNT 3.141592 * WHEEL_DIAMETER / COUNTS_PER_ROTATION
#define RAD_PER_COUNT 2.0*3.141592  / (double) COUNTS_PER_ROTATION

#define FREQ 0.5 // hz
#define STEP_TIME 1000/FREQ // 1sec
#define OMEGA 2.0*3.141592*FREQ 
#define SPEEDREF 0.025 // m/s
#define SAMPLING 10 //ms

Encoder myEnc(2, 3);

// Servo ........ 
Servo myservoA;

// initial values
int SteerAngle = 90;


// DC motor .......
const int 
PWM_B   = 11,
DIR_B   = 13,
BRAKE_B = 8,
SNS_B   = A1;

long oldPosition  = 0;
long newPosition, dest = 0;
long prevTime, currTime, cmdTime;
int outputPWM = 0;
long timecount=0;
double Omega,Input,Setpoint;

//Specify the links and initial tuning parameters
double Kp=440.0, Ki=5.0, Kd=0.0;
//double Kp=540.0, Ki=0.0, Kd=0.0;

double N = 100; //derivative filter constant D(s)=s/(1+s/N)
uint32_t ST = 8;
double OutputV = 0;
// Constructor
PID_IC myPID(&OutputV, Kp, Ki, Kd, N, ST);


void setup() {
  // Servo ......
  myservoA.attach(5);  //  Servo motor pin 5 
  myservoA.write(SteerAngle);
  delay(1000);
  
  // DC motor .......
  // Configure the B output
  pinMode(BRAKE_B, OUTPUT);  // Brake pin on channel B
  pinMode(DIR_B, OUTPUT);    // Direction pin on channel B
  cmdTime = millis();
  prevTime = millis();
  oldPosition = myEnc.read();
  Input = 0.0;
  Setpoint = 0.0;

  //turn the PID on
  myPID.SetSaturation(-5, 5);
  myPID.Reset();

  // avoid 0 devide
  delay(100);
  //Serial.begin(9600);
  Serial.begin(38400);
}

void loop() {

  // Get data
  newPosition = myEnc.read();
  currTime = millis();
  Input = -(double)(newPosition - oldPosition) / (currTime - prevTime) * 1000; // Velocity in counts/sec
  Omega = (double)Input * RAD_PER_COUNT; // Rad/sec
  Input = Input * METERS_PER_COUNT; // Velocity in meters/sec
  //if (Input > 0.5) Input = 0.5; // HotFix : Limit in case of buffer overflow. Bug
  timecount = timecount+ currTime-prevTime;
  prevTime = currTime;
  oldPosition = newPosition;

// ----------------------  Signal Generator ---------------------- //
  // for Square wave
  if(timecount < STEP_TIME ){
    Setpoint = SPEEDREF;
  }else if(timecount < 2 * STEP_TIME ){
    Setpoint = -SPEEDREF;
    myPID.Reset();
  }else{
    Setpoint = SPEEDREF;
    timecount = timecount- 2*STEP_TIME;
    myPID.Reset();
  }

  // Get error from setpoint and current value  
  double error = Setpoint - Input;
  if(myPID.AutoCompute(error)){
    outputPWM = (long)(OutputV*255.0/5.0);
  }else{
    Serial.println("PID Error!!");
    delay(10);
    exit(0);
  }
// -------------------- End of Signal Generator ---------------------- //

  // Use arduino serial monitor to get these data
  double pose = (double)newPosition * RAD_PER_COUNT;
  //Serial.print("Curr Time = ");
  Serial.println(currTime);
  //Serial.print(",");
  //Serial.print("Curr Vel = ");
  Serial.println(Input,3);
  //Serial.print(",");
  //Serial.print("Curr Pos = ");
  Serial.println(Setpoint,3);
  //Serial.print(",");
  //Serial.print("Curr Input = ");
  Serial.println(OutputV,1);
  //Serial.println(timecount);

  
  if (outputPWM > 0)  {
    digitalWrite(DIR_B, HIGH);   // setting direction to HIGH the motor will spin forward
  } else  {
    digitalWrite(DIR_B, LOW);
    outputPWM = -outputPWM;
  }
  //  ===============  Switch ==================== // 
  // comment out the line below to run the robot 
  outputPWM = 0;
  analogWrite(PWM_B, outputPWM);     // Set the speed of the motor, 255 is the maximum value
  myservoA.write(SteerAngle);
}
