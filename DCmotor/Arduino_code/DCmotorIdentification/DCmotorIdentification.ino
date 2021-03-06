// Yoshi Ri @ Univ Tokyo
// Sep 27 2017
// output sin voltage pwm signal and get data 
// confirm your arduino port and install header files  below

#include <Encoder.h>
#include <Servo.h> 

// encoder count per rotation 
//#define COUNTS_PER_ROTATION 18934
#define COUNTS_PER_ROTATION 17952
#define WHEEL_DIAMETER 0.11
#define METERS_PER_COUNT 3.141592 * WHEEL_DIAMETER / COUNTS_PER_ROTATION
#define RAD_PER_COUNT 2.0*3.141592  / (double) COUNTS_PER_ROTATION

#define STEP_TIME 1000 // 1sec
#define FREQ 5 // hz
#define OMEGA 2.0*3.141592*FREQ 

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
long outputPWM = 0;
long timecount=0;
double Omega,Input;

void setup() {
  // Servo Motor for steering
  myservoA.attach(5);  //  Servo motor pin 5 
  myservoA.write(SteerAngle);
  delay(1000);
  
  // DC motor for driving
  // Configure the B output
  pinMode(BRAKE_B, OUTPUT);  // Brake pin on channel B
  pinMode(DIR_B, OUTPUT);    // Direction pin on channel B
  cmdTime = millis();
  prevTime = millis();
  oldPosition = myEnc.read();
  Input = 0.0;
  
  //Serial.begin(9600);
  Serial.begin(38400);
}

void loop() {

  myservoA.write(SteerAngle);

  // Get data
  newPosition = myEnc.read();
  currTime = millis();
  Input = -(double)(newPosition - oldPosition) / (currTime - prevTime) * 1000; // Velocity in counts/sec
  Omega = (double)Input * RAD_PER_COUNT; // Rad/sec
  Input = Input * METERS_PER_COUNT; // Velocity in meters/sec
  if (Input > 0.5) Input = 0.5; // HotFix : Limit in case of buffer overflow. Bug
  timecount = timecount+ currTime-prevTime;
  prevTime = currTime;
  oldPosition = newPosition;

// ----------------------  Signal Generator ---------------------- //
  // for Square wave
  if(timecount < STEP_TIME ){
    outputPWM = 255;
  }else if(timecount < 2 * STEP_TIME ){
    outputPWM = -255;
  }else{
    outputPWM = 255;
    timecount = timecount- 2*STEP_TIME;
  }

  // Sin wave
  outputPWM = 255 * sin(OMEGA*float(timecount)/1000.0);

// -------------------- End of Signal Generator ---------------------- //


  // Use arduino serial monitor to get these data
  double pose = (double)newPosition * RAD_PER_COUNT;
  //Serial.print("Curr Time = ");
  Serial.println(currTime);
  //Serial.print(",");
  //Serial.print("Curr Vel = ");
  Serial.println(Omega);
  //Serial.print(",");
  //Serial.print("Curr Pos = ");
  Serial.println(pose);
  //Serial.print(",");
  //Serial.print("Curr Input = ");
  Serial.println(outputPWM);
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

  // choose sampling frequency? 1000hz
  delay(1);
}
