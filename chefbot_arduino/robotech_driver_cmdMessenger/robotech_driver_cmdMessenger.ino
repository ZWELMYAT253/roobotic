/* Author: Pyae Soan Aung (ROM ROBOTICS)*/

#define right_encoderA     2    
#define right_encoderB     3
#define left_encoderA      19   
#define left_encoderB      18

#define spray_pin          53
#define other_pin          51

#define ENA1 10
#define Right_in1 9
#define Right_in2 8

#define ENA2 5
#define Left_in1 6
#define Left_in2 7

#include <Wire.h>
#include "robot_specs.h"
#include <CmdMessenger.h>

int pump_state = 0;

int end_bit = 0;
char field_separator   = ',';
char command_separator = ' ';
char escape_separator  = '/n';
float frequency = 0;
enum
{
  Start,          // 0
  rpm_req,        // 1
  right_rpm_req,  // 2
  left_rpm_req,   // 3
  Rate,           // 4
  actual_rpm,      // 5
  imu             // 6
};

CmdMessenger cmdMessenger = CmdMessenger(Serial, field_separator, command_separator, escape_separator);

unsigned long lastMilli = 0;   

double desire_rpm_right = 0;
double actual_rpm_right = 0;

double desire_rpm_left = 0;
double actual_rpm_left = 0;

int right_pwm = 0; 
int left_pwm = 0;

volatile long right_count = 0;
volatile long left_count = 0;
long prev_right_count = 0;
long prev_left_count = 0;


void attachCommandCallbacks()
{
 
  cmdMessenger.attach(rpm_req, setRPM);
}

void setRPM()
{
  desire_rpm_right = cmdMessenger.readInt16Arg();
  desire_rpm_left = cmdMessenger.readInt16Arg();
  int spray = int( cmdMessenger.readInt16Arg() );
  int other = int( cmdMessenger.readInt16Arg() );
  if(spray >= 1) {pump_state =1;}
  if(other >= 1) {pump_state =0;}
}

void setup() {
  pinMode(spray_pin, OUTPUT);
  pinMode(other_pin, OUTPUT);
  
  setupMessenger();
  setupMotors();
  setupEncoders();
}
void loop() {
  unsigned long time_t = millis();
  if (time_t - lastMilli >= 100)   { 
    cmdMessenger.feedinSerialData();
    
    long delta_t = time_t - lastMilli;    
    getMotorData(delta_t);
    float delta_t_sec = delta_t / 1000.0;   
     
    right_pwm = updatePid(1, right_pwm, desire_rpm_right, actual_rpm_right);
    left_pwm = updatePid(2, left_pwm, desire_rpm_left, actual_rpm_left);

    if (right_pwm > 0 && left_pwm > 0) 
    {
      Forward();
    }
    else if (right_pwm < 0 && left_pwm < 0) 
    {
      Backward();
    }
    else if (right_pwm > 0 && left_pwm < 0) 
    {
      Right();
    }
    else if (right_pwm < 0 && left_pwm > 0) 
    {
      Left();
    }
    if (right_pwm == 0 && left_pwm == 0) 
    {
      Release();
    }  
    digitalWrite(spray_pin, pump_state);
    sendMessage(delta_t_sec);
    lastMilli = time_t;
  }
}
