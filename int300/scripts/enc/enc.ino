#include <Wire.h>
#define SLAVE_ADDR 8
#define ANSWERSIZE 8


//initializing all the variables
#define LOOPTIME                      50     //Looptime in millisecond
int noCommLoopMax = 100;                //number of main loops the robot will execute without communication before stopping
unsigned int noCommLoops = 0;                 //main loop without communication counter

//Adafruit_MCP4725 dac_Left;

//Adafruit_MCP4725 dac_Right;

union
{
  long m[2];
  byte byteArray[8];  //little endian
} data;

double speed_cmd_left2 = 0;

const int PIN_ENCOD_A_MOTOR_LEFT = 2;               //A channel for encoder of left motor
const int PIN_ENCOD_B_MOTOR_LEFT = 4;               //B channel for encoder of left motor

const int PIN_ENCOD_A_MOTOR_RIGHT = 3;              //A channel for encoder of right motor
const int PIN_ENCOD_B_MOTOR_RIGHT = 5;              //B channel for encoder of right motor

const int PIN_MOTOR_LEFT_SPEED = 8;               //A channel for encoder of left motor
const int PIN_MOTOR_LEFT_DIR = 10;               //B channel for encoder of left motor
const int PIN_MOTOR_ENABLE = 12;               //B channel for encoder of left motor


const int PIN_MOTOR_RIGHT_SPEED = 9;              //A channel for encoder of right motor
const int PIN_MOTOR_RIGHT_DIR = 11;              //B channel for encoder of right motor
const int PIN_MOTOR_BRAKE = 13;               //B channel for encoder of left motor


const int PIN_SIDE_LIGHT_LED = 46;                  //Side light blinking led pin

unsigned long lastMilli = 0;

//--- Robot-specific constants ---
const double radius = 0.0625;                   //Wheel radius, in m
const double wheelbase = 0.225;               //Wheelbase, in m
const double encoder_cpr = 7425;               //Encoder ticks or counts per rotation
const double speed_to_pwm_ratio = 0.00205;//0.00235;    //Ratio to convert speed (in m/s) to PWM value. It was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the slope of the linear function).
const double min_speed_cmd = 0.0082;          //(min_speed_cmd/speed_to_pwm_ratio) is the minimum command value needed for the motor to start moving. This value was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the constant of the linear function).

double speed_req = 0;                         //Desired linear speed for the robot, in m/s
double angular_speed_req = 0;                 //Desired angular speed for the robot, in rad/s

double speed_req_left = 0;                    //Desired speed for left wheel in m/s
double speed_act_left = 0;                    //Actual speed for left wheel in m/s
double speed_cmd_left = 0;                    //Command speed for left wheel in m/s

double speed_req_right = 0;                   //Desired speed for right wheel in m/s
double speed_act_right = 0;                   //Actual speed for right wheel in m/s
double speed_cmd_right = 0;   //Command speed for right wheel in m/s
double distance_to_target = 0;
double left_motor_position = 0;
double right_motor_position = 0;

volatile long pulse_output_ra_f = 0;
volatile long pulse_output_rb_r = 0;
volatile long pulse_output_la_f = 0;
volatile long pulse_output_lb_r = 0;
void right_motor_forward();
void left_motor_forward();
void right_motor_reverse();
void left_motor_reverse();

const double max_speed = 0.5;                 //Max speed in m/s

int PWM_leftMotor = 0;                     //PWM command for left motor
int PWM_rightMotor = 0;                    //PWM command for right motor

// PID Parameters
const double PID_left_param[] = { 0, 0, 0.1 }; //Respectively Kp, Ki and Kd for left motor PID
const double PID_right_param[] = { 0, 0, 0.1 }; //Respectively Kp, Ki and Kd for right motor PID

volatile float pos_left = 0;       //Left motor encoder position
volatile float pos_right = 0;      //Right motor encoder position

void receiveEvent(int howmany)
{
  while (0 < Wire.available())
  {
    byte x = Wire.read();
  }
  //Serial.println("Receive event");
}
uint32_t ct = 300000;
//int pt;
//_____o____________________

float speed_factor = 1;

void requestEvent()
{
  //Serial.println("Inside requestEvent");

  //ct++;
  data.m[0] = pulse_output_ra_f;;
  data.m[1] = pulse_output_la_f;;
  Wire.write(data.byteArray[0]);
  //ct++;
  Wire.write(data.byteArray[1]);
  //ct++;
  Wire.write(data.byteArray[2]);
  Wire.write(data.byteArray[3]);
  Wire.write(data.byteArray[4]);
  //ct++;
  Wire.write(data.byteArray[5]);
  //ct++;
  Wire.write(data.byteArray[6]);
  Wire.write(data.byteArray[7]);
    Wire.write(data.byteArray[8]);
  //ct++;
  /*
  Wire.write(data.byteArray[9]);
  //ct++;
  Wire.write(data.byteArray[10]);
  Wire.write(data.byteArray[11]);
  Wire.write(data.byteArray[12]);
  //ct++;
  Wire.write(data.byteArray[13]);
  //ct++;
  Wire.write(data.byteArray[14]);
  Wire.write(data.byteArray[15]);
  */
  //Serial.println(ct);
  //delay(100);
}

void right_motor_forward()
{
  if (digitalRead(20) == digitalRead(21))
  {
    pulse_output_ra_f--;
  }
  else {
    pulse_output_ra_f++;
  }
}
void left_motor_forward()
{
  if (digitalRead(19) == digitalRead(18))
  {
    pulse_output_la_f--;

  }
  else {
    pulse_output_la_f++;  
  }
}


//__________________________


void setup() {


 /* Wire.begin(8);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  */
  Serial.begin(9600);
  Serial2.begin(9600);
  pinMode(21, INPUT_PULLUP);
  pinMode(20, INPUT_PULLUP);
  pinMode(19, INPUT_PULLUP);
  pinMode(18, INPUT_PULLUP);
  //digitalWrite(23,HIGH);
  //digitalWrite(3,HIGH);
  //digitalWrite(19,HIGH);
  //digitalWrite(18,HIGH);
 
  attachInterrupt(digitalPinToInterrupt(20), right_motor_forward, RISING);
  attachInterrupt(digitalPinToInterrupt(19), left_motor_forward, RISING);
 

}

int last_millis_for_1sec = 0;
long prevDiff_forward = 0;
long prevDiff_reverse = 0;

int leftSpeed = 0;
int rightSpeed = 0;


void loop() {


  /*
  Serial.print("pulse_output_la_f: ");
  Serial.print(pulse_output_la_f);
  Serial.print("     pulse_output_ra_f: ");
  Serial.println(pulse_output_ra_f);
  */

//  leftSpeed = pulse_output_la_f - leftSpeed;
  Serial.print(" : ");
  Serial.print(pulse_output_la_f);
//
//  rightSpeed = pulse_output_ra_f - rightSpeed;
  Serial.print(" : ");
  Serial.print(pulse_output_ra_f);
  Serial.println(" : ");

  Serial2.print(" : ");
  Serial2.print(pulse_output_la_f);

  Serial2.print(" : ");
  Serial2.print(pulse_output_ra_f);
  Serial2.println("  ");
 

  //leftSpeed = pulse_output_la_f;
  //rightSpeed = pulse_output_ra_f;

  delay(30);
 
}
