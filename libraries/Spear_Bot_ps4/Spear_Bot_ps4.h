#ifndef Spear_Bot_ps4
#define Spear_Bot_ps4

//Libraries
#include <PS4BT.h>
#include <ZumoMotors.h>
#include <Pixy.h>
#include <math.h>
#include <SPI.h>

//Classes for PS4 controller, Zumo motors, and pixycam
ZumoMotors motors;
Pixy pixy;
USB Usb;
BTD Btd(&Usb);
PS4BT PS4(&Btd);

//Robot Enumerations
enum ROBOT_AUTONOMY  {SEMI_AUTONOMY_TELEOP, SEMI_AUTONOMY, SUPERVISED_AUTONOMY, FULL_AUTONOMY};
enum ROBOT_INTERFACE {NORMAL, JAMMED};
enum ROBOT_STATE {DESTROYED, DISARMED, ARMED, TARGET_SPOTTED, ENGAGING_TARGET};
enum RULES_OF_ENGAGEMENT {HOLD_FIRE, WEAPONS_HOLD, WEAPONS_TIGHT, WEAPONS_FREE};
enum CONTROLLER_INVERSION {NON_INVERTED = 1, INVERTED = -1};
enum SEARCH_MODES {PAN, PAN_CCW, PAN_CW, PAN_FW};
enum COLOR_SIGNATURES {UNUSED=0, FRIENDLY=1, HOSTILE=2, NEUTRAL=3, UNKNOWN=4, INCOMING_FIRE=5, JAMMER=6};
enum ROBOT_STATUS {INERT, LIVE};

//Gains for Pan/Tilt Controllers
int Kp_p = 100; //Proportional Gain (Pan)
int Kd_p = 150; //Derivative Gain (Pan)

int Kp_t = 150; //Proportional Gain (Tilt)
int Kd_t = 200; //Derivative Gain (Tilt)

//Gains for Motor Controller
float Kp_f = 0.3; //Proportional Gain (Motor) [0.5]
float Kd_f = 0.1; //Derivative Gain (Motor) [0.2]

//Parameters for Pixycam cameras and pan/tilt servos
#define X_CENTER    160  //Center Pixel for  Camera
#define Y_CENTER    100  //Center Pixel for Camera
#define RCS_PAN_CENTER_POS  500 //Pan Servo Center Position - Should be aligned with robot's spear (Verify in Pixymon for your robot)
#define RCS_TILT_CENTER_POS 500 //Tilt Servo Center Position - Should be aligned with robot's spear (Verify in Pixymon for your robot)
#define RCS_MIN_POS     0     // Minimum Servo Position (Verify in Pixymon for your robot)
#define RCS_MAX_POS     1000  // Maximum Servo Position (Verify in Pixymon for your robot)

float tilt_upper_limit = 500; // controls how high the camera looks (Range 0-1000)
float tilt_lower_limit = 500; // controls how low the camera looks (Range 0-1000)

int pixy_pan = PIXY_RCS_CENTER_POS; //Current Position of Pixycam Pan servo
int pixy_tilt = PIXY_RCS_CENTER_POS; //Current Position of Pixycam Tilt servo

//Controller Variables 
int pixy_servo_increment = 40; //Joystick increment for Pixy Pan/tilt (20 works with 20 msec delay)
long teleop_period = 40; //Period of teleop algorithm (msec)
long buttonRead_period = 100; //Period of button reads (msec)

//Motor Variables
int MAX_MOTOR_SPEED = 300;    // controls maximum speed of the robot
int MIN_MOTOR_SPEED = -300;   // controls minimum speed of the robot

int leftSpeed = 0; //Current Speed for Left Motor 
int rightSpeed = 0; //Current Speed for Right Motor 

int MAX_TELEOP_SPEED = MAX_MOTOR_SPEED; //Max Teleop Speed (i.e. joystick control)

//Search Variables
int scanIncrement = 20; // controls how fast the robot pans (used when robot is searching for targets) [6 works with 20 msec delay]
long search_period = 40; //Period of search algorithm (msec)
int MAX_SEARCH_SPEED = MAX_MOTOR_SPEED; //Max Search Speed

//Engage Variables
long engage_period = 10; //Period of engage algorithm (msec)
int nominal_engage_speed = 150; //Nominal engagement speed (reference point - occurs when there is 0 error)
int MAX_ENGAGE_SPEED = MAX_MOTOR_SPEED; //Max Engage Speed
int scaling_distance = 300; //Motor speeds will be cut by scaling_factor when pixycam within this distance of the servo limits
float scaling_factor = 2; //Motors get divided by this amount when pixycam is near servo limits

int prevTargetError = 0; //Error term needed for derivative component of Motor PD controller

//Timing Variables
unsigned long eccm_activation_time = 0; //Tracks when ECCM was last activated
unsigned long target_last_spotted = 0; //Tracks when valid target was last spotted
unsigned long last_search     = 0; //Tracks when robot last initiated a search
unsigned long last_teleop     = 0; //Tracks when robot was last teleoped
unsigned long last_engage     = 0; //Tracks when robot last attacked a target
unsigned long last_jammed     = 0; //Tracks when robot was last jammed
unsigned long last_buttonRead = 0; //Tracks when controller buttons were last read

long search_duration = 0; //Duration of search algorithm

//Other Variables
double joy_max = 255; //Max value for Left & Right Joysticks
long jamming_duration = 3000; // Jamming duration (msec)

//====================================================================================================================================================  
//  Pixy Pet Robot
//
//  Adafruit invests time and resources providing this open source code,
//  please support Adafruit and open-source hardware by purchasing
//  products from Adafruit!
//
// Written by: Bill Earl for Adafruit Industries
//
//==========================================================================
// begin license header
//
// All Pixy Pet source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
//
// end license header
//
//==========================================================================
//
// Portions of this code are derived from the Pixy CMUcam5 pantilt example code.
//
//==========================================================================
//
//====================================================================================================================================================
//---------------------------------------
// Servo Control Class
// A Proportional/Derivative feedback
// loop for pan/tilt servo tracking of
// blocks.
// (Based on Pixy CMUcam5 example code)
//---------------------------------------

class ServoControl
{
  public:
    ServoControl();
    ServoControl(int proportionalGain, int derivativeGain, int centerPos);

    void init(int proportionalGain, int derivativeGain, int centerPos);
    void update(int error);

    int m_pos;
    int m_prevError;
    int m_proportionalGain;
    int m_derivativeGain;
};

// ServoControl Constructors
ServoControl::ServoControl(int proportionalGain, int derivativeGain, int centerPos )
{
  m_pos = centerPos;
  m_proportionalGain = proportionalGain;
  m_derivativeGain = derivativeGain;
  m_prevError = 0x8000;
}

ServoControl::ServoControl(){}

//ServoControl initialization
void ServoControl::init(int proportionalGain, int derivativeGain, int centerPos )
{
  m_pos = centerPos;
  m_proportionalGain = proportionalGain;
  m_derivativeGain = derivativeGain;
  m_prevError = 0x8000;
}

// ServoControl Update
// Calculates new output based on the measured
// error and the current state.

void ServoControl::update(int error)
{
  int velocity;

  if (m_prevError != 0x8000)
  {
    velocity = (error * m_proportionalGain + (error - m_prevError) * m_derivativeGain) >> 8; //Divide error by 256 (i.e. Right Shift 8 bits)

    m_pos += velocity;
    if (m_pos > RCS_MAX_POS)
    {
      m_pos = RCS_MAX_POS;
    }
    else if (m_pos < RCS_MIN_POS)
    {
      m_pos = RCS_MIN_POS;
    }
  }
  m_prevError = error;
}

// End Servo Control Class
#endif