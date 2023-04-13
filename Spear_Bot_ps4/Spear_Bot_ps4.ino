//====================================================================================================================================================
// Spear_Bot_ps4.ino
//
// This program detects different-colored objects and engages them based on user-specified rules of engagement. A wireless Playstation 4 controller 
// can be used to control the robot, if desired.
//
// See controller_layout.pdf for controller layout. Addtional program settings can be found in: Arduino/libraries/Spear_Bot_ps4/Spear_Bot_ps4.h
//====================================================================================================================================================

#include <Spear_Bot_ps4.h>


/*********************INITIAL ROBOT STATES*************************
//
//Robot States (See Spear_Bot_ps4.h header file for definitions)
//
**********************************************************/

ROBOT_AUTONOMY        robot_autonomy      =   SEMI_AUTONOMY_TELEOP;   //Robot's autonomy 
ROBOT_INTERFACE       robot_interface     =   NORMAL;                 //Robot's interface
ROBOT_STATE           robot_state         =   DISARMED;               //Robot's state
RULES_OF_ENGAGEMENT   robot_rules         =   HOLD_FIRE;              //Rules of Engagement for robot
SEARCH_MODES          robot_search_mode   =   PAN;                    //Robot's search mode
ROBOT_STATUS          robot_status        =   LIVE;                   //Robot's status

//Jamming Parameters
int eccm = 3; //Number of ECCM Activations [1,inf) - robot cannot be unjammed when this number reaches zero
long eccm_duration = 5000; //Duration of ECCM Activation (milliseconds)
bool robot_unjammable = false; //Can robot be jammed? (no=true, yes=false)

bool finite_actions = false; //Are there a finite number of actions before robot link is jammed? (yes=true, no=false)
int actions_remaining = 5; //Number of actions that can be issued to the robot before robot link is permenantly jammed [Must be a positive number]

//Signature List for pixycam. Valid values are: UNUSED, FRIENDLY, HOSTILE, NEUTRAL, UNKNOWN, INCOMING_FIRE, JAMMER
COLOR_SIGNATURES signatureList[7] = {
  UNUSED, //Signature 1
  UNUSED, //Signature 2
  UNUSED, //Signature 3
  UNUSED, //Signature 4
  UNUSED, //Signature 5
  UNUSED, //Signature 6
  UNUSED, //Signature 7
};

//==========================================================================
//  PERSISTENCE
//
// Persistence values range from 0 - 2^32.
// This variable effects how long the robot will continue to repeat its last action if the robot loses sight of a target.

unsigned long persistence = 2000;    // time in milliseconds
//==========================================================================

/*********************CONTROLLER CONFIGURATION*************************/
bool use_controller = true; //Is controller being used? (yes=true, no=false)

double deadzone_l = 20; //Left Joystick Deadzone (0-255)
double deadzone_r = 20; //Right Joystick Deadzone (0-255)

double sensitivity_l = 1; //Sensitivity for Left Joystick [0,inf)
double sensitivity_r = 1; //Sensitivity for Right Joystick [0,inf)

//Controller Axis Inversion(s): Valid Values are [NON_INVERTED, INVERTED]
double invX_l = NON_INVERTED; //Inversion for Left Joystick, X-Axis 
double invY_l = NON_INVERTED; //Inversion for Left Joystick, Y-Axis 
double invX_r = NON_INVERTED; //Inversion for Right Joystick, X-Axis 
double invY_r = INVERTED; //Inversion for Right Joystick, Y-Axis

//====================================================================================================================================================
// 
// Main Program Starts Here
//
//====================================================================================================================================================
ServoControl panServo, tiltServo; //PD Controllers for Pixycam Pan/Tilt Servos

void setup() { //This code runs *once* at the start of the program

  //Serial Port Initialization
  Serial.begin(115200);
  #if !defined(__MIPSEL__)
    while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  #endif
  
  if (Usb.Init() == -1) { //Initialize USB Shield
    Serial.print(F("\r\nOSC did not start"));
    while (1); // Halt
  }

  //Set Motors to zero
  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);

  //Initialize pixycam
  pixy.init();
  pixy.setServos(RCS_PAN_CENTER_POS, RCS_TILT_CENTER_POS); //Ensure that pixycam is centered

  panServo.init(Kp_p, Kd_p, RCS_PAN_CENTER_POS);
  tiltServo.init(Kp_t, Kd_t, RCS_TILT_CENTER_POS);

}

void loop() { //This is the main loop of the program
  
  //Task #0: Initialize Local Variables    
  Usb.Task();
  int leftMotorSpeed, rightMotorSpeed; //Speeds for the left/right motors
  int targetID = -1; //ID of closest valid target. -1 means there are no valid targets.
  bool valid_target = false; //Flag denoting whether robot sees a valid target

  if (robot_status == LIVE){ //Run program when robot is LIVE
    if (PS4.connected() || use_controller == false) { //Run program when PS4 Controller is connected (unless controller is explictly stated to be unused)
      //---------Task #1: Process PS4 Controller---------
      
      //Controller commands only sent when robot is not JAMMED, and the robot is ARMED/TARGET_SPOTTED and in SEMI_AUTONOMY_TELEOP mode
      //Joysticks commands are throttled. This is needed so that there is time to read/process pixycam signatures.
      if (robot_autonomy == SEMI_AUTONOMY_TELEOP && (robot_state == ARMED || robot_state == TARGET_SPOTTED) && robot_interface == NORMAL){         
        if (millis() - last_teleop > teleop_period){ //Period of teleop algorithm. "teleop_period" defined in Spear_Bot_ps4.h
          parseJoysticks(leftMotorSpeed, rightMotorSpeed, panServo.m_pos, tiltServo.m_pos); 
          
          last_teleop = millis();
          motors.setLeftSpeed(leftMotorSpeed);
          motors.setRightSpeed(rightMotorSpeed);
          pixy.setServos(panServo.m_pos, tiltServo.m_pos); 
        }
      }

      if (use_controller == true && (millis() - last_buttonRead > buttonRead_period)){ //If controller is being used, parse Controller Buttons/LED with specified period.
        parseButtons();
        setControllerLED();
        last_buttonRead = millis();
      }

      if (finite_actions && actions_remaining <= 0) {robot_interface = JAMMED;} //Permenantly Jam robot if there are no moves remaining
      
      //---------Task #2: Read Robot Sensors and Identify Targets---------
      valid_target = identifyObjects(pixy.getBlocks(), targetID); //Returns ID of closest valid target
           
      //---------Task #3: Perform Robot Actions---------   
      if (robot_state != DISARMED){ //Only perform actions when the robot isn't disarmed

        //Perform these actions when there are no valid targets
        if (valid_target == false){
          //Lost track of target
          if (millis() - target_last_spotted > persistence){
            prevTargetError = 0; //Zero error from previous engagement loop
            robot_state = ARMED; //Change robot state to ARMED (i.e. robot will leave the TARGET_SPOTTED/ENGAGING_TARGET states when this occurs)

            if (robot_autonomy != SEMI_AUTONOMY_TELEOP && (millis() - last_search > search_period) && (millis() - last_search > search_duration)){ //Periodically search for targets if robot is not being teleoped
              motors.setLeftSpeed(0); 
              motors.setRightSpeed(0);
              
              last_search = millis(); //Mark time when robot initiated search
              searchForTargets();
            }
          } 
        }
  
        // Respond to valid targets based on robot autonomy 
        if (valid_target == true){
          target_last_spotted = millis(); //Mark time that target was spotted
                    
          switch (robot_autonomy){
            case SEMI_AUTONOMY_TELEOP: //If robot is armed, change to target_spotted state             
              if (robot_state == ARMED){robot_state = TARGET_SPOTTED;}
              break;
    
            case SEMI_AUTONOMY: //If robot is armed, change to target_spotted state
              if (robot_state == ARMED){robot_state = TARGET_SPOTTED;}
              break;
    
            case SUPERVISED_AUTONOMY: //engage target
              robot_state = ENGAGING_TARGET;
              break;
    
            case FULL_AUTONOMY: //engage target
              robot_state = ENGAGING_TARGET;
              break;
    
            default:
              break; 
          }
                  
          //Target Engagement Algorithm: Engage closest valid target for the specified duration.         
          if (robot_state == ENGAGING_TARGET && targetID >= 0 && (millis() - last_engage > engage_period)){ //Criteria for engaging a target               
            //Align pixycam with target
            trackTarget(targetID);

            //Attack Target
            engageTarget(nominal_engage_speed); //Engage target at specified motor speed
       
            last_engage = millis(); //Mark time when engagement ended
          }

        
        
        } //end if (valid_target == true)
      } // end if (robot_state != DISARMED)

      else { //This code only executes when robot is DISARMED
        motors.setLeftSpeed(0);
        motors.setRightSpeed(0);
      }

    } //end if (PS4.connected())
     
    else { //Stop motors when PS4 controller is disconnected
      motors.setLeftSpeed(0);
      motors.setRightSpeed(0);
    }
  } //end if (robot_status == LIVE)
  
  //This code is only executed when robot isn't LIVE (e.g. INERT)
  else { 
    motors.setLeftSpeed(0);
    motors.setRightSpeed(0); 
  }

}//end loop()

//====================================================================================================================================================
//====================================================================================================================================================
//
//
//      CLASSES AND FUNCTIONS
//
//
//====================================================================================================================================================
//====================================================================================================================================================

//---------------------------------------
// parseJoysticks()
//
// This function parses the joysticks on
// the PS4 controller and uses them to 
// control the robot's movement.
//---------------------------------------

void parseJoysticks(int &leftMotorSpeed, int &rightMotorSpeed, int &curr_pixy_pan, int &curr_pixy_tilt){
    /***********
    * /JAC: Code to control Zumo Motors with PS4 Joystick
    ************/

    //Step #0: Initializations
    double x_norm_l, y_norm_l, mag_v_l, ang_v_l;
    
    leftMotorSpeed = 0;
    rightMotorSpeed = 0;
    
    //Step #1: Normalize Joystick values to Unit Circle (i.e. [-1..1])   
    x_norm_l = (double) PS4.getAnalogHat(LeftHatX);
    y_norm_l = (double) PS4.getAnalogHat(LeftHatY);

    double curr_x_l = x_norm_l - (joy_max/2);
    double curr_y_l = y_norm_l - (joy_max/2);
    
    if (abs(curr_x_l)  < deadzone_l){ //X-axis within deadzone, set value to 0
      x_norm_l = 0;
    } else{
      x_norm_l = invX_l * (x_norm_l - deadzone_l - ((joy_max - deadzone_l)/2)) / ((joy_max - deadzone_l)/2);
    }

    if (abs(curr_y_l) < deadzone_l){ //Y-axis within deadzone, set value to 0
      y_norm_l = 0;
    } else{
      y_norm_l = -invY_l * (y_norm_l - deadzone_l - ((joy_max - deadzone_l)/2)) / ((joy_max - deadzone_l)/2);
    }
    
    //Step #2: Get Magnitude and Direction of Joystick Vector using Polar coordinates
    mag_v_l = sqrt(x_norm_l * x_norm_l + y_norm_l * y_norm_l);
    ang_v_l = atan2(y_norm_l, x_norm_l) - (45 * M_PI/180); //-45 Degree angle shift needed to align movement with joysticks (e.g. Up on joystick moves both motors forward)

    //Step #3: Convert Joystick direction to Motor Commands
    leftMotorSpeed  = (int) (sensitivity_l * mag_v_l * cos(ang_v_l) * (double) MAX_TELEOP_SPEED);
    rightMotorSpeed = (int) (sensitivity_l * mag_v_l * sin(ang_v_l) * (double) MAX_TELEOP_SPEED);
    
    constrain(leftMotorSpeed, -MAX_TELEOP_SPEED, MAX_TELEOP_SPEED);
    constrain(rightMotorSpeed,-MAX_TELEOP_SPEED, MAX_TELEOP_SPEED);
    
   /***********
    * /JAC: Code to control Pixycam Pan/Tilt with PS4 Joystick
    ************/

    //Step #0: Initializations
    double x_norm_r, y_norm_r, mag_v_r, ang_v_r;
        
    //Step #1: Normalize Joystick values to Unit Circle (i.e. [-1..1])   
    x_norm_r = (double) PS4.getAnalogHat(RightHatX);
    y_norm_r = (double) PS4.getAnalogHat(RightHatY);

    double curr_x_r = x_norm_r - (joy_max/2);
    double curr_y_r = y_norm_r - (joy_max/2);
    
    if (abs(curr_x_r)  < deadzone_r){ //X-axis within deadzone, set value to 0
      x_norm_r = 0;
    } else{
      x_norm_r = -invX_r * (x_norm_r - deadzone_r - ((joy_max - deadzone_r)/2)) / ((joy_max - deadzone_r)/2); //Negative sign needed so that camera pans left when joystick is moved left.
    }

    if (abs(curr_y_r) < deadzone_l){ //Y-axis within deadzone, set value to 0
      y_norm_r = 0;
    } else{
      y_norm_r = invY_r * (y_norm_r - deadzone_r - ((joy_max - deadzone_r)/2)) / ((joy_max - deadzone_r)/2); //Negative sign needed so that camera tilts up when controller is moved up
    }
    
    //Step #2: Get Magnitude and Direction of Joystick Vector using Polar coordinates
    mag_v_r = sqrt(x_norm_r * x_norm_r + y_norm_r * y_norm_r);
    ang_v_r = atan2(y_norm_r, x_norm_r); 

    //Step #3: Convert Joystick direction to Pan/Tilt Commands
    curr_pixy_pan  = curr_pixy_pan   + (int) (sensitivity_r * mag_v_r * cos(ang_v_r) * (double) pixy_servo_increment);
    curr_pixy_tilt = curr_pixy_tilt  + (int) (sensitivity_r * mag_v_r * sin(ang_v_r) * (double) pixy_servo_increment);
       
    if (curr_pixy_pan > PIXY_RCS_MAX_POS){curr_pixy_pan = PIXY_RCS_MAX_POS;}
    if (curr_pixy_pan < PIXY_RCS_MIN_POS){curr_pixy_pan = PIXY_RCS_MIN_POS;}

    if (curr_pixy_tilt > PIXY_RCS_MAX_POS){curr_pixy_tilt = PIXY_RCS_MAX_POS;}
    if (curr_pixy_tilt < PIXY_RCS_MIN_POS){curr_pixy_tilt = PIXY_RCS_MIN_POS;}
       
} //End Function

//====================================================================================================================================================
//---------------------------------------
// parseButtons()
//
// This function parses the buttons on the PS4
// controller and uses them to modify the
// behavior of the robot.
//---------------------------------------

void parseButtons(){

  //--------------------------------------------------------------------------------------------------------
  // These buttons will *only* be read when the robot isn't jammed and isn't in FULL_AUTONOMY mode
  //--------------------------------------------------------------------------------------------------------
  if (robot_interface == NORMAL && robot_autonomy != FULL_AUTONOMY){

    //General Functionality
    if (PS4.getButtonPress(R3) && robot_state != ENGAGING_TARGET){ //Recenter Pixy camera
      panServo.m_pos = RCS_PAN_CENTER_POS;
      tiltServo.m_pos = RCS_TILT_CENTER_POS;
     
      pixy.setServos(panServo.m_pos, tiltServo.m_pos); //Recenter Pixycam Pan/Tilt
      if (finite_actions) {actions_remaining--;}
    }
    
    //Robot States & Autonomy
    if (PS4.getButtonPress(L3)){ //Set Robot's state to SEMI_AUTONOMY
      robot_autonomy = SEMI_AUTONOMY;
      robot_state = ARMED;

      motors.setLeftSpeed(0);
      motors.setRightSpeed(0);
      if (finite_actions) {actions_remaining--;}
    }
    
    if (PS4.getButtonPress(L2)){ //Disarm Robot
      robot_state = DISARMED;

      motors.setLeftSpeed(0);
      motors.setRightSpeed(0);
      if (finite_actions) {actions_remaining--;}
    }
    
    if (PS4.getButtonPress(R2) && robot_state == TARGET_SPOTTED){ //Engage Target(s)
      robot_state = ENGAGING_TARGET;
      if (finite_actions) {actions_remaining--;}
    }
    
    if (PS4.getButtonPress(L1)){ //Set Robot's state to SEMI_AUTONOMY (Teleop)
      robot_autonomy = SEMI_AUTONOMY_TELEOP;
      robot_state = ARMED; 
      if (finite_actions) {actions_remaining--;}     
    }
    
    if (PS4.getButtonPress(R1)){ //Set Robot's state to SUPERVISED_AUTONOMY
      robot_autonomy = SUPERVISED_AUTONOMY;
      robot_state = ARMED;

      motors.setLeftSpeed(0);
      motors.setRightSpeed(0);
      if (finite_actions) {actions_remaining--;}
    }

    //Rules of Engagement
    if (PS4.getButtonPress(TRIANGLE)) { //Weapons Free
      robot_rules = WEAPONS_FREE;
      robot_state = ARMED;  
      if (finite_actions) {actions_remaining--;}
    }
    
    if (PS4.getButtonPress(CIRCLE)) { //Weapons Tight
      robot_rules = WEAPONS_TIGHT;
      robot_state = ARMED;
      if (finite_actions) {actions_remaining--;}
    }
    
    if (PS4.getButtonPress(CROSS)) { //Weapons Hold
      robot_rules = WEAPONS_HOLD;
      robot_state = ARMED;
      if (finite_actions) {actions_remaining--;}
    }
    
    if (PS4.getButtonPress(SQUARE)) { //Hold Fire
      robot_rules = HOLD_FIRE;
      robot_state = ARMED;
      if (finite_actions) {actions_remaining--;}
    }

    //Search Modes
    if (PS4.getButtonPress(UP)) { //Search Mode PAN_FW
      robot_search_mode = PAN_FW;
      if (finite_actions) {actions_remaining--;}
    } 
    
    if (PS4.getButtonPress(RIGHT)) { //Search Mode PAN_CW
      robot_search_mode = PAN_CW;
      if (finite_actions) {actions_remaining--;}
    } 
    
    if (PS4.getButtonPress(DOWN)) { //Search Mode PAN
      robot_search_mode = PAN;
      if (finite_actions) {actions_remaining--;}
    } 
    
    if (PS4.getButtonPress(LEFT)) { //Search Mode PAN_CCW
      robot_search_mode = PAN_CCW;
      if (finite_actions) {actions_remaining--;}
    }

    //Terminal Commands (you lose control of the robot once these commands are issued)
    if (PS4.getButtonPress(SHARE)){ //Set Robot to Full Autonomy Mode
      robot_autonomy = FULL_AUTONOMY;
      robot_state = ARMED;

      motors.setLeftSpeed(0);
      motors.setRightSpeed(0);
      if (finite_actions) {actions_remaining--;}
    }
  
    if (PS4.getButtonPress(OPTIONS)){ //Issue Self-Destruct (i.e. Halt Program)
      robotDestroyed();
      if (finite_actions) {actions_remaining--;}
    }
    
  } //End Normal Operation Commands

  //--------------------------------------------------------------------------------------------------------
  // These buttons will *always* be read, regardless of robot's state
  //--------------------------------------------------------------------------------------------------------

  if (PS4.getButtonPress(PS)) { //Connect/Disconnect PS4 Controller
    PS4.disconnect();
  }
  
  if (PS4.getButtonPress(TOUCHPAD) && eccm > 0 && robot_interface != NORMAL){ //Deploy ECCM iff robot is jammed
    eccm--;
    robot_interface = NORMAL;
    eccm_activation_time = millis(); 
  }
  
} //End parseButtons Function

//====================================================================================================================================================
//---------------------------------------
// trackTarget(int blockCount, int targetID)
//
// This code aligns the pixycam with the closest
// valid target
//
// Input Variables:
//  1. int targetID: Target ID of the target in the pixycam
//      blocks array. ID must be non-negative.
//---------------------------------------

void trackTarget(int targetID)
{
  if (targetID < 0) {return;} //Lost track of target, exit function
  
  int panError = X_CENTER - pixy.blocks[targetID].x;
  int tiltError = pixy.blocks[targetID].y - Y_CENTER;

  panServo.update(panError);
  tiltServo.update(tiltError);

  pixy.setServos(panServo.m_pos, tiltServo.m_pos);
}

//====================================================================================================================================================
//---------------------------------------
// engageTarget(int targetID, int engageSpeed)
//
// This code engages the current target. It
// assumes that identifyTarget() has been called.
// The robot uses this function to ram the
// target. A PD controller is used to keep
// the robot centered on-target.
//
// Input Variables:
//  1. int engageSpeed: The robot engages the
//      target at this speed. Value should be >0.
//---------------------------------------

void engageTarget(int engageSpeed)
{    
  //Align robot with Target
  int targetError = RCS_PAN_CENTER_POS - panServo.m_pos;  // How far off-center are we looking now?

  int forwardSpeed = engageSpeed; //Robot will chase targets at this speed

  //Error Terms for PD Controller
  float Kp_term = Kp_f*float(targetError);
  float Kd_term = Kd_f*float(targetError-prevTargetError);
  int differential = int(Kp_term+Kd_term);
 
  // Adjust the left and right speeds by the steering differential.
  leftSpeed = constrain(forwardSpeed + differential, -MAX_ENGAGE_SPEED, MAX_ENGAGE_SPEED);
  rightSpeed = constrain(forwardSpeed - differential, -MAX_ENGAGE_SPEED, MAX_ENGAGE_SPEED);

  //Scale motor speeds so that robot does not overshoot target
  if (abs(panServo.m_pos - RCS_MIN_POS) < scaling_distance || abs(RCS_MAX_POS - panServo.m_pos) < scaling_distance){ //Reduce motor speeds when pixycam is near servo limits
    leftSpeed = round((float) leftSpeed / scaling_factor);
    rightSpeed = round((float) rightSpeed / scaling_factor); 
  }
  
  prevTargetError = targetError; //Store follow error for next iteration

  motors.setLeftSpeed(leftSpeed);
  motors.setRightSpeed(rightSpeed);
  
}

//====================================================================================================================================================
//---------------------------------------
// searchForTargets()
//
// This function uses one of the specified
// search patterns to find targets. Search
// patterns 0-3 are mapped to the PS4 D-pad.
//---------------------------------------

// Scan left and right for blob while adjusting scan speed and tilt at the end of each scan.
void searchForTargets()
{ 
  //Initialization
  search_duration = 5;
  int speed1 = 0; 
  int speed2  = 0;
  panServo.m_pos += scanIncrement;

  //Rotate the pixycam, if possible 
  if (panServo.m_pos > RCS_MIN_POS && panServo.m_pos < RCS_MAX_POS && tiltServo.m_pos > RCS_MIN_POS && tiltServo.m_pos < RCS_MAX_POS){ //Rotate the pixycam if it isn't at a servo limit 
    pixy.setServos(panServo.m_pos, tiltServo.m_pos);
    return;
  }

  //Adjust pixycam's tilt angle
  tiltServo.m_pos = random(tilt_lower_limit, tilt_upper_limit); //Randomize the pixycam's tilt angle. Tilting the camera could allow the pixycam to detect additional objects

  //Ensure that pixycam pan servo has a valid position
  if (panServo.m_pos > RCS_MAX_POS) {panServo.m_pos = RCS_MAX_POS;}
  if (panServo.m_pos < RCS_MIN_POS) {panServo.m_pos = RCS_MIN_POS;}
  
  scanIncrement = -scanIncrement; //Reverse pan direction

  //Step 3: Execute Search Algorithm
  switch(robot_search_mode)
  {
    case PAN: //D-pad: Down, Pan camera
      search_duration = random(150, 250);
      leftSpeed = 0;
      rightSpeed = 0;
      break;

    case PAN_CCW: //D-pad: Left, Pan camera; rotate counter-clockwise if no targets found
      if (scanIncrement < 0)
      {
        leftSpeed = -150;
        rightSpeed = 150;
      }
      search_duration = random(150, 250);
      break;
                
    case PAN_CW: //D-pad: Right, Pan camera; rotate clockwise if no targets found
      if (scanIncrement < 0)
      {
        leftSpeed = 150;
        rightSpeed = -150;
      }
      search_duration = random(150, 250);         
      break;

    case PAN_FW: //D-pad: Up, Pan camera; drive forward if no targets found 
      leftSpeed = random(200, 300);
      rightSpeed = leftSpeed;

      search_duration = random(500, 700);         
      break;
         
    default: //Pan camera
      search_duration = random(150, 250);
      leftSpeed = 0;
      rightSpeed = 0;
      break;
  }

  //Step 4: Set the motor speed based on search algorithm
  motors.setLeftSpeed(constrain(leftSpeed, -MAX_SEARCH_SPEED, MAX_SEARCH_SPEED));
  motors.setRightSpeed(constrain(rightSpeed, -MAX_SEARCH_SPEED, MAX_SEARCH_SPEED));
  
} //End searchForTargets() Function

//====================================================================================================================================================
//---------------------------------------
// robotDestroyed()
//
// This function sets the robot to "INERT" status,
// halts the program, and stops the motors
//---------------------------------------
void robotDestroyed(){
  
  //Stop Motors
  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);

  //Set robot status to Destroyed and disconnected PS4 controller if it's connected
  robot_status = INERT;
  if (PS4.connected()){PS4.disconnect();} 
}

//====================================================================================================================================================
//---------------------------------------
// setControllerLED()
//
// This function sets the controller's LED
// based on the robot's state.
//---------------------------------------
void setControllerLED(){
 
  if (robot_interface != NORMAL){ //Solid white light if robot is jammed
    PS4.setLed(White);
    PS4.setLedFlash(0, 0); //Turn off blinking
    return;
  }
  
  switch (robot_state) { //LED Colors based on Robot State
    
    case DISARMED: //Cyan LED
      PS4.setLed(Lightblue);
      break;
    
    case ARMED: //LED Color based on Autonomy mode
    
      switch (robot_autonomy) {        
        case SEMI_AUTONOMY_TELEOP: //Blue LED
          PS4.setLed(Blue);
          break;

        case SEMI_AUTONOMY: //Purple LED
          PS4.setLed(Purple);
          break;

        case SUPERVISED_AUTONOMY://Green LED
          PS4.setLed(Green);
          break;

        case FULL_AUTONOMY: //Ochre LED
          PS4.setLed(204, 119, 34);
          break; 

        default:
          break;
      }
      break;

    case TARGET_SPOTTED: //Yellow LED
      PS4.setLed(Yellow);
      break;
      
    case ENGAGING_TARGET: //Red LED
      PS4.setLed(Red);
      break;  

    default:
      break;
  }

  switch (robot_rules){ //LED Flashing based on Rules of Engagement
    case HOLD_FIRE: //Solid light
      PS4.setLedFlash(0, 0);
      break;
      
    case WEAPONS_HOLD: //Slowly Blinking Light
      PS4.setLedFlash(90, 90);
      break;
      
    case WEAPONS_TIGHT: //Moderately-Fast Blinking Light
      PS4.setLedFlash(30, 30);
      break;

    case WEAPONS_FREE: //Rapidly Blinking Light
      PS4.setLedFlash(10, 10);
      break;

    default:
      break;
  }
  
} //End setControllerLED()

//====================================================================================================================================================
//---------------------------------------
// identifyObjects()
//
// Identifies objects detected by pixycam.
// Returns the targetID of the closest valid
// target.
//
// Input Variables:
//  1. int blockCount: Number of objects detected by pixycam.
//  2. int targetID: ID for the closest valid target. Will
//      not get set if there are no valid targets.
//
// Return Variables:
//  1. bool validTarget: Flag denoting whether there are
//      any valid targets. True = Valid, False = Invalid
//---------------------------------------

bool identifyObjects(int blockCount, int &targetID)
{
  //Initializations
  int trackedBlock = -1; //Closest valid target. -1 means there are no valid targets.
  long maxSize = 0; //Size of closest valid target
  int current_signature; //ID of current signature
  
  bool robot_under_fire = false;  //Flag denoting whether robot is under attack
  bool jamming_signal = false; //Flag denoting whether robot is being jammed
  bool validTarget = false; //Flag denoting whether there is at least one valid target to engage
  bool currentTarget = false; //Flag denoting whether current object is a valid target

  for (int i = 0; i < blockCount; i++) // Look at each blob found and find the largest blob
  {
    //Get current signature
    current_signature = signatureList[pixy.blocks[i].signature-1]; //pixy.blocks.signature starts at 1. Need to subtract 1 so that it indexes at 0.

    //Process Battlefield events: Incoming Fire & Jamming
    if (current_signature == INCOMING_FIRE){ //Robot is under fire when it sees the color for INCOMING_FIRE
      robot_under_fire = true;
    }

    if (robot_autonomy != FULL_AUTONOMY && !robot_unjammable && current_signature == JAMMER){ //Robot is jammed when it sees the color for JAMMER and the robot is niether unjammable nor fully_autonomous
      jamming_signal = true;
       
      if (millis() - eccm_activation_time > eccm_duration) { //ECCM is no longer effective. Robot reverts to being jammed.
        robot_interface = JAMMED;
        last_jammed = millis();
      }
    }

    //Assess Target Validity
    currentTarget = targetAssessment(current_signature, robot_under_fire);

    if (!currentTarget) {continue;} //Continue to next target if the current target is invalid

    validTarget = true; //There is at least one valid target
    
    //Track the largest valid target
    long newSize = pixy.blocks[i].height * pixy.blocks[i].width;
    if (newSize > maxSize)
    {
      trackedBlock = i;
      maxSize = newSize;
    }
  } // end for()

  
  if (actions_remaining > 0 && jamming_signal == false && (millis() - last_jammed > jamming_duration) && robot_interface==JAMMED){robot_interface = NORMAL;} //Clear Jamming if it is no longer present
  
  targetID = trackedBlock; //Set target ID as the index of the closest valid target. Value will be -1 if there are no valid targets.
  
  return validTarget;
}

//====================================================================================================================================================
//---------------------------------------
// targetAssessment()
//
// Applies rules of engagement to determine
// whether target is valid.
//
// Input Variables:
//  1. int signature: Detected object's color signature
//  2. bool robot_under_fire: Flag denoting whether robot is
//      being attacked. True = Yes, False = No
//
// Return Variables:
//  1. bool validTarget: Flag denoting whether target
//      is valid. True = Valid, False = Invalid
//---------------------------------------

bool targetAssessment(int signature, bool robot_under_fire){

  switch (robot_rules){
    case HOLD_FIRE: //Don't engage targets
      return false;
      break;
      
    case WEAPONS_HOLD: //Only engage targets that are attacking you. Attackers are assumed to be hostile
      if (signature == INCOMING_FIRE){return true;}
      else {return false;}
      break;

    case WEAPONS_TIGHT: //Engage targets postively identified as hostile
      if (signature == HOSTILE || signature == INCOMING_FIRE){return true;}
      else {return false;}
      break;

    case WEAPONS_FREE: //Engage all hostile/unknown targets
      if (signature == HOSTILE || signature == UNKNOWN || signature == INCOMING_FIRE){return true;}
      else {return false;}
      break;
      
    default:
      return false;
      break;
  }
}
