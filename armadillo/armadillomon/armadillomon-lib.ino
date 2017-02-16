/*
  Armadillomon
  Armadillomon is a base robot.

  This code example show a base robot controlled using an infrared controller.

  To use this sketch install these libraries first:

    * IRRemote
    * TimerOne
    * DigitalIO

  To install these libraries open Sketch -> Include Library -> Manage Libraries
  select show all, and then search for the libraries, after install it, you
  can use the code.

  modified 13 Feb 2017
  by Jeferson Lima and Andressa Andrade
 */

#include <math.h>
#include <DigitalIO.h>
#include <IRremote.h>
#include <TimerOne.h>
#include <SoftwareSerial.h>

/***** USER CONFIG *****/

/* Enable control over serial */
#define SERIAL_CONTROL_ENABLED

/* Enable command using Ir Signal */
//#define IR_CONTROL_ENABLED

/* Enable serial log if you are using serial communication */
#define ENABLE_LOG

/* Enable log on software serial, the default is on main Serial */
//#define SOFTWARE_SERIAL_LOG

/* Enable display of lazy debug. Please don't use this on normal conditions */
#define LAZY_DEBUG

/* Enable use of old and deprecated functions */
//#define USE_OBSOLETE

/* Enable this macro if you have a L298P with mirroed motor outputs */
#define MIRROED_MOTORS

/***********************/

/* Serial Speed */
#define SERIAL_SPEED      9600

/* Motor Types */
/* Define the types of each motor */
#define MOTOR_LEFT        1
#define MOTOR_RIGHT       2
#define BOTH_MOTORS       3

/* Optical Encoders */
#define ENCODER_LEFT_PIN  2
#define ENCODER_RIGHT_PIN 3

/* IR Receiver pin */
#define RECEIVER_PIN      4

/* Speed of each motor */
/* Change speed of each motor */
#define SPEED_MOTOR_LEFT  5
#define SPEED_MOTOR_RIGHT 6

/* Enable of the Motor Left */
/* Enable change direction or stop this motor */
#define IN1_MOTOR_LEFT    7
#define IN2_MOTOR_LEFT    8

/* Enable of the Motor Right */
/* Enable change direction or stop this motor */
#define IN1_MOTOR_RIGHT   9
#define IN2_MOTOR_RIGHT   10

/* Software Serial Pins */
#define SOFTWARE_SERIAL_TX_PIN  11
#define SOFTWARE_SERIAL_RX_PIN  12

/* Define the pins of the Ultrasonic Sensors */
#define TRIGGER_PIN       14
#define ECHO_PIN_LEFT     15
#define ECHO_PIN_RIGHT    16

/* Robot Configs */

/* Robot Width */
#define ROBOT_WIDTH_MM    156

/* Robot Length */
#define ROBOT_LENGTH_MM   210

/* Center of wheels */
#define WHEEL_CENTER_X    78
#define WHEEL_CENTER_Y    135

/* Wheel width */
#define WHEEL_WIDTH_MM    26

/* Wheel size */
#define WHEEL_DIAMETER_MM 76

/* Robot distance between wheels */
#define WHEEL_DISTANCE_MM 100

/* Number of slices of optical encoder */
/* WARNING: MULTIPLY NUMBER OF SPACES IN ENCODER BY TWO IF INTERRUPTION IS CHANGE! */
#define ENCODER_STEPS 20

/* Encoder type of interruption */
#define ENCODER_INTERRUPTION_TYPE RISING

/* Max number of difference between the speed of the motors */
#define MAX_STEPS_ERROR 2

/* Command buttons */

/* Forward Button */
#define FORWARD_BUTTON_IR   0x8076A05F
#define FORWARD_COMMAND     'F'

/* Reverse Button */
#define REVERSE_BUTTON_IR   0x807620DF
#define REVERSE_COMMAND     'R'

/* Step Left Button */
#define LEFT_BUTTON_IR      0x8076F807
#define STEP_LEFT_COMMAND    '1'

/* Step Right Button */
#define RIGHT_BUTTON_IR     0x80767887
#define STEP_RIGHT_COMMAND  '2'

/* Turn Left Command */
#define TURN_LEFT_COMMAND   '3'

/* Turn Right Command */
#define TURN_RIGHT_COMMAND  '4'

/* Step Command */
#define STEP_COMMAND        'S'
#define TURN_COMMAND        'T'

/* Directions possible */
#define LEFT_DIRECTION      'L'
#define RIGHT_DIRECTION     'R'

/* Acelerate Command */
#define ACELERATE_COMMAND   'A'

/* Brake Button */
#define BRAKE_BUTTON_IR     0x8076708F
#define BRAKE_COMMAND       'B'

/* Dump data */
#define DUMP_COMMAND        'D'

/* Default speeds */

/* Stop speed */
#define PWM_STOP_SPEED    0

/* Low speed - 30% */
#define PWM_LOW_SPEED     80

/* Medium speed - 60% */
#define PWM_MEDIUM_SPEED  150

/* High Speed - 90% */
#define PWM_HIGH_SPEED    230

/* Maximum Speed - 100% */
#define PWM_MAXIMUM_SPEED 255

/* Max pwm increment */
#define MAX_PWM_INCREMENT 20

/* MAX pwm decrement */
#define MAX_PWM_DECREMENT 20

/* Timer Utilities */

/* Time constants */
#define TIME_1S_MS        1000
#define TIME_2S_MS        2000
#define TIME_0_1S_US      100000
#define TIME_0_25S_US     250000
#define TIME_0_5S_US      500000
#define TIME_1S_US        1000000
#define TIME_2S_US        1000000

/* Wait while some mutex are locked */
#define await(MUTEX_NAME) while(MUTEX_NAME)

/* Grid Movimentation  */

/* Grid Size */
#define GRID_SIZE         20

/* 
 *                   !!!!! WARNING !!!!!
 *           GRIDS SHOULDN'T BE GREATER THAN 35! 
 *  A 35x35 CONSUMES 1225 BYTES AND MIGHT BREAK EVERYTHING!
 */

/* Helpers */

/* Acelerate both motors */
#define acelerateMotors(motor_speed) acelerateMotor(BOTH_MOTORS, motor_speed)

/* Turn Forward a given motor */
#define turnForwardMotor(motor_num) changeMotorState(motor_num, HIGH, LOW)

/* Turn Reverse a given motor */
#define turnReverseMotor(motor_num) changeMotorState(motor_num, LOW, HIGH)

/* Brake a given motor */
#define brakeMotor(motor_num) changeMotorState(motor_num, LOW, LOW)

/* Turn both motors foward */
#define turnForward()   turnForwardMotor(BOTH_MOTORS)

/* Turn both motors reverse */
#define turnReverse()   turnReverseMotor(BOTH_MOTORS)

/* Remove PWM adjust from both motors */
#define zeroPwmAdjust() do { pwm_adjust_motor_left = 0; pwm_adjust_motor_right = 0; } while(false)

/* Zero distance reach */
#define zeroDistanceReach() do { distance_wanted = 0; distance_reach_motor_left = 0; distance_reach_motor_right = 0; } while(false)

/* Brake both motors */
#define brake()         do { brakeMotor(BOTH_MOTORS); acelerateMotors(PWM_STOP_SPEED); zeroPwmAdjust(); } while(false)

/* Step Right */
/* Stop one of the motors and move the other in opossite direction */
#define stepLeft()      do { brakeMotor(MOTOR_RIGHT); turnForwardMotor(MOTOR_LEFT); } while(false)

/* Step Left */
/* Stop one of the motors and move the other in opossite direction */
#define stepRight()     do { brakeMotor(MOTOR_LEFT);  turnForwardMotor(MOTOR_RIGHT); } while(false)

/* Turn Left */
/* Move both motors in opossite directions */
#define turnLeft()      do { turnForwardMotor(MOTOR_LEFT); turnReverseMotor(MOTOR_RIGHT); } while(false)

/* Turn Right */
/* Move both motors in opossite directions */
#define turnRight()     do { turnForwardMotor(MOTOR_RIGHT); turnReverseMotor(MOTOR_LEFT); } while(false)

/* Log Utilities */

#ifdef ENABLE_LOG

/* Start of a log */
#define LOG_LINE_START    "[log] >> "

/* Start of a debug message */
#define DEBUG_LINE_START  "[debug] >> "

/* Log serial object */
#ifndef SOFTWARE_SERIAL_LOG
#define LOG_OBJECT        Serial
#else
/* Create the object */
SoftwareSerial            serialLog(SOFTWARE_SERIAL_TX_PIN, SOFTWARE_SERIAL_RX_PIN);
#define LOG_OBJECT        serialLog
#endif

/* Print Log */
#define printLog(MSG)     LOG_OBJECT.print(F(LOG_LINE_START MSG))
#define printLogn(MSG)    printLog(MSG "\n")
#define printLogVar(VAR)  LOG_OBJECT.print(VAR)
#define printLogVarn(VAR) LOG_OBJECT.println(VAR)

/* Print Debug */
#define printDebug(MSG)   LOG_OBJECT.print(F(DEBUG_LINE_START MSG))
#define printDebugn(MSG)  printDebug(MSG "\n")

/* Print Progmem */
#define printMem(MSG)     LOG_OBJECT.print(F(MSG))
#define printMemn(MSG)    LOG_OBJECT.println(F(MSG))

#endif

#ifdef IR_CONTROL_ENABLED

/* IR Receiver */
IRrecv ir_receiver(RECEIVER_PIN);

/* IR Receiver Decoder */
decode_results ir_result;

#endif

/* Button State */
unsigned long button_state;

/* Selected virtual speed in cm/s */
int motor_speed_virtual_cms;

/* Selected pwm speed */
int actual_pwm_motor_speed;

/* Pwm adjust for each motor */
int pwm_adjust_motor_left;
int pwm_adjust_motor_right;

/* Actual rotation */
int actual_rotation;

/* Step Angle */
int step_angle;

/* Step Length */
int step_length;

/* Rotation length */
int rotation_length;

/* Distance wanted */
volatile unsigned int distance_wanted;

/* Distance read by each motor */
volatile unsigned int distance_reach_motor_left;
volatile unsigned int distance_reach_motor_right;

/* Number of steps completed */
volatile unsigned short steps_motor_left;
volatile unsigned short steps_motor_right;

/* Number of rotations completed */
volatile unsigned long rotations_motor_left;
volatile unsigned long rotations_motor_right;

/* Number of ticks on each motor, used to tune adjust */
volatile int adjust_ticks_motor_left;
volatile int adjust_ticks_motor_right;

/* Routine Flag Lock */
volatile bool routine_flag_mutex_lock;

/* ::::::::::::::::::: Configure the robot ::::::::::::::::::: */

void setup(){
  #ifdef ENABLE_LOG
  /* Initialize serial communication */
  LOG_OBJECT.begin(SERIAL_SPEED);
  #endif

  #ifdef ENABLE_LOG
  /* Print Hello Message */
  printMemn("...::: Hi, I'm Armadillomon :) ! :::...\n"
            "By Jeferson Lima & Andressa Andrade\n"
            "Starting System...\n" );
  #endif

  /* Print which user configurations are enabled */
  #ifdef ENABLE_LOG
  printMemn("Checking user settings:");
  
  /* Use of Serial Control */
  #ifdef SERIAL_CONTROL_ENABLED
  printMemn("SERIAL_CONTROL = ENABLED");
  #else
  printMemn("SERIAL_CONTROL = DISABLED");
  #endif
  
  /* Use of Control by IR */
  #ifdef IR_CONTROL_ENABLED
  printMemn("IR_CONTROL = ENABLED");
  #else
  printMemn("IR_CONTROL = DISABLED");
  #endif
  
  /* Log on Software Serial */
  #ifdef SOFTWARE_SERIAL_LOG
  printMemn("SOFTWARE_SERIAL_LOG = ENABLED");
  #else
  printMemn("SOFTWARE_SERIAL_LOG = DISABLED");
  #endif
  
  /* Lazy Debug */
  #ifdef LAZY_DEBUG
  printMemn("LAZY_DEBUG = ENABLED");
  #else
  printMemn("LAZY_DEBUG = DISABLED");
  #endif
  
  /* Use of obsolete or deprecated functions */
  #ifdef USE_OBSOLETE
  printMemn("USE_OBSOLETE = ENABLED");
  #else
  printMemn("USE_OBSOLETE = DISABLED");
  #endif
  
  /* Mirroed output motors pins */ 
  #ifdef MIRROED_MOTORS
  printMemn("MIRROED_MOTORS = ENABLED");
  #else
  printMemn("MIRROED_MOTORS = DISABLED");
  #endif

  printMemn("");
    
  #endif
  
  /* Configure all motors related pins as output */
  fastPinMode(IN1_MOTOR_LEFT, OUTPUT);
  fastPinMode(IN2_MOTOR_LEFT, OUTPUT);
  fastPinMode(IN1_MOTOR_RIGHT, OUTPUT);
  fastPinMode(IN2_MOTOR_RIGHT, OUTPUT);
  fastPinMode(SPEED_MOTOR_LEFT, OUTPUT);
  fastPinMode(SPEED_MOTOR_RIGHT, OUTPUT);

  /* Configure Interruption pins as INPUT with internal PULLUP ressitors */
  pinMode(ENCODER_LEFT_PIN, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_PIN, INPUT_PULLUP);

  #ifdef ENABLE_LOG
  /* Configurated all the pins */
  printLogn("Configurated all pin modes");
  #endif

  #ifdef IR_CONTROL_ENABLED
  /* Enable IR Receiver */
  ir_receiver.enableIRIn();
  #endif

  /* Initialize actual cm/s virtual speed */
  motor_speed_virtual_cms = 0;

  /* Initialize actual pwm motor speed */
  actual_pwm_motor_speed = 0;

  /* Initialize distance wanted by user */
  distance_wanted = 0;
  
  /* Initialize distance rech by each motor */
  distance_reach_motor_left = 0;
  distance_reach_motor_right = 0;

  /* Initialize step counters */
  steps_motor_left = 0;
  steps_motor_right = 0;

  /* Initiliaze rotation counters */
  rotations_motor_left = 0;
  rotations_motor_right = 0;

  /* Initialize actual rotation */
  actual_rotation = 0;

  /* Initialize pwm adjust speed */
  pwm_adjust_motor_left = 0;
  pwm_adjust_motor_right = 0;

  /* Initialize counter of ticks per motor adjust */
  adjust_ticks_motor_left = 0;
  adjust_ticks_motor_right = 0;

  #ifdef ENABLE_LOG
  /* Initialized all variables */
  printLogn("Initialized all variables");
  #endif

  /* Initialize step angle */
  step_angle = ceil(360.0 / ENCODER_STEPS);

  #ifdef ENABLE_LOG
  /* Calculated step angle */
  printLogn("Calculated step angle");
  #endif

  /* Initialize step length */
  step_length = ceil((step_angle * PI * (WHEEL_DIAMETER_MM / 2.0)) / 180.0);

  #ifdef ENABLE_LOG
  /* Calculated step length */
  printLogn("Calculated step length");
  #endif

  /* Initialize Rotation Length */
  rotation_length = ceil(PI * WHEEL_DIAMETER_MM);

  #ifdef ENABLE_LOG
  /* Calculated rotation length*/
  printLogn("Calculated rotation length");
  #endif

  /* Attach interrupts of the encoders */
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), stepCounterMotorLeft, ENCODER_INTERRUPTION_TYPE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), stepCounterMotorRight, ENCODER_INTERRUPTION_TYPE);
  
  #ifdef ENABLE_LOG
  /* Attached the encoder interrupts */
  printLogn("Attached the encoder interrupts");
  #endif

  /* Set motor default state */
  brake();

  #ifdef ENABLE_LOG
  /* Brake motor */
  printLogn("Brake motor");
  #endif
  
  #ifdef ENABLE_LOG
  /* Start adjust of motors */
  printLogn("Attached the encoder interrupts");
  #endif
  
  /* Start timer to adjust Motors in 1/4s */
  Timer1.initialize(TIME_0_1S_US);
  Timer1.attachInterrupt(adjustMotors);

  #ifdef ENABLE_LOG
  /* Start adjust of motors to desired virtual cm/s speed */
  printLogn("Motors adjust timer started");
  #endif

  #ifdef ENABLE_LOG
  /* Print end of setup */
  printMemn("\nSystem Started Successfully!\n");
  #endif
}

/* ::::::::::::::::::: Execute System code ::::::::::::::::::: */

void loop(){

#ifdef IR_CONTROL_ENABLED
  /* Read IR Signal */
  readIRSignal();
#endif

#ifdef SERIAL_CONTROL_ENABLED
  /* Read Serial commands */
  readSerialData();
#endif

  /* Check obstacles */
  checkObstacles();
}

/* :::::::::::::::::::       Helpers       ::::::::::::::::::: */

/* Increment counter of the motor left */
void stepCounterMotorLeft(){
  steps_motor_left++;  
  adjust_ticks_motor_left++;
  distance_reach_motor_left++;

  /* Show that a interruption has occured on lazy debug */
  #if defined(ENABLE_LOG) && defined(LAZY_DEBUG)
  printDebug("Interruption Ocurred - Left Side - #");
  printLogVar(rotations_motor_left);
  printMem(" $");
  printLogVarn(steps_motor_left);
  #endif
  
  /* Brake motors if we had reached the distance wanted */
  if(distance_reach_motor_left >= distance_wanted){
    brake();
    zeroDistanceReach();
  }

  /* Increase one rotation if we had completed one */
  if(steps_motor_left == ENCODER_STEPS){
    steps_motor_left = 0;
    rotations_motor_left++;
  }
}

/* Increment counter of the motor right */
void stepCounterMotorRight(){
  steps_motor_right++;
  adjust_ticks_motor_right++;
  distance_reach_motor_right++;

  /* Show that a interruption has occured on lazy debug */
  #if defined(ENABLE_LOG) && defined(LAZY_DEBUG)
  printDebug("Interruption Ocurred - Right Side - #");
  printLogVar(rotations_motor_right);
  printMem(" $");
  printLogVarn(steps_motor_right);
  #endif

  /* Brake motors if we had reached the distance wanted */
  if(distance_reach_motor_left >= distance_wanted){
    brake();
    zeroDistanceReach();
  }

  /* Increase one rotation if we had completed one */
  if(steps_motor_right == ENCODER_STEPS){
    steps_motor_right = 0;
    rotations_motor_right++;
  }
}

/* Adjust desired speed to pwm */
void adjustMotors(){
  // TODO
}

/* Check Obstacles */
void checkObstacles(){
  // TODO
}

/* Read Serial data */
void readSerialData(){

  /* If there are serial incoming data */
  if(Serial.available() > 0){
    /* Read serial and execute command */
    char serial_data = Serial.read();

    /* If we have special commands */
    if((serial_data == STEP_COMMAND || serial_data == TURN_COMMAND)){
      char serial_data2 = Serial.read();
      switch(serial_data2){
        case LEFT_DIRECTION:
          serial_data = (serial_data == STEP_COMMAND) ? '1' : '3';
        case RIGHT_DIRECTION:
          serial_data = (serial_data == STEP_COMMAND) ? '2' : '4';
        default:
          #ifdef ENABLE_LOG
          printLog("Invalid char read : ");
          printLogVarn(serial_data);
          #endif
          serial_data = '0';
      }
    }

    /* Execute the command received */
    executeCommand((unsigned long) serial_data);
  }
}

#ifdef IR_CONTROL_ENABLED
/* Read Ir signal */
void readIRSignal(){

  /* Verify if there are Ir signal */
  if(ir_receiver.decode(&ir_result)){

    /* Get the button pressed */
    button_state = ir_result.value;

    /* Execut the input command */
    executeCommand(button_state);

    /* Flush the ir receiver buffer */
    ir_receiver.resume();
  }
}
#endif

/* Change Motor State */
void executeCommand(unsigned long motor_command){
  
  /* Check the command and change the direction state */
  switch(motor_command){
    case ACELERATE_COMMAND:
      acelerateMotors(PWM_MEDIUM_SPEED);
      #ifdef ENABLE_LOG
      printLogn("Pressed acelerator");
      #endif
      break;
    case FORWARD_BUTTON_IR:
      acelerateMotors(PWM_MEDIUM_SPEED);    
    case FORWARD_COMMAND:
      turnForward();
      #ifdef ENABLE_LOG
      printLogn("Pressed Forward");
      #endif
      break;
    case REVERSE_BUTTON_IR:
      acelerateMotors(PWM_MEDIUM_SPEED);
    case REVERSE_COMMAND:
      turnReverse();
      #ifdef ENABLE_LOG
      printLogn("Pressed Reverse");
      #endif
      break;
    case LEFT_BUTTON_IR:
    case STEP_LEFT_COMMAND:
      stepLeft();
      #ifdef ENABLE_LOG
      printLogn("Pressed Step Left");
      #endif
      break;
    case RIGHT_BUTTON_IR:
    case STEP_RIGHT_COMMAND:
      stepRight();
      #ifdef ENABLE_LOG
      printLogn("Pressed Step Right");
      #endif
      break;
    case TURN_LEFT_COMMAND:
      turnLeft();
      #ifdef ENABLE_LOG
      printLogn("Pressed Turn Left");
      #endif
      break;
    case TURN_RIGHT_COMMAND:
      turnRight();
      #ifdef ENABLE_LOG
      printLogn("Pressed Turn Right");
      #endif
      break;
    case BRAKE_BUTTON_IR:
    case BRAKE_COMMAND:
      brake();
      #ifdef ENABLE_LOG
      printLogn("Pressed Brake");
      #endif
      break;
    case DUMP_COMMAND:
      #ifdef ENABLE_LOG
      dumpInfo();
      #endif
      break;
    default:
      #ifdef ENABLE_LOG
      printLogn("INVALID COMMAND!");
      printLog("Command Received: ");
      if(motor_command < 255){
        LOG_OBJECT.print((char)motor_command);
        LOG_OBJECT.print('\n');
      }
      else{
        printLogVarn(motor_command);
      }
      #endif
  }
}

/* Acelerate a given motor */
void acelerateMotor(int motor_num, int pwm_motor_speed){

  /* Store Actual Speed */
  actual_pwm_motor_speed = pwm_motor_speed;

  switch(motor_num){
    case MOTOR_LEFT:
      analogWrite(SPEED_MOTOR_LEFT, pwm_motor_speed + pwm_adjust_motor_left);
      break;
    case MOTOR_RIGHT:
      analogWrite(SPEED_MOTOR_RIGHT, pwm_motor_speed + pwm_adjust_motor_right);
      break;
    case BOTH_MOTORS:
      analogWrite(SPEED_MOTOR_LEFT, pwm_motor_speed + pwm_adjust_motor_left);
      analogWrite(SPEED_MOTOR_RIGHT, pwm_motor_speed + pwm_adjust_motor_right);
      break;
  }
}

/* Change state of enable pin */
void changeMotorState(int motor_num, bool in1_motor_state, bool in2_motor_state){
  switch(motor_num){
    case MOTOR_LEFT:
      fastDigitalWrite(IN1_MOTOR_LEFT, in1_motor_state);
      fastDigitalWrite(IN2_MOTOR_LEFT, in2_motor_state);
      break;
    case MOTOR_RIGHT:
      #ifdef MIRROED_MOTORS
      fastDigitalWrite(IN1_MOTOR_RIGHT, in2_motor_state);
      fastDigitalWrite(IN2_MOTOR_RIGHT, in1_motor_state);
      #else
      fastDigitalWrite(IN1_MOTOR_RIGHT, in1_motor_state);
      fastDigitalWrite(IN2_MOTOR_RIGHT, in2_motor_state);
      #endif
      break;
    case BOTH_MOTORS:
      fastDigitalWrite(IN1_MOTOR_LEFT, in1_motor_state);
      fastDigitalWrite(IN2_MOTOR_LEFT, in2_motor_state);
      #ifdef MIRROED_MOTORS
      fastDigitalWrite(IN1_MOTOR_RIGHT, in2_motor_state);
      fastDigitalWrite(IN2_MOTOR_RIGHT, in1_motor_state);
      #else
      fastDigitalWrite(IN1_MOTOR_RIGHT, in1_motor_state);
      fastDigitalWrite(IN2_MOTOR_RIGHT, in2_motor_state);
      #endif
      break;
  }
}

/* Dump variables in log object */
#ifdef ENABLE_LOG
void dumpInfo(){
  printLogn("Dumping system variables...");
  printMem("Button State = ");
  printLogVarn(button_state);
  printMem("Motor Speed Virtual Cms = ");
  printLogVarn(motor_speed_virtual_cms);
  printMem("Actual Pwm Motor Speed = ");
  printLogVarn(actual_pwm_motor_speed);
  printMem("Pwm Adjust Motor Left = ");
  printLogVarn(pwm_adjust_motor_left);
  printMem("Pwm Adjust Motor Right = ");
  printLogVarn(pwm_adjust_motor_right);
  printMem("Actual Rotation = ");
  printLogVarn(actual_rotation);
  printMem("Step Angle = ");
  printLogVarn(step_angle);
  printMem("Step Length = ");
  printLogVarn(step_length);
  printMem("Rotation Length = ");
  printLogVarn(rotation_length);
  printMem("Distance Wanted = ");
  printLogVarn(distance_wanted);
  printMem("Distance Reach Motor Left = ");
  printLogVarn(distance_reach_motor_left);
  printMem("Distance Reach Motor Right = ");
  printLogVarn(distance_reach_motor_right);
  printMem("Steps Motor Left = ");
  printLogVarn(steps_motor_left);
  printMem("Steps Motor Right = ");
  printLogVarn(steps_motor_right);
  printMem("Rotations Motor Left = ");
  printLogVarn(rotations_motor_left);
  printMem("Rotations Motor Right = ");
  printLogVarn(rotations_motor_right);
  printMem("Adjust Ticks Motor Left = ");
  printLogVarn(adjust_ticks_motor_left);
  printMem("Adjust Ticks Motor Right = ");
  printLogVarn(adjust_ticks_motor_right);
  printMem("Routine Flag Mutex Lock = ");
  printLogVarn(routine_flag_mutex_lock);
}
#endif

/* ::::::::::::::::::: Obsolete  Functions ::::::::::::::::::: */

/* Just here for history purpose */
#ifdef USE_OBSOLETE

/* Chaos correction test */
#define WITH_MORE_CHAOS_MORE_ORDER 5
#define MAX_CHAOS_ALLOWED 50
#define MIN_CHAOS_ALLOWED -MAX_CHAOS_ALLOWED

/* Increment counter of the motor left */
void oldStepCounterMotorLeft(){
  steps_motor_left++;

  /* Reset chaos if it's too great */
  if(pwm_adjust_motor_left > MAX_CHAOS_ALLOWED){
    pwm_adjust_motor_left = 0;
  }

  /* Adjust motor if it's starting move oposite */
  if((steps_motor_left - steps_motor_right) > 3){
    steps_motor_right -= 2;
    pwm_adjust_motor_left -= (pwm_adjust_motor_left > MIN_CHAOS_ALLOWED) ? (pwm_adjust_motor_left) : WITH_MORE_CHAOS_MORE_ORDER;
    pwm_adjust_motor_right += (pwm_adjust_motor_right > MAX_CHAOS_ALLOWED) ? (-pwm_adjust_motor_right) : WITH_MORE_CHAOS_MORE_ORDER;
  }

  /* Increase one rotation if we had completed */
  if(steps_motor_left == ENCODER_STEPS){
    steps_motor_left = 0;
    rotations_motor_left++;
  }
}

/* Increment counter of the motor right */
void oldStepCounterMotorRight(){
  steps_motor_right++;

  /* Adjust motor if it's starting move oposite */
  if((steps_motor_right - steps_motor_left) > 3){
    steps_motor_left -= 2;
    pwm_adjust_motor_right -= (pwm_adjust_motor_right > MIN_CHAOS_ALLOWED) ? (pwm_adjust_motor_right) : WITH_MORE_CHAOS_MORE_ORDER;
    pwm_adjust_motor_left += (pwm_adjust_motor_left > MAX_CHAOS_ALLOWED) ? (-pwm_adjust_motor_left) : WITH_MORE_CHAOS_MORE_ORDER;
  }

  /* Increase one rotation if we had completed */
  if(steps_motor_right == ENCODER_STEPS){
    steps_motor_right = 0;
    rotations_motor_right++;
  }
}

/* Motor aceleration test routine */
void oldAcelerationTestRoutine(){

  /* Stop and dettach this routine after execution */
  Timer1.stop();
  Timer1.detachInterrupt();

  /* Brake the motor */
  brake();

  /* Unlock mutex lock */
  routine_flag_mutex_lock = false;
}

/* Aceleration Test */
void oldAcelerationTest(){

  #ifdef ENABLE_LOG
  /* Doing aceleration test */
  printLogn("Doing aceleration test");
  #endif

  /* Clean counters variables */
  steps_motor_left = 0;
  rotations_motor_left = 0;
  steps_motor_right = 0;
  rotations_motor_right = 0;
  
  /* Lock a mutex and acelerate motors with a high speed */
  routine_flag_mutex_lock = true;
  turnForward();
  acelerateMotors(PWM_HIGH_SPEED);

  delay(TIME_1S_MS);
  
  /* Reconfigurate timer */
  Timer1.stop();
  Timer1.attachInterrupt(oldAcelerationTestRoutine);
  Timer1.start();

  /* Wait until it finishes the thread */
  await(routine_flag_mutex_lock);
}

/* Adjust motors to run in the same speed */
void oldAdjustMotors(){

  /* First initialize Timer1 with a routine of 1 second */
  Timer1.initialize(TIME_2S_US);

  int motors_steps_difference = 0;

  /* Compute the difference between these two motors, until it finishes */
  do{

    #ifdef ENABLE_LOG
    /* Start adjust of motors */
    printLogn("Adjusting motors...");
    #endif

    /* Do aceleration test */
    oldAcelerationTest();
 
    /* Total number of steps perfomed by each motor */
    int motor_left_total_steps = (rotations_motor_left * ENCODER_STEPS) + steps_motor_left;
    int motor_right_total_steps = (rotations_motor_right * ENCODER_STEPS) + steps_motor_right;
    
    /* Check the difference between the two motors */
    int motors_rotation_difference = abs(rotations_motor_left - rotations_motor_right);
    motors_steps_difference = abs(motor_left_total_steps - motor_right_total_steps);

    #ifdef ENABLE_LOG
    /* Start adjust of motors */
    printLog("Motor left total steps = ");
    printLogVarn(motor_left_total_steps);
    printLog("Motor right total steps = ");
    printLogVarn(motor_right_total_steps);
    printLog("Motors rotation difference = ");
    printLogVarn(motors_rotation_difference);
    printLog("Motors steps difference = ");
    printLogVarn(motors_steps_difference);
    printLog("Motors pwm adjust motor left = ");
    printLogVarn(pwm_adjust_motor_left);
    printLog("Motors pwm adjust motor right = ");
    printLogVarn(pwm_adjust_motor_right);
    #endif

    /* Until the error between the two motors are close enough */
    if(motors_steps_difference > MAX_STEPS_ERROR){
      
      /* Check which motor should be corrected to the other */
      if(motor_left_total_steps > motor_right_total_steps){
        /* Adjust strength of motor right */
        if(pwm_adjust_motor_right < MAX_PWM_INCREMENT){
          pwm_adjust_motor_right++;
        }
        /* Adjust strength of motor left */
        else if(pwm_adjust_motor_left > MAX_PWM_DECREMENT){
          pwm_adjust_motor_left--;
        }
      }
      else if(motor_right_total_steps > motor_left_total_steps){
        /* Adjust strength of motor right */
        if(pwm_adjust_motor_left < MAX_PWM_INCREMENT){
          pwm_adjust_motor_left++;
        }
        /* Adjust strength of motor left */
        else if(pwm_adjust_motor_right > MAX_PWM_DECREMENT){
          pwm_adjust_motor_right--;
        }
      }
    }
    
    #ifdef ENABLE_LOG
    /* Print Check between error allowed and speed */
    printLog("(motors_steps_difference = ");
    printLogVar(motors_steps_difference);
    printMem(") <= (MAX_STEPS_ERROR = ");
    printLogVar(MAX_STEPS_ERROR);
    printMemn(")");
    #endif

    /* Wait 2S to perform new adjustment */
    delay(TIME_2S_MS);
    
  } while(motors_steps_difference > MAX_STEPS_ERROR);

  /* Stop pwm to speed */
  acelerateMotors(PWM_STOP_SPEED);
}

/* Synchronize motors */
void oldSyncMotors(){
  analogWrite(SPEED_MOTOR_LEFT, actual_pwm_motor_speed + pwm_adjust_motor_left);
  analogWrite(SPEED_MOTOR_RIGHT, actual_pwm_motor_speed + pwm_adjust_motor_right);
}

#endif
