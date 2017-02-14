/*
  Armadillomon
  Armadillomon is a base robot.

  This code example show a base robot controlled using an infrared controller.

  modified 13 Feb 2017
  by Jeferson Lima and Andressa Andrade
 */

#include <IRremote.h> 
#include <SoftwareSerial.h>

/***** USER CONFIG *****/

/* Enable control over serial */
#define SERIAL_CONTROL_ENABLED

/* Enable serial log if you are using serial communication */
#define ENABLE_LOG

/* Enable log on software serial, the default is on main Serial */
//#define SOFTWARE_SERIAL_LOG

/* Enable command using Ir Signal */
//#define IR_CONTROL_ENABLED

/* Enable command using Serial */
#define SERIAL_CONTROL_ENABLED

/***********************/

/* Serial Speed */
#define SERIAL_SPEED          9600

/* Motor Types */
/* Define the types of each motor */
#define MOTOR_LEFT        1
#define MOTOR_RIGHT       2

/* Optical Encoders */
#define ENCODER_LEFT_PIN  2
#define ENCODER_RIGHT_PIN 3

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

/* IR Receiver pin */
#define RECEIVER_PIN      11

/* Software Serial Pins */
#define SOFTWARE_SERIAL_TX_PIN  12
#define SOFTWARE_SERIAL_RX_PIN  13

/* Define the pins of the Ultrasonic Sensors */
#define TRIGGER_PIN       16
#define ECHO_PIN_LEFT     14
#define ECHO_PIN_RIGHT    15

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
#define WHEEL_DIAMETER_MM 75

/* Robot distance between wheels */
#define WHEEL_DISTANCE_MM 100

/* Number of slices of optical encoder */
#define OPTICAL_ENCODER_STEPS 20

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

/* Brake Button */
#define BRAKE_BUTTON_IR     0x8076708F
#define BRAKE_COMMAND       'S'

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

/* Helpers */

/* Acelerate both motors */
#define acelerateMotors(motor_speed) do { acelerateMotor(MOTOR_RIGHT, motor_speed); acelerateMotor(MOTOR_LEFT, motor_speed); } while(false)

/* Turn Forward a given motor */
#define turnForwardMotor(motor_num) changeMotorState(motor_num, HIGH, LOW)

/* Turn Reverse a given motor */
#define turnReverseMotor(motor_num) changeMotorState(motor_num, LOW, HIGH)

/* Brake a given motor */
#define brakeMotor(motor_num) changeMotorState(motor_num, LOW, LOW)

/* Turn both motors foward */
#define turnForward()   do { turnForwardMotor(MOTOR_RIGHT);  turnForwardMotor(MOTOR_LEFT); } while(false)

/* Turn both motors reverse */
#define turnReverse()   do { turnReverseMotor(MOTOR_RIGHT);  turnReverseMotor(MOTOR_LEFT); } while(false)

/* Brake both motors */
#define brake()         do { brakeMotor(MOTOR_RIGHT); brakeMotor(MOTOR_LEFT); } while(false)

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

/* Print Progmem */
#define printMem(MSG)     LOG_OBJECT.println(F(MSG))

#endif

/* IR Receiver */
IRrecv ir_receiver(RECEIVER_PIN);

/* IR Receiver Decoder */
decode_results ir_result;

/* Button State */
long button_state;

/* Selected virtual speed in cm/s */
int motor_speed_virtual_cms;

/* Selected pwm speed */
int actual_pwm_motor_speed;

/* Pwm adjust for each motor */
int pwm_adjust_motor_left;
int pwm_adjust_motor_right;

/* Actual rotation */
int actual_rotation;

/* Number of steps completed */
int steps_motor_left;
int steps_motor_right;

/* Number of rotations completed */
int rotations_motor_left;
int rotations_motor_right;

/* ::::::::::::::::::: Configure the robot ::::::::::::::::::: */

void setup(){
  #ifdef ENABLE_LOG
  /* Initialize serial communication */
  LOG_OBJECT.begin(SERIAL_SPEED);
  #endif

  /* Print Hello Message */
  printMem( "...::: Hi, I'm Armadillomon :) ! :::...\n"
            "By Jeferson Lima & Andressa Andrade\n"
            "Starting System...\n" );
  
  /* Configure all motors related pins as output */
  pinMode(IN1_MOTOR_LEFT, OUTPUT);
  pinMode(IN2_MOTOR_LEFT, OUTPUT);
  pinMode(IN1_MOTOR_RIGHT, OUTPUT);
  pinMode(IN2_MOTOR_RIGHT, OUTPUT);
  pinMode(SPEED_MOTOR_LEFT, OUTPUT);
  pinMode(SPEED_MOTOR_RIGHT, OUTPUT);

  /* Enable IR Receiver */
  ir_receiver.enableIRIn();

  /* Set motor default state */
  brake();

  #ifdef ENABLE_LOG
  /* Print end of setup */
  printMem("System Started Successfully!\n");
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
  
  /* Synchronize motors */
  syncMotors();
}

/* :::::::::::::::::::       Helpers       ::::::::::::::::::: */

/* Increment counter of the motor left */
void stepCounterMotorLeft(){
  steps_motor_left++;

  /* Increase one rotation if we had completed */
  if(steps_motor_left == ){
    
  }
}

/* Increment counter of the motor right */
void stepCounterMotorRight(){
  steps_motor_right++;

  /* Increase one rotation if we had completed */
  if(){
    
  }
}

/* Synchronize motors */
void syncMotors(){
  // TODO
}

/* Check Obstacles */
void checkObstacles(){
  // TODO
}

/* Read Serial data */
void readSerialData(){

  /* If there are serial incoming data */
  if(Serial.available()){
    /* Read serial and execute command */
    char serial_data = Serial.read();

    /* If we have special commands */
    if(serial_data == STEP_COMMAND || serial_data == TURN_COMMAND){
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
    executeCommand((long) serial_data);
  }  
}

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

/* Change Motor State */
void executeCommand(long motor_command){
  
  /* Check the command and change the direction state */
  switch(motor_command){
    case FORWARD_BUTTON_IR:
    case FORWARD_COMMAND:
      turnForward();
      #ifdef ENABLE_LOG
      printLogn("Pressed Forward");
      #endif
      break;
    case REVERSE_BUTTON_IR:
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
    default:
      #ifdef ENABLE_LOG
      printLogn("INVALID COMMAND!\nButton Pressed: ");
      printLogVarn(motor_command);
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
  }
}

/* Change state of enable pin */
void changeMotorState(int motor_num, bool in1_motor_state, bool in2_motor_state){
  switch(motor_num){
    case MOTOR_LEFT:
      digitalWrite(IN1_MOTOR_LEFT, in1_motor_state);
      digitalWrite(IN2_MOTOR_LEFT, in2_motor_state);
      break;
    case MOTOR_RIGHT:
      digitalWrite(IN1_MOTOR_RIGHT, in2_motor_state);
      digitalWrite(IN2_MOTOR_RIGHT, in1_motor_state);
      break;
  }
}
