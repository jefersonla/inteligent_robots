/*
  Armadillomon
  Armadillomon is a base robot.

  This code example show a base robot controlled using an infrared controller.

  modified 13 Feb 2017
  by Jeferson Lima and Andressa Andrade
 */

#include <IRremote.h> 

/***** USER CONFIG *****/

/* Enable control over serial */
#define SERIAL_CONTROL_ENABLED

/***********************/

/* IR Receiver pin */
#define RECEIVER_PIN      8

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

/* Define the pins of the Ultrasonic Sensors */
#define TRIGGER_PIN       16
#define ECHO_PIN_LEFT     14
#define ECHO_PIN_RIGHT    15

/* Robot Configs */

/* Wheel size */
#define WHEEL_SIZE_MM     75

/* Optical Encoder step angle */
#define OPTICAL_ENCODER_GRADE

/* Command buttons */

/* Forward Button */
#define FORWARD_BUTTON_IR   0x8076A05F
#define FORWARD_COMMAND     'F'

/* Reverse Button */
#define REVERSE_BUTTON_IR   0x807620DF
#define REVERSE_COMMAND     'R'

/* Turn Left Button */
#define LEFT_BUTTON_IR      0x8076F807
#define LEFT_COMMAND        'L'

/* Turn Right Button */
#define RIGHT_BUTTON_IR     0x80767887
#define RIGHT_COMMAND       'R'

/* Brake Button */
#define BRAKE_BUTTON_IR     0x8076708F
#define BRAKE_COMMAND       'S'

/* Default speeds */

/* Stop speed */
#define PWM_STOP_SPEED    0

/* Low speed */
#define PWM_LOW_SPEED     100

/* Medium speed */
#define PWM_MEDIUM_SPEED  150

/* High Speed */
#define PWM_HIGH_SPEED    250

/* Helpers */

/* Acelerate both motors */
#define acelerateMotors(motor_speed) do { speed_selected = motor_speed; acelerateMotor(MOTOR_RIGHT, motor_speed); acelerateMotor(MOTOR_LEFT, motor_speed); } while(false)

/* Turn Forward a given motor */
#define turnForwardMotor(motor_num) changeMotorState(motor_num, HIGH, LOW)

/* Turn Reverse a given motor */
#define turnReverseMotor(motor_num) changeMotorState(motor_num, LOW, HIGH)

/* Brake a given motor */
#define brakeMotor(motor_num) changeMotorState(motor_num, HIGH, HIGH)

/* Turn both motors foward */
#define turnForward()  do { turnForwardMotor(MOTOR_RIGHT);  turnForwardMotor(MOTOR_LEFT); } while(false)

/* Turn both motors reverse */
#define turnReverse() do { turnReverseMotor(MOTOR_RIGHT);  turnReverseMotor(MOTOR_LEFT); } while(false)

/* Brake both motors */
#define brake()        do { brakeMotor(MOTOR_RIGHT); brakeMotor(MOTOR_LEFT); } while(false)

/* Step Right */
/* Stop one of the motors and move the other in opossite direction */
#define stepLeft)      do { brakeMotor(MOTOR_RIGHT); turnReverseMotor(MOTOR_LEFT); } while(false)

/* Step Left */
/* Stop one of the motors and move the other in opossite direction */
#define stepRight()    do { brakeMotor(MOTOR_LEFT);  turnReverseMotor(MOTOR_RIGHT); } while(false)

/* Turn Left */
/* Move both motors in opossite directions */
#define turnRight()   do { turnForward(MOTOR_RIGHT); turnReverse(MOTOR_LEFT); } while(false)

/* Turn Right */
/* Move both motors in opossite directions */
#define turnLeft()    do { turnForward(MOTOR_LEFT); turnReverse(MOTOR_RIGHT); } while(false)

/* Log Utilities */

/* Start of a log */
#define LOG_LINE_START "[log] >> "

/* Print Log */
#define printLog(MSG)     Serial.print(F(LOG_LINE_START MSG))
#define printLogn(MSG)    printLog(MSG "\n")
#define printLogVar(VAR)  Serial.print(VAR)
#define printLogVarn(VAR) Serial.println(VAR)

/* Print Progmem */
#define printMem(MSG) Serial.println(F(MSG))

/* IR Receiver */
IRrecv ir_receiver(RECEIVER_PIN);

/* IR Receiver Decoder */
decode_results ir_result;

/* Button State */
long button_state;

/* Ultrasonic Sensors*/
//Ultrasonic ultrasonic_left(TRIGGER_PIN, LECHO_PIN);
//Ultrasonic ultrasonic_right(TRIGGER_PIN, RECHO_PIN);

/* Ultrassonic trigger execution time */
long ultrassonic_timing;

/* Ultrassonic distance read */
float ultrassonic_distance_left;
float ultrassonic_distance_right;

/* Check Obstacles previous time */
long obstacles_previous_time;

/* ::::::::::::::::::: Configure the robot ::::::::::::::::::: */

void setup(){
  /* Initialize serial communication */
  Serial.begin(9600);

  /* Print Hello Message */
  printMem( "...::: Hi, I'm Armadillomon :) ! :::...\n"
            "By Jeferson Lima & Andressa Andrade\n"
            "Starting System...\n" );
  
  /* Configure all motors related pins as output */
  pinMode(IN1_MOTOR1, OUTPUT);
  pinMode(IN2_MOTOR1, OUTPUT);
  pinMode(IN1_MOTOR2, OUTPUT);
  pinMode(IN2_MOTOR2, OUTPUT);
  pinMode(SPE_MOTOR1, OUTPUT);
  pinMode(SPE_MOTOR2, OUTPUT);

  /* Enable IR Receiver */
  ir_receiver.enableIRIn();

  /* Set the default speed of the motors */
  acelerateMotors(MEDIUM_SPEED);

  /* Set motor default state */
  brake();

  /* Obstacles previous time */
  obstacles_previous_time = millis();

  /* Print end of setup */
  printMem("System Started Successfully!\n");
}

/* ::::::::::::::::::: Execute System code ::::::::::::::::::: */

void loop(){

#ifdef IR_CONTROL_ENABLED
  /* Read IR Signal */
  readIRSignal();
#elif SERIAL_CONTROL_ENABLED
  /* Read Serial commands */
  readSerialData();
#endif

  /* Check obstacles */
  checkObstacles();
  
  /* Synchronize motors */
  syncMotors();
}

/* :::::::::::::::::::       Helpers       ::::::::::::::::::: */

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
    executeCommand((int) serial_data);
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
void executeCommand(int motor_command){
  
  /* Check the command and change the direction state */
  switch(motor_command){
    case UP_BUTTON:
    case UP_COMMAND:
      turnForward();
      printLogn("Pressed UP");
      break;
    case BOTTOM_BUTTON:
    case BOTTOM_COMMAND:
      turnBackward();
      printLogn("Pressed DOWN");
      break;
    case LEFT_BUTTON:
    case LEFT_COMMAND:
      stepLeft();
      printLogn("Pressed LEFT");
      break;
    case RIGHT_BUTTON:
    case RIGHT_COMMAND:
      stepRight();
      printLogn("Pressed RIGHT");
      break;
    case BRAKE_BUTTON:
    case BRAKE_COMMAND:
      brake();
      printLogn("Pressed BRAKE");
      break;
    default:
      printLogn("INVALID COMMAND!\nButton Pressed: ");
      Serial.println(motor_command);
  }
}

/* Acelerate a given motor */
void acelerateMotor(int motor_num, int motor_speed){
  switch(motor_num){
    case MOTOR_LEFT:
      analogWrite(SPE_MOTOR1, motor_speed);
      break;
    case MOTOR_RIGHT:
      analogWrite(SPE_MOTOR2, motor_speed);
      break;
  }
}

/* Change state of enable pin */
void changeMotorState(int motor_num, bool in1_motor_state, bool in2_motor_state){
  switch(motor_num){
    case MOTOR_LEFT:
      digitalWrite(IN1_MOTOR1, in1_motor_state);
      digitalWrite(IN2_MOTOR1, in2_motor_state);
      break;
    case MOTOR_RIGHT:
      digitalWrite(IN1_MOTOR2, in2_motor_state);
      digitalWrite(IN2_MOTOR2, in1_motor_state);
      break;
  }
}
