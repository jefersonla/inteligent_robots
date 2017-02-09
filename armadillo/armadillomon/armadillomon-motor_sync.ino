/*
  Armadillomon
  Armadillomon is a base robot.

  This code example show a base robot controlled using an infrared controller.

  modified 15 Dec 2016
  by Jeferson Lima and Andressa Andrade
 */

#include <IRremote.h> 
//#include <Ultrasonic.h>

/* Receiver pin */
#define RECEIVER_PIN 8

/* Motor Types */
#define MOTOR_RIGHT 1
#define MOTOR_LEFT  2

/* Enable of Motor 1 */
#define IN1_MOTOR1 7
#define IN2_MOTOR1 8

/* Enable of Motor 2 */
#define IN1_MOTOR2 10
#define IN2_MOTOR2 9

/* Speed of each motor */
#define SPE_MOTOR1 6
#define SPE_MOTOR2 5

/* Interruption motor 1 */
#define INT_MOTOR1_PIN  2

/* Interruption motor 2 */
#define INT_MOTOR2_PIN  3

/* Time to sinchronyze motors */
#define MOTOR_SYNCHRONIZE_INTERVAL ((unsigned long)2000UL)

/* Define Ultrasonics's pin */
#define TRIGGER_PIN 16
#define RECHO_PIN   15
#define LECHO_PIN   14 

/* Command buttons */

/* Up Button */
#define UP_BUTTON     0x8076A05F
#define UP_COMMAND    'F'

/* Bottom Button */
#define BOTTOM_BUTTON 0x807620DF
#define BOTTOM_COMMAND 'B'

/* Left Button */
#define LEFT_BUTTON   0x8076F807
#define LEFT_COMMAND  'L'

/* Right Button */
#define RIGHT_BUTTON  0x80767887
#define RIGHT_COMMAND 'R'

/* Brake Button */
#define BRAKE_BUTTON  0x8076708F
#define BRAKE_COMMAND 'S'

/* Default speeds */

/* Stop speed */
#define STOP_SPEED 0

/* Low speed */
#define LOW_SPEED 70

/* Medium speed */
#define MEDIUM_SPEED 120

/* High Speed */
#define HIGH_SPEED 250

/* Check Obstacles Interval */
#define CHECK_OBSTACLES_INTERVAL 500

/* Acelerate both motors */
#define acelerateMotors(motor_speed) do { acelerateMotor(MOTOR_RIGHT, motor_speed + adjust_motor_right); acelerateMotor(MOTOR_LEFT, motor_speed + adjust_motor_left); } while(false)

/* Turn Forward a given motor */
#define turnForwardMotor(motor_num) changeMotorState(motor_num, HIGH, LOW)

/* Turn Backward a given motor */
#define turnBackwardMotor(motor_num) changeMotorState(motor_num, LOW, HIGH)

/* Brake a given motor */
#define brakeMotor(motor_num) changeMotorState(motor_num, HIGH, HIGH)

/* Turn both motors foward */
#define turnForward()  do { acelerateMotors(MEDIUM_SPEED); turnForwardMotor(MOTOR_RIGHT);  turnForwardMotor(MOTOR_LEFT); } while(false)

/* Turn both motors backward */
#define turnBackward() do { acelerateMotors(MEDIUM_SPEED); turnBackwardMotor(MOTOR_RIGHT); turnBackwardMotor(MOTOR_LEFT); } while(false)

/* Brake both motors */
#define brake()        do { acelerateMotors(STOP_SPEED);   brakeMotor(MOTOR_RIGHT); brakeMotor(MOTOR_LEFT); } while(false)

/* Step Right */
#define stepRight()    do { acelerateMotors(LOW_SPEED);    brakeMotor(MOTOR_RIGHT); turnBackwardMotor(MOTOR_LEFT); } while(false)

/* Step Left */
#define stepLeft()     do { acelerateMotors(LOW_SPEED);    brakeMotor(MOTOR_LEFT);  turnBackwardMotor(MOTOR_RIGHT); } while(false)

/* Log Utilities */

/* Start of a log */
#define LOG_LINE_START "[log] >> "

/* Print Log */
#define printLog(MSG) Serial.print(F(LOG_LINE_START MSG))
#define printLogn(MSG) printLog(MSG "\n")

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

/* Number of cycles of each motor */
volatile long cycles_motor1 = 0;
volatile long cycles_motor2 = 0;
void increaseCyclesMotor1();
void increaseCyclesMotor2();

/* Current and previous time */
unsigned long previous_millis;
unsigned long current_millis;

/* Initialize adjust motor speed */
int adjust_motor_right;
int adjust_motor_left;

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
 
  /* Initialize adjust motor speed */
  adjust_motor_right = 0;
  adjust_motor_right = 1;

  /* Set the default speed of the motors */
  acelerateMotors(MEDIUM_SPEED);

  /* Set motor default state */
  brake();

  /* Obstacles previous time */
  obstacles_previous_time = millis();

  /* Add interruptions */
  attachInterrupt(digitalPinToInterrupt(INT_MOTOR1_PIN), increaseCyclesMotor1, FALLING);
  attachInterrupt(digitalPinToInterrupt(INT_MOTOR2_PIN), increaseCyclesMotor2, FALLING);

  /* Initialize Previous millis */
  previous_millis = millis();

  /* Setup the motors */
  setupMotors();

  /* Print end of setup */
  printMem("System Started Successfully!\n");
}

/* ::::::::::::::::::: Execute System code ::::::::::::::::::: */

void loop(){
  
  /* Read IR Signal */
  //readIRSignal();

  /* Read Serial commands */
  readSerialData();

  /* Check obstacles */
  //checkObstacles();
  
  /* Synchronize motors */
  syncMotors();
}

/* :::::::::::::::::::       Helpers       ::::::::::::::::::: */

/* Setup Motors */
void setupMotors(){

  /* Start of this action */
  unsigned long start_motors = millis();
  
  /* Ignite motors */
  turnForward();

  /* While one of the motors don't reach one revolution */
  while(cycles_motor1 < 20 && cycles_motor2 < 20);

  /* Stop interruption */
  cli();

  /* End of this action */
  unsigned long stop_motors = millis();

  /* Brake the motors */
  brake();

  /* Compare the speeds */
  if(cycles_motor1 != cycles_motor2){
    if(cycles_motor1 > cycles_motor2){
      adjust_motor_right = ((2000 / cycles_motor2) * MEDIUM_SPEED) / 100;
    }
    else{
      adjust_motor_left = ((2000 / cycles_motor1) * MEDIUM_SPEED) / 100;
    }
  }

  /* Clear the motor cycles count */
  cycles_motor1 = 0;
  cycles_motor2 = 0;

  /* Start interruption */
  sei();
}

/* Synchronize motors */
void syncMotors(){
  current_millis = millis();
  
  if(current_millis - previous_millis > MOTOR_SYNCHRONIZE_INTERVAL){
    Serial.print(F("Motor 1 - Count Cycles = "));
    Serial.println(cycles_motor1);
    Serial.print(F("Motor 2 - Count Cycles = "));
    Serial.println(cycles_motor2);
    previous_millis = millis();
  }
}

/* Check Obstacles */
void checkObstacles(){

  /* Only check for obstacles in a given ammount of time */
  if((millis() - obstacles_previous_time) > CHECK_OBSTACLES_INTERVAL){
    
    /* Trigger the sound of the left ultrassonic sensor */
    //ultrassonic_timing = ultrasonic_left.timing();
  
    /* Measure distance of the left ultrassonic sensor */
    //ultrassonic_distance_left = ultrasonic_left.convert(ultrassonic_timing, Ultrasonic::CM);
    
    /* Trigger the sound of the right ultrassonic sensor */
    //ultrassonic_timing = ultrasonic_right.timing();
  
    /* Measure distance of the right ultrassonic sensor */
    //ultrassonic_distance_right = ultrasonic_right.convert(ultrassonic_timing, Ultrasonic::CM);

    /* Send data over serial */
    Serial.print(F("{\n\"distance_right\":"));
    Serial.print(ultrassonic_distance_right);
    Serial.print(F(",\n\"distance_left\":"));
    Serial.println(ultrassonic_distance_left);
    printMem("}");
  }
  
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
      digitalWrite(IN1_MOTOR2, in1_motor_state);
      digitalWrite(IN2_MOTOR2, in2_motor_state);
      break;
  }
}

/* Increase the number of cycles of each motor */
void increaseCyclesMotor1(){
  cycles_motor1++;
}

void increaseCyclesMotor2(){
  cycles_motor2++;
}
