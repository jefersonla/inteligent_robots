#include <IRremote.h> 

/* Receiver pin */
#define RECEIVER_PIN 8

/* Motor Types */
#define MOTOR_RIGHT 1
#define MOTOR_LEFT  2

/* Enable of Motor 1 */
#define IN1_MOTOR1 4
#define IN2_MOTOR1 5

/* Enable of Motor 2 */
#define IN1_MOTOR2 6
#define IN2_MOTOR2 7

/* Speed of each motor */
#define SPE_MOTOR1 9
#define SPE_MOTOR2 10

/* Command buttons */

/* Up Button */
#define UP_BUTTON     0x8076A05F

/* Bottom Button */
#define BOTTOM_BUTTON 0x807620DF

/* Left Button */
#define LEFT_BUTTON   0x8076F807

/* Right Button */
#define RIGHT_BUTTON  0x80767887

/* Brake Button */
#define BRAKE_BUTTON  0x8076708F

/* Default speeds */

/* Stop speed */
#define STOP_SPEED 0

/* Low speed */
#define LOW_SPEED 100

/* Medium speed */
#define MEDIUM_SPEED 150

/* High Speed */
#define HIGH_SPEED 250

/* Acelerate both motors */
#define acelerateMotors(motor_speed) do { acelerateMotor(MOTOR_RIGHT, motor_speed); acelerateMotor(MOTOR_LEFT, motor_speed); } while(false)

/* Turn Forward a given motor */
#define turnForwardMotor(motor_num) changeMotorState(motor_num, LOW, HIGH)

/* Turn Backward a given motor */
#define turnBackwardMotor(motor_num) changeMotorState(motor_num, HIGH, LOW)

/* Brake a given motor */
#define brakeMotor(motor_num) changeMotorState(motor_num, HIGH, HIGH)

/* Turn both motors foward */
#define turnForward()  do { acelerateMotors(MEDIUM_SPEED); turnForwardMotor(MOTOR_RIGHT);  turnForwardMotor(MOTOR_LEFT); } while(false)

/* Turn both motors backward */
#define turnBackward() do { acelerateMotors(MEDIUM_SPEED); turnBackwardMotor(MOTOR_RIGHT); turnBackwardMotor(MOTOR_LEFT); } while(false)

/* Brake both motors */
#define brake()        do { acelerateMotors(STOP_SPEED);   brakeMotor(MOTOR_RIGHT); brakeMotor(MOTOR_LEFT); } while(false)

/* Step Right */
#define stepRight()    do { acelerateMotors(LOW_SPEED);    brakeMotor(MOTOR_RIGHT); turnForwardMotor(MOTOR_LEFT); } while(false)

/* Step Left */
#define stepLeft()     do { acelerateMotors(LOW_SPEED);    brakeMotor(MOTOR_LEFT);  turnForwardMotor(MOTOR_RIGHT); } while(false)

/* IR Receiver */
IRrecv ir_receiver(RECEIVER_PIN);

/* IR Receiver Decoder */
decode_results ir_result;

/* Button State */
long button_state;

void setup(){
  /* Initialize serial communication */
  Serial.begin(9600);

  /* Print Hello Message */
  Serial.println("Hi, I'm Armadillomon :) !");
  
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
}

void loop(){
  /* Verify if there are Ir signal */
  if(ir_receiver.decode(&ir_result)){

    /* Get the button pressed */
    button_state = ir_result.value;

    /* Check the command and change the direction state */
    switch(button_state){
      case UP_BUTTON:
        turnForward();
        break;
      case BOTTOM_BUTTON:
        turnBackward();
        break;
      case LEFT_BUTTON:
        stepLeft();
        break;
      case RIGHT_BUTTON:
        stepRight();
        break;
      case BRAKE_BUTTON:
        brake();
        break;
      default:
        Serial.print("INVALID BUTTON!\nButton Pressed: ");
        Serial.println(button_state);
    }

    /* Flush the ir receiver buffer */
    ir_receiver.resume();
  }
}

/* Acelerate a given motor */
void acelerateMotor(int motor_num, int motor_speed){
  switch(motor_num){
    case MOTOR_RIGHT:
      analogWrite(SPE_MOTOR1, motor_speed);
      break;
    case MOTOR_LEFT:
      analogWrite(SPE_MOTOR2, motor_speed);
      break;
  }
}

/* Change state of enable pin */
void changeMotorState(int motor_num, bool in1_motor_state, bool in2_motor_state){
  switch(motor_num){
    case MOTOR_RIGHT:
      digitalWrite(IN1_MOTOR1, in1_motor_state);
      digitalWrite(IN2_MOTOR1, in2_motor_state);
      break;
    case MOTOR_LEFT:
      digitalWrite(IN1_MOTOR2, in1_motor_state);
      digitalWrite(IN2_MOTOR2, in2_motor_state);
      break;
  }
}
