#include <PinChangeInt.h>  //Pin Change Interrupts Library
#include <MsTimer2.h>      //Timer Interrupts Library

//Encoder Pin Definitions
#define ENCODER_PIN_A 4  //Encoder pin A
#define ENCODER_PIN_B 2  //Encoder pin B

//TB6612 Motor Driver Pin Definitions
#define MOTOR_PWM 9       //PWM pin for speed control
#define MOTOR_PIN_ONE 10  //Input 1 for motor direction control
#define MOTOR_PIN_TWO 11  //Input 2 for motor direction control

#define TARGET_ANGLE 816  //Target potentiometerreading (upright)

//Global Variables
int TARGET_POSITION = 0;  //Target position
int currentPosition = 0;  //Current cart position
float sensorReading;      //Potentiometer reading (0 - 1023)
float controlValue;       //Motor control value containing direction and PWM
int anglePIDValue;        //PWM for angle
int cartPIDValue;         //PWM for position
int newCenter = 0;        //Calibrated center of track
float theta;              //Angle in degrees
float time = 0;           //Run time
int newAngle = 0;         //New angle to check for free-fall
int controlTime = 0;      //5ms check to keep control loop on scheduable
int prevTime = 0;         //Previous control loop end time

//CALIBRATION WITH NO VOLTAGE DIVIDER:
float IP_UP = 816;
float IP_DOWN = 277;
float DEGREES_PER_READING = 2.8416;
float MAX_READING = 1023;

//PID Tunings:
float angleKp = 45;
float angleKi = 0.6;
float angleKd = 0.003;

float cartKp = 0.15;
float cartKi = 0;
float cartKd = 0.06;

//Swing-Up States
enum State {
  INITIAL_IMPULSE,   //In initial impulse state
  SWING_UP,          //Main swing-up state
  CATCH_AND_CONTROL  //Catch and control pendulum state
};

State swingUpStates = INITIAL_IMPULSE;  //Start state
int startSpeed = 110;                   //Swing-up starting speed

void setup() {
  /*  
  *Push Timer1 to maximum frequency for smoother PWM
  *Modify Timer1 prescaler to 1
  *This tells Timer1 to count at the full system clock speed  
  */
  TCCR1B = (TCCR1B & 0xF8) | 1;  //Adjust the counter division to increase the frequency to 31.374KHZ

  //Initialize pins/baud rate
  pinMode(MOTOR_PIN_ONE, OUTPUT);  //TB6612 direction control pin 1
  pinMode(MOTOR_PIN_TWO, OUTPUT);  //TB6612 direction control pin 2
  pinMode(MOTOR_PWM, OUTPUT);      //TB6612 speed control pin (PWM)
  digitalWrite(MOTOR_PIN_ONE, 0);  //TB6612 control pin is pulled low at start of program
  digitalWrite(MOTOR_PIN_TWO, 0);  //TB6612 control pin is pulled low at start of program
  digitalWrite(MOTOR_PWM, 0);      //TB6612 control pin is pulled low at start of program
  pinMode(ENCODER_PIN_A, INPUT);   //Encoder pin
  pinMode(ENCODER_PIN_B, INPUT);   //Encoder pin
  Serial.begin(115200);            //Set baud rate (115200)


  //Interrupt to read encoder on pin change
  attachInterrupt(0, readEncoder, CHANGE);  //Enable external interrupt (Pin 2 - CHANGE)

  //Center cart and set target position:
  delay(200);
  findCenter();  //Locate the start of the track
  delay(200);
  TARGET_POSITION = newCenter - 500;  //Set target cart positiom
  Serial.println(currentPosition);
  Serial.println(newCenter);

  delay(2000);
  findCenter();  //Call findCenter() again to create initial impulse for swing-up
}

void loop() {

  sensorToDegrees(analogRead(A5));  //Take angle reading (in degrees for simplicity)

  //Print to PuTTy for Plotting (comment out when not recording data):
  time = millis();
  Serial.print(time - 6000);//Time since system was restart - 5 seconds for initial set up
  Serial.print(",");
  Serial.print(theta - 9);  //Angle in degrees + adjustment for uneven potentiomter reading due to location and size of dead zone
  Serial.print(",");
  Serial.print(((currentPosition - TARGET_POSITION) * 0.219) + 171);  //Current position (shifted to +180 to match angle target for clearer plots)
  Serial.println("");

  switch (swingUpStates) {              //Switch statements for states of swing up manoeuvre
    case INITIAL_IMPULSE:               //Initial impulse state
      if (theta < 345 || theta > 15) {  //If pendulums swing is at least 15 degrees from stationary
        swingUpStates = SWING_UP;       //Switch case to swing-up controller
        startSpeed = 100;               //Set the base swing-up motor speed
        Serial.println("BREAK");
      }
      break;

    case SWING_UP:                       //Swing-up state
      if (theta < 360 && theta > 355) {  //If pendulum passes by 0 degree p[ostion from left side
        moveMotor(-1 * startSpeed);      //Reverse cart to build momentum
        swingUpStates = SWING_UP;        //Stay in swing-up state
      }

      else if (theta > 0 && theta < 5) {  //If pendulum passes by 0 degree p[ostion from right side
        moveMotor(1 * startSpeed);        //Reverse cart to build momentum
        swingUpStates = SWING_UP;         //Stay in swing-up state
      }

      else if (theta > 178 && theta < 200) {           //If pendulum is within 11 degrees of target
        if (theta < 189) {                             //If pendulum is on right side
          delay(30);                                   //Delay 30ms to record new angle
          newAngle = sensorToDegrees(analogRead(A5));  //Record new angle
          if (theta > newAngle) {                      //If new angle is less than old angle, pendulum is in free-fall
            swingUpStates = CATCH_AND_CONTROL;         //Switch case to catch and control
            moveMotor(0);                              //Turn off motor
          }
        } else {
          delay(30);                                   //Delay 30ms to record new angle
          newAngle = sensorToDegrees(analogRead(A5));  //Record new angle
          if (theta < newAngle) {                      //If new angle is greater than old angle, pendulum is in free-fall
            swingUpStates = CATCH_AND_CONTROL;         //Switch case to catch and control
            moveMotor(0);                              //Turn off motor
          }
        }
      }

      if ((theta > 150 + 9 && theta < 189) || (theta > 189 && theta < 210 + 9)) {  //If pendulum is within 30 degrees of target (upright)
        startSpeed = 85;                                                           //Slow the cart velocity to help with catch case
      }
      break;

    //This case acts similarly to the MsTimer2 interrupts used in the previous code
    //It will call the control loop every 5ms to keep the pendulum upright
    case CATCH_AND_CONTROL:               //Catch and control state
      controlTime = millis();             //Start of control loop time
      if (controlTime - prevTime >= 5) {  //If change in time is at least 5ms
        parallelPIDControl();             //Call PID controller
        prevTime = controlTime;           //Update time since last controller call
      }
      break;
  }
}

/*
This function is the main system control loop
- It calls all neccessary functions to control the IP
- Begins by taking an average angle
- Converts angle to degrees purely for ploting, keeps original unit for control
- Calculates angle PID
- Sample and holds position PID value
- Determines motor direction and PWM
- Moves cart
*/
void parallelPIDControl() {
  static unsigned char cartSampleCount;  //Sample and hold count
  sensorReading = sensorAverage(5, 5);   //Reads and calculates 5 potentiometer readings and takes an average

  sensorToDegrees(sensorReading);  //Converts angle reading to degrees for plotting

  anglePIDValue = anglePIDController(sensorReading);  //Calculates angle PID value

  if (++cartSampleCount > 4) {                          //Sample and hold (gathers 5 balance values before a new position value will be set)
    cartPIDValue = cartPIDController(currentPosition);  //Calculates position PID value
    cartSampleCount = 0;                                //Resets sample and hold count
  }

  controlValue = anglePIDValue - cartPIDValue;  //Calculates control value (direction + PWM)

  if (turnOffMotor() == false) {  //If Turn_Off conditions are not met
    moveMotor(controlValue);      //Turn on motor
  }
}

/*
Function to calculate the angle PID value
- Takes in a reading/average reading from the potentiometer
- Computes the proportional, integral and derivative errors
- Returns a PID output value based on the global tunings
*/
float anglePIDController(float sensorReading) {
  static float oldGainError;     //Variables to be remembered for next function call
  float pError, iError, dError;  //Errors
  float timeChange = 0.005;      //Change in time to calculate integral and derivative errors
  int anglePID;                  //PID output

  int tolerance = -1;  //Allow +1 tolerance for noisy sensor

  if (sensorReading - TARGET_ANGLE == tolerance) {  //If sensor reading is +1 off
    pError = 0;                                     //Set error to 0 (consider target met)
  } else {
    pError = sensorReading - TARGET_ANGLE;  //Else compute error as normal
  }

  iError += pError * timeChange;                  //Calculate integral error
  iError = constrain(iError, -50, 50);            //Contsrain integral error
  dError = (pError - oldGainError) / timeChange;  //Calculate derivative error
  oldGainError = pError;                          //Set previous gain error

  anglePID = (pError * angleKp) + (iError * angleKi) + (dError * angleKd);  //Compute PID output

  return anglePID;  //Return control value
}

/*
Function to calculate the position PID value
- Takes in the carts position (global variable)
- Computes proportional, integral and derivative errors
- Filters error so angle PID has much heavier weighting on final control value
- Returns a PID output value based on the global tunings
*/
float cartPIDController(int position) {
  static float oldPositionError, pErrorFiltered;  //Variables to be remembered for next function call
  float pError, iError, dError;                   //Errors
  float timeChange = 0.005;                       //Change in time to calculate integral and derivative errors
  int cartPID;                                    //PID output

  pError = position - TARGET_POSITION;                        //Calculate gain error
  pErrorFiltered *= 0.8;                                      //First-order low-pass filter (80% previous value retained)
  pErrorFiltered += pError * 0.2;                             //First-order low-pass filter (20% previous value retained)
  iError += pError * timeChange;                              //Calculate integral error
  iError = constrain(iError, -50, 50);                        //Constrain integral error
  dError = (pErrorFiltered - oldPositionError) / timeChange;  //Calculate derivative error
  oldPositionError = pErrorFiltered;                          //Set previous gain error

  cartPID = (pErrorFiltered * cartKp) + (iError * cartKi) + (dError * cartKd);  //Calculate PID output

  return cartPID;  //Return control value
}

/*
Function moves the cart, the speed of the cart is dependannt on 'motor'
- Motor value < 0 will move the cart to the left
- Motor value > 0 will move the cart to the right
- AnalogWrite sets the Pulse Width Modulation (PWM) duty cycle (modulated from 0 - 255)
- The PWM determines how much power is supplied to the motor, it makes the motor move while the digitalWrites just determine direction
*/
void moveMotor(int motor) {
  //Checks to see if motor value exceeds hardware limit
  if (motor > 255) {          //If control value is too high
    motor = 255;              //Limits PWM
  } else if (motor < -255) {  //Else if control value is too low
    motor = -255;             //Limits PWM
  }

  if (motor < 0) {                                                        //If control value is greater than 0
    digitalWrite(MOTOR_PIN_ONE, HIGH), digitalWrite(MOTOR_PIN_TWO, LOW);  //Move cart
  } else {
    digitalWrite(MOTOR_PIN_ONE, LOW), digitalWrite(MOTOR_PIN_TWO, HIGH);  //Move cart
  }
  analogWrite(MOTOR_PWM, abs(motor));  //Assign control gain to PWM register
}

/*
This function will set a flag to either turn on or off the motor in the control loop
- Takes no inputs as it relys on the global angle
- Checks if angle is outside boundaries
- Turns off motor if neccessary, sets false flag state if not (turn on motor)
*/
bool turnOffMotor() {
  bool motorState;                                                                     //On/off flag
  if (sensorReading < (TARGET_ANGLE - 100) || sensorReading > (TARGET_ANGLE + 100)) {  //If angle reading is outside range or manual turn off enabled
    motorState = true;                                                                 //Turn off flag
    digitalWrite(MOTOR_PWM, LOW);                                                      //Turn off motor
  } else {
    motorState = false;  //Turn on flag
  }
  return motorState;  //Returns current motor flag
}

/*
This function returns an average value of potentiometer readings for increase accuracy
- u8 ANALOG_PIN: always 5 because A5 is sensor
- u8 numberOfSamples: how many readings are included in average
*/
u16 sensorAverage(u8 ANALOG_PIN, u8 numberOfSamples) {
  unsigned int sensorReadingSum = 0;  //Sum of potentiometer readings
  for (unsigned char t = 0; t < numberOfSamples; t++) {
    sensorReadingSum += analogRead(ANALOG_PIN);  //Add up readings
  }
  return sensorReadingSum / numberOfSamples;  //Return average
}

float sensorToDegrees(float recordedAngle) {
  if (recordedAngle >= IP_DOWN) {                                             //If recorded angle is in the 3 quartes anti clockwise from down
    theta = (recordedAngle - IP_DOWN) / DEGREES_PER_READING;                  //The calibrated angle should be the recorded angle - distance from actual 0 to down (275)
  } else {                                                                    //If recorded angle is the the bottom left quarted (between 1023 and 257 counting up)
    theta = ((MAX_READING - IP_DOWN) + recordedAngle) / DEGREES_PER_READING;  //The calibrated angle should be (max value - down position) + the recorded angle
  }
  return theta;
}

/*
Function reads the encoder and increments a global variable to determine the cart position
- External interrupt to call this function is triggered by transition edges
- Checks all possible combinations of movement and updates currentPosition accordingly
*/
void readEncoder() {
  if (digitalRead(ENCODER_PIN_A) == HIGH) {  //If triggered by rising edge A
    if (digitalRead(ENCODER_PIN_B) == LOW) {
      currentPosition++;  //Increment position
    } else {
      currentPosition--;  //Decrement position
    }
  } else {  //If triggered by rising edge B
    if (digitalRead(ENCODER_PIN_B) == LOW) {
      currentPosition--;  //Decrement position
    } else {
      currentPosition++;  //Increment position
    }
  }
}

/*
This function is used to move the cart to the center of the track
- It achieves this by first moving to the right edge and then moving the required number of ticks backwards to reach the center
- Edge detection is determined by comparing position readings at different times to check if the cart has stopped
*/
void findCenter() {
  int posiBreak = 1;     //Flag to say boundary found
  int posiOne, posiTwo;  //Position readings

  while (posiBreak == 1) {      //While boundary not found
    moveMotor(220);             //Move cart left at moderate speed
    posiOne = currentPosition;  //Read current position
    Serial.println("");
    delay(200);                 //Delay to give cart a chance to move
    posiTwo = currentPosition;  //Read new current position
    Serial.println("");

    if (posiOne == posiTwo) {        //If position readings match (cart has hit boundary and stopped)
      digitalWrite(MOTOR_PWM, LOW);  //Set speed to 0
      posiBreak = 0;                 //Flag that boundary has been found
      Serial.println("Found Edge");
    }
  }

  while (posiBreak == 0) {                     //While boundary has been found
    if (posiOne < 0) {                         //Executes if cart has hit left side of track
      moveMotor(-200);                         //Move cart right at moderate speed
      if (currentPosition == posiOne + 900) {  //If cart reaches center of track
        digitalWrite(MOTOR_PWM, LOW);          //Turn off motor
        posiBreak = 2;                         //Break out of loop
        newCenter = posiOne + 900;             //Sets center of track
      }
    } else {  //Executes when cart hits right side (additional functionality if desired)
      moveMotor(200);
      if (currentPosition == posiOne - 900) {  //If cart reaches center of track
        digitalWrite(MOTOR_PWM, LOW);          //Turn off motor
        posiBreak = 2;                         //Break out of loop
        newCenter = posiOne - 900;             //Sets center of track
      }
    }
  }
}