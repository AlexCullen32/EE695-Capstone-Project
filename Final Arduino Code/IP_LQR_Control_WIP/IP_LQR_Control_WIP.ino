#include <PinChangeInt.h>  //Pin Change Interrupts Library
#include <MsTimer2.h>      //Timer Interrupts Library

//Encoder Pin Definitions
#define ENCODER_PIN_A 4  //Encoder pin A
#define ENCODER_PIN_B 2  //Encoder pin B

//TB6612 Motor Driver Pin Definitions
#define MOTOR_PWM 9         //PWM pin for speed control
#define MOTOR_PIN_ONE 10    //Input 1 for motor direction control
#define MOTOR_PIN_TWO 11    //Input 2 for motor direction control

#define TARGET_ANGLE 816    //Target potentiometerreading (upright)

int TARGET_POSITION = 0;    //Target position

float currentPosition = 0;  //Current cart position
float sensorReading;        //Potentiometer reading (0 - 1023)
float controlValue;         //Motor control value containing direction and PWM


//CALIBRATION WITH NO VOLTAGE DIVIDER:
float IP_UP = 816;
float IP_DOWN = 310;
float DEGREES_PER_READING = 2.8416;
float MAX_READING = 1023;

float theta;                                          //Angle in degrees

float POSITION_CURRENT = 0, POSITION_PREVIOUS = 0;    //Position storage
float xDot;                                           //Cart velocity
unsigned long prevTimePosition = 0;                   //Old position time

float ANGLE_CURRENT = 0, ANGLE_PREVIOUS = 0;          //Angle storage
float thetaDot;                                       //Angular velocity
unsigned long prevTimeAngle = 0;                      //Old angle time


//Q1=0.001
//Q2=0.000001
//Q3=27
//Q4=0.00001
//R = 0.005
float k[4] = {-0.4472,   -3.4156,   78.9501,    6.0472};    //Gain matrix

//ERRORS:
float currentPosition_ERROR;
float xDot_ERROR;
float angle_ERROR;
float thetaDot_ERROR;


void setup() {
  /*
  *Push Timer1 to maximum frequency for smoother PWM
  *Modify Timer1 prescaler to 1
  *This tells Timer1 to count at the full system clock speed
  */
  TCCR1B = (TCCR1B & 0xF8) | 1;  //Adjust the counter division to increase the frequency to 31.374KHZ

  //Initialize pins/baud rate
  pinMode(MOTOR_PIN_ONE, OUTPUT);   //TB6612 direction control pin 1
  pinMode(MOTOR_PIN_TWO, OUTPUT);   //TB6612 direction control pin 2
  pinMode(MOTOR_PWM, OUTPUT);       //TB6612 speed control pin (PWM)
  digitalWrite(MOTOR_PIN_ONE, 0);   //TB6612 control pin is pulled low at start of program
  digitalWrite(MOTOR_PIN_TWO, 0);   //TB6612 control pin is pulled low at start of program
  digitalWrite(MOTOR_PWM, 0);       //TB6612 control pin is pulled low at start of program
  pinMode(ENCODER_PIN_A, INPUT);    //Encoder pin
  pinMode(ENCODER_PIN_B, INPUT);    //Encoder pin
  Serial.begin(115200);             //Set baud rate (115200)

  attachInterrupt(0, readEncoder, CHANGE);  //Enable external interrupt (Pin 2 - CHANGE)

  delay(200);                            //Delay to wait for initialization to complete
  MsTimer2::set(5, LQRControl);  //Use Timer2 to set up 5ms timer interrupt
  MsTimer2::start();                     //Interrupt enable

  Serial.print("\n\n");
}

void loop() {

  //Print to PuTTY for Plotting (comment out when not recording data):
  int time = millis();
  Serial.print(time - 6000);  //Time since system was restart - 5 seconds for initial set up
  Serial.print(",");
  Serial.print(theta + 2);    //Angle in degrees + adjustment for uneven potentiomter reading due to location and size of dead zone
  Serial.print(",");
  Serial.print(((currentPosition - TARGET_POSITION) * 0.219) + 171);  //Current position (shifted to +180 to match angle target for clearer plots)
  Serial.println("");

//Print States for Testing:
/*
  float state[4 * 1] = {
    currentPosition,
    xDot,
    theta,
    thetaDot
  };

  Serial.print("Position: ");
  Serial.print(state[0]);
  Serial.print(", Velocity: ");
  Serial.print(state[1]);
  Serial.print(", Angle: ");
  Serial.print(state[2]);
  Serial.print(", Angular Velocity: ");
  Serial.print(state[3]);
  Serial.print(", Control: ");
  Serial.println(controlValue);
*/
}

/*
LQR control function
- This function is triggered by an intteruopt every 5ms
- Calculates state errors and multiplies by the gain matrix to obtain control value
- Turns off motor
*/
void LQRControl() {

  calculate_X_Dot();                //Calculate cart velocity
  sensorToDegrees(analogRead(A5));  //Read angle in degrees
  calculate_Theta_Dot();            //Calculate angular velocity

  currentPosition_ERROR = (currentPosition - 0)* 0.219; //Calculate cart poistion error
  xDot_ERROR = (xDot - 0)* 0.219;                       //Calculate cart velocity error
  angle_ERROR = theta - 180;                            //Calculate pendulum angle velocity
  thetaDot_ERROR = thetaDot - 0;                        //Calculate angular velocity

  float stateErrors[4 * 1] = {                          //State errors
    currentPosition_ERROR,
    xDot_ERROR,
    angle_ERROR,
    thetaDot_ERROR
  };

  //LQR Control Law:
  controlValue = (-1 * (k[0] * stateErrors[0] + k[1] * stateErrors[1] + k[2] * stateErrors[2] + k[3] * stateErrors[3]));

  moveMotor(controlValue);   //Turn on motor
}

/*
Calculates cart velocity
*/
void calculate_X_Dot() {
  unsigned long currentTime = millis();
  float deltaT = (currentTime - prevTimePosition) / 1000;  //Convert to seconds
  if (deltaT > 0) {                                        //Ensure no dividing by 0
    POSITION_CURRENT = currentPosition;                    //Current position
    xDot = (POSITION_CURRENT - POSITION_PREVIOUS) / deltaT;//Calculate cart velocity
    POSITION_PREVIOUS = POSITION_CURRENT;                  //Set previous position
    prevTimePosition = currentTime;                        //Set prebious time
  }
}

/*
Calculates angualr velocity
*/
void calculate_Theta_Dot() {
  unsigned long currentTime = millis();                   //Current time
  float deltaT = (currentTime - prevTimeAngle) / 1000;    //Convert to seconds
  if (deltaT > 0) {                                       //Ensure no dividing by 0
    ANGLE_CURRENT = theta;                                //Current angle
    thetaDot = (ANGLE_CURRENT - ANGLE_PREVIOUS) / deltaT; //Calculate angular velocity
    ANGLE_PREVIOUS = ANGLE_CURRENT;                       //Set previous angle
    prevTimeAngle = currentTime;                          //Set prebious time
  }
}


/*
Function moves the cart, the speed of the cart is dependannt on 'controlValue'
- Motor value < 0 will move the cart to the left
- Motor value > 0 will move the cart to the right
- AnalogWrite sets the Pulse Width Modulation (PWM) duty cycle (modulated from 0 - 255)
- The PWM determines how much power is supplied to the motor, it makes the motor move while the digitalWrites just determine direction
*/
void moveMotor(int controlValue) {
  //Checks to see if motor value exceeds hardware limit
  if (controlValue > 255) {            //If control value is too high
    controlValue = 255;                //Limits PWM
  } else if (controlValue < -255) {    //Else if control value is too low
    controlValue = -255;               //Limits PWM
  }

  if (controlValue < 0){                                     //If control value is greater than 0
    digitalWrite(MOTOR_PIN_ONE, HIGH), digitalWrite(MOTOR_PIN_TWO, LOW);  //Move cart
  }
  else{
    digitalWrite(MOTOR_PIN_ONE, LOW), digitalWrite(MOTOR_PIN_TWO, HIGH);  //Move cart
  }
  analogWrite(MOTOR_PWM, abs(controlValue));                       //Assign control gain to PWM register
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
    digitalWrite(MOTOR_PWM, LOW);                                                            //Turn off motor
  } else {
    motorState = false; //Turn on flag
  }
  return motorState;    //Returns current motor flag
}

/*
This function returns an average value of potentiometer readings for increase accuracy
- u8 ANALOG_PIN: always 5 because A5 is sensor
- u8 numberOfSamples: how many readings are included in average
*/
u16 sensorAverage(u8 ANALOG_PIN, u8 numberOfSamples) {
  unsigned int sensorReadingSum = 0;                      //Sum of potentiometer readings
  for (unsigned char t = 0; t < numberOfSamples; t++) {
    sensorReadingSum += analogRead(ANALOG_PIN);           //Add up readings
  }
  return sensorReadingSum / numberOfSamples;              //Return average
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
  if (digitalRead(ENCODER_PIN_A) == HIGH) {   //If triggered by rising edge A
    if (digitalRead(ENCODER_PIN_B) == LOW) {
      currentPosition++;                  //Increment position
    } else {
      currentPosition--;                  //Decrement position
    }
  } else {                                //If triggered by rising edge B
    if (digitalRead(ENCODER_PIN_B) == LOW) {
      currentPosition--;                  //Decrement position
    } else {
      currentPosition++;                  //Increment position
    }
  }
}