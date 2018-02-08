/****************************************************
HelloRobot.ino: Initial Robot test sketch

Bernardo Ronquillo // 27 December 2017
*****************************************************/
// include mCore Makeblock library
#include <MeMCore.h>  // mCore Makeblock library

/***** Motor objects declaration *****/ 
MeDCMotor motor1(M1); //Motor1 is Left Motor
MeDCMotor motor2(M2); //Motor2 is Right Motor
/*
 * motorX.run() maximum speed is 255 to -255, 0 is stop (rember that PWM sends 8 bits signal)
 * motorX.stop()
 */
/***** Sensor objects declaration *****/ 
MeUltrasonicSensor ultrasonic(PORT_3); // Ultrasonic sensor
/* 
 * ultrasonic.distanceCm());  // Distance value from 3cm - 400cm
 */
MeLineFollower lineFinder(PORT_2); // Black line finder sensor
/*
 * int sensorState = lineFinder.readSensors();
 * S1_IN_S2_IN
 * S1_IN_S2_OUT
 * S1_OUT_S2_IN
 * S1_OUT_S2_OUT
 */
MeLEDMatrix Matrix_1(PORT_4); // LED matrix

/***** On Board LED and sensors objects declaration *****/ 
MeRGBLed led(0, 30); // RGB LEDs
/* led.setColorAt(0/1, 255, 0, 0); // LED0 (RGBLED1) (RightSide), LED1 (RGBLED2) (LeftSide)
 *  // 0-255 for each of 3 positions: Red, Green, Blue
 *  led.show(); // Make the LED on
 */
MeIR ir; // Infrared sensor from IR remote
/*
 * ir.begin();    // initialize sensor
 * ir.value >> 16 & 0xff; // Shifts to the right by 16 bits and masks with FF = 255 = 11111111
 *  // IR_BUTTON_A, B, ..., F, SETTING, LEFT, RIGHT, UP, DOWN, 0, 1, ..., 9
 */

/***** Global Defines ****/ 
// Constants for the delays needed to rotate the robot
const int MIN_SPEED = 40; // first table entry is 40% speed
const int SPEED_TABLE_INTERVAL = 10; // each table entry is 10% faster speed
const int NBR_SPEEDS = 1 + (100 - MIN_SPEED)/ SPEED_TABLE_INTERVAL;
int speedTable[NBR_SPEEDS] = {40, 50, 60, 70, 80, 90, 100}; // speeds
int rotationTime[NBR_SPEEDS] = {5500, 3300, 2400, 2000, 1750, 1550, 1150}; // time

// defines to identify sensors
const int SENSE_IR_LEFT   = 0;
const int SENSE_IR_RIGHT  = 1;
const int SENSE_IR_CENTER = 2;

// defines for directions
const int DIR_LEFT   = 0;
const int DIR_RIGHT  = 1;
const int DIR_CENTER = 2;  

const char* locationString[] = {"Left", "Right", "Center"}; // Debug labels
// http://arduino.cc/en/Reference/String for more on character string arrays

// obstacles constants 
const int OBST_NONE       = 0;  // no obstacle detected
const int OBST_LEFT_EDGE  = 1;  // left edge detected 
const int OBST_RIGHT_EDGE = 2;  // right edge detected
const int OBST_FRONT_EDGE = 3;  // edge detect at both left and right sensors

/**** End of Global Defines ****************/

// Setup runs at startup and is used configure pins and init system variables
void setup()
{
  Serial.begin(9600); 

  Matrix_1.setBrightness(Brightness_8);
  led.setpin(LED_BUILTIN); // = led.setpin(LED_BUILTIN); and no need of defining LED_PIN constant
  
  led.setColorAt(0, 255, 0, 0); //Set LED0 (RGBLED1) (RightSide) to Red
  led.setColorAt(1, 0, 0, 255); //Set LED1 (RGBLED2) (LeftSide)  to Blue
  led.show();                  

  Serial.println("Waiting for a sensor to detect blocked reflection");
}

void loop()
{
   // call a function when reflection blocked on left side
   if(lookForObstacle(OBST_LEFT_EDGE) == true)   {   
     calibrateRotationRate(DIR_LEFT,360);  // calibrate CCW rotation
   }
   // as above for right sensor
   if(lookForObstacle(OBST_RIGHT_EDGE) == true)   {   
     calibrateRotationRate(DIR_RIGHT, 360);  // calibrate CW rotation
   }  
}

// function to indicate numbers by flashing the built-in LED
void blinkNumber( byte number) {
   while(number--) {
    // Conver number to char array in order to can show it in the LED matrix
    char data[1];
    itoa(number,data,10);
    Serial.println(data);
    Matrix_1.drawStr(5,7,data);

    //And blink the RGB LEDs
    led.setColorAt(0, 255,255,255); // WHITE to correspond wit HIGH
    led.setColorAt(1, 255,255,255);
    led.show(); delay(100);
    led.setColorAt(0, 255,0,0); // RED to correspond wit LOW
    led.setColorAt(1, 255,0,0);
    led.show(); delay(400);
   }
  led.setColorAt(0, 0,0,255); // Stay BLUE meanwhile
  led.setColorAt(1, 0,0,255);
  led.show();
}

/**********************
 code to look for obstacles
**********************/

// returns true if the given obstacle is detected
boolean lookForObstacle(int obstacle)
{
  switch(obstacle) {
     case  OBST_FRONT_EDGE: return irEdgeDetect(DIR_LEFT) || irEdgeDetect(DIR_RIGHT);
     case  OBST_LEFT_EDGE:  return irEdgeDetect(DIR_LEFT); 
     case  OBST_RIGHT_EDGE: return irEdgeDetect(DIR_RIGHT);      
  }
  return false; 
}

/*************************************
 functions to rotate the robot
*************************************/
  
// return the time in milliseconds to turn the given angle at the given speed
long rotationAngleToTime( int angle, int speed)
{
int fullRotationTime; // time to rotate 360 degrees at given speed

  if(speed < MIN_SPEED)
    return 0; // ignore speeds slower then the first table entry
  
  angle = abs(angle);
  
  if(speed >= 100)
    fullRotationTime = rotationTime[NBR_SPEEDS-1]; // the last entry is 100%
  else
  { 
    int index = (speed - MIN_SPEED) / SPEED_TABLE_INTERVAL; // index into speed
                                                            // and time tables
    int t0 =  rotationTime[index];
    int t1 = rotationTime[index+1];    // time of the next higher speed 
    fullRotationTime = map(speed,  
                           speedTable[index],  
                           speedTable[index+1], t0, t1);
    // Serial.print("index= ");  Serial.print(index); Serial.print(", t0 = ");  
    // Serial.print(t0);  Serial.print(", t1 = ");  Serial.print(t1); 
  }
  // Serial.print(" full rotation time = ");  Serial.println(fullRotationTime);
  long result = map(angle, 0,360, 0, fullRotationTime);
  return result; 
}

// rotate the robot from MIN_SPEED to 100% increasing by SPEED_TABLE_INTERVAL
void calibrateRotationRate(int sensor, int angle)
{  
  Serial.print(locationString[sensor]);
  Serial.println(" calibration" );
  for(int speed = MIN_SPEED; speed <= 100; speed += SPEED_TABLE_INTERVAL)
  {   

    delay(1000);
    blinkNumber(speed/10);   
 
    if( sensor == DIR_LEFT)
    {    // rotate left
      motor1.run(speed); // BACKWARD is +positive. Motor 1 is the LEFT one
      motor2.run(-speed); // FORWARD is -negative. Motor 1 is the RIGHT one
      /*
       * motorReverse(MOTOR_LEFT,  speed); 
       * motorForward(MOTOR_RIGHT, speed);  
       */
    }
    else if( sensor == DIR_RIGHT)
    {    // rotate right
      motor1.run(-speed); // LEFT motor goes FORWARD
      motor2.run(speed); // RIGHT motor goes BACKWARD
      /*
       * motorForward(MOTOR_LEFT,  speed);
       * motorReverse(MOTOR_RIGHT, speed);
       */
      
      
    }
    else
       Serial.println("Invalid sensor");     
    
    int time = rotationAngleToTime(angle, speed);

    Serial.print(locationString[sensor]); Serial.print(": rotate ");
    Serial.print(angle); Serial.print(" degrees at speed "); Serial.print(speed);
    Serial.print(" for "); Serial.print(time); Serial.println("ms");
    
    delay(time); 
    motor1.stop();
    motor2.stop(); 
    delay(2000); // two second delay between speeds
  }    
}

/****************************
   ir reflectance sensor code   
****************************/

const byte NBR_SENSORS = 3;  // this version only has left and right sensors
const byte IR_SENSOR[NBR_SENSORS] = {0, 1, 2}; // analog pins for sensors

int irSensorAmbient[NBR_SENSORS]; // sensor value with no reflection
int irSensorReflect[NBR_SENSORS]; // value considered detecting an object
int irSensorEdge[NBR_SENSORS];    // value considered detecting an edge
boolean isDetected[NBR_SENSORS] = {false,false}; // set true if object detected

/* SOLO PARA EL CASO EN QUE TUVIESEMOS VALORES ANALÓGICOS
 *  
const int irReflectThreshold = 10; // % level below ambient to trigger reflection
const int irEdgeThreshold    = 90; // % level above ambient to trigger edge

void irSensorBegin()
{
  for(int sensor = 0; sensor < NBR_SENSORS; sensor++)
     irSensorCalibrate(sensor);
}

// calibrate thresholds for ambient light 
void irSensorCalibrate(byte sensor)
{
  int ambient = lineFinder.readSensors(); // get ambient level
  irSensorAmbient[sensor] = ambient; 
  // precalculate the levels for object and edge detection  
  irSensorReflect[sensor] = (ambient * (long)(100-irReflectThreshold)) / 100;
  irSensorEdge[sensor]    = (ambient * (long)(100+irEdgeThreshold)) / 100; 
}
*/

// returns true if an object reflection detected on the given sensor
// the sensor parameter is the index into the sensor array
boolean irSensorDetect(int sensor)
{
  boolean result = false; // default value
  int value = lineFinder.readSensors(); // get IR light level
  if( value <= irSensorReflect[sensor]) {
    result = true; // object detected (lower value means more reflection)
    if( isDetected[sensor] == false) { // only print on initial detection
      Serial.print(locationString[sensor]);         
      Serial.println(" object detected");
    }
  }
  isDetected[sensor] = result;  
  return result;
}

boolean irEdgeDetect(int sensor)
{
  boolean result = false; // default value
  int value = lineFinder.readSensors(); // get IR light level
  if( value >= irSensorEdge[sensor]) {
    result = true; // edge detected (higher value means less reflection)
    if( isDetected[sensor] == false) { // only print on initial detection
      Serial.print(locationString[sensor]);         
      Serial.println(" edge detected");
     }
  }
  isDetected[sensor] = result; 
  return result;
}
