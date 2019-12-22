#include <QTRSensors.h>

#define Kp 0 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 0 // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define rightMaxSpeed 200 // max speed of the robot
#define leftMaxSpeed 200 // max speed of the robot
#define rightBaseSpeed 150 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 150  // this is the speed at which the motors should spin when the robot is perfectly on the line
#define NUM_SENSORS  8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 us for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 2

#define rightMotor1 3
#define rightMotor2 4
#define rightMotorPWM 5
#define leftMotor1 12
#define leftMotor2 13
#define leftMotorPWM 11
#define motorPower 8

bool dry_angle[]={'1','0','0','0','0','0','0','1','1','0','1','0','0','0','0','0','0','0','0','0','0','0','0','0','0','1','1','0','0','0','0','0','0','0','1','1','0','0','0','0','0','0','1','0','1','0','0','0','0','0'};
char dry_path[]= {'L','L','L','B','S','R','L','L','R','R','R','L','L','L','B','S','R','R','B','R','R','S','R','L','L','L','R','R','L','B','L','L','B','S','L','S','R','L','R','L','R','R','R','B','L','R','L','S','B','L'};

bool actual_angle[]={'1','0','0','0','0','0','0','0','0','0','0','0','0','0'};
char actual_path[]= {'L','L','R','R','L','R','L','R','R','L','R','S','L','R'};

bool LEEFT_IR;
bool RIGHT_IR;

QTRSensorsRC qtrrc((unsigned char[]) {
  A0, A1, A2, A3, A4, A5, A6, A7
} , NUM_SENSORS, TIMEOUT, EMITTER_PIN); // sensor connected through analog pins A0 - A7 i.e. digital pins 14-19 in uno

unsigned int sensorValues[NUM_SENSORS];

void setup()
{
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(motorPower, OUTPUT);

  int i;
  for (int i = 0; i < 100; i++) // calibrate for sometime by sliding the sensors across the line, or you may use auto-calibration instead

    /* comment this part out for automatic calibration
      if ( i  < 25 || i >= 75 ) // turn to the left and right to expose the sensors to the brightest and darkest readings that may be encountered
       turn_right();
      else
       turn_left(); */
    qtrrc.calibrate();
  delay(20);
  wait();
  delay(2000); // wait for 2s to position the bot before entering the main loop

  /* comment out for serial printing

    Serial.begin(9600);
    for (int i = 0; i < NUM_SENSORS; i++)
    {
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(' ');
    }
    Serial.println();

    for (int i = 0; i < NUM_SENSORS; i++)
    {
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(' ');
    }
    Serial.println();
    Serial.println();
  */
}

int lastError = 0;
int position;
int error;

void loop()
{
  unsigned int sensors[8];
  position = qtrrc.readLine(sensors,QTR_EMITTERS_ON,1); // get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
  error = position - 2500;

  dryrun();
  
}

void dryrun(){

    int motorSpeed = Kp * error + Kd * (error - lastError);
    lastError = error;


    if(position<4300 && poistion>700){                                  //OR LEFT AND RIGHT IR ARE 1
        int rightMotorSpeed = rightBaseSpeed + motorSpeed;
        int leftMotorSpeed = leftBaseSpeed - motorSpeed;

        if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
        if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
        if (rightMotorSpeed < 0) rightMotorSpeed = 0; // keep the motor speed positive
        if (leftMotorSpeed < 0) leftMotorSpeed = 0; // keep the motor speed positive

        {
            digitalWrite(motorPower, HIGH); // move forward with appropriate speeds
            digitalWrite(rightMotor1, HIGH);
            digitalWrite(rightMotor2, LOW);
            analogWrite(rightMotorPWM, rightMotorSpeed);
            digitalWrite(motorPower, HIGH);
            digitalWrite(leftMotor1, HIGH);
            digitalWrite(leftMotor2, LOW);
            analogWrite(leftMotorPWM, leftMotorSpeed);
        }
    }
    else
    {
        switch(dry_path[ctr]){
            case 'L':
                if(!dry_angle[ctr])
                    leftTurn();
                else
                    left135();
                break;
            case 'R':
                if(!dry_angle[ctr])
                    rightTurn();
                else
                    right135();
                break;
            case 'S':
                if(!dry_angle[ctr])
                    straight();
                else 
                    straight135();
                break;
            case 'B':
                back();
                break;
        }
        ctr++;
    }
}


void wait() {
  digitalWrite(motorPower, LOW);
}

void rightTurn(){
   

void leftTurn(){
    
}

void straight(){
   
}

void back(){
   
}

void right135(){
   

void left135(){
    
}

void straight135(){
   
}


void actualrun(){
    int motorSpeed = Kp * error + Kd * (error - lastError);
    lastError = error;
    int ctr=0;

    if(position<4300 && poistion>700){                                  //OR LEFT AND RIGHT IR ARE 1
        int rightMotorSpeed = rightBaseSpeed + motorSpeed;
        int leftMotorSpeed = leftBaseSpeed - motorSpeed;

        if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
        if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
        if (rightMotorSpeed < 0) rightMotorSpeed = 0; // keep the motor speed positive
        if (leftMotorSpeed < 0) leftMotorSpeed = 0; // keep the motor speed positive

        {
            digitalWrite(motorPower, HIGH); // move forward with appropriate speeds
            digitalWrite(rightMotor1, HIGH);
            digitalWrite(rightMotor2, LOW);
            analogWrite(rightMotorPWM, rightMotorSpeed);
            digitalWrite(motorPower, HIGH);
            digitalWrite(leftMotor1, HIGH);
            digitalWrite(leftMotor2, LOW);
            analogWrite(leftMotorPWM, leftMotorSpeed);
        }
    }
    else
    {
        switch(actual_path[ctr]){
            case 'L':
                if(!actual_angle[ctr])
                    leftTurn();
                else
                    left135();
                break;
            case 'R':
                if(!actual_angle[ctr])
                    rightTurn();
                else
                    right135();
                break;
            case 'S':
                if(!actual_angle[ctr])
                    straight();
                else 
                    straight135();
                break;
            case 'B':
                back();
                break;
        }
        ctr++;
    }
}