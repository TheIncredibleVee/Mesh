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

char path[25];
int pathLen=0;

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
    else if(LEFT_IR==0){                     //Case T
        leftTurn();
    }
    else if(RIGHT_IR==0 ){
        if(position>2000 && position<3000)  //straight and right
            straight();
        else
            rightTurn();
    }
    else
        back();
}

void pathStore(char x){
    path[pathLen++]= x;
    if(pathLen < 3 || path[pathLen-2] != 'B')
        return;

    int totalAngle = 0;
    int i;
    for(i=1;i=<3;i++)
    {
        switch(path[pathLen-i])
        {
        case 'R':
            totalAngle += 90;
            break;
        case 'L':
            totalAngle += 270;
            break;
        case 'B':
            totalAngle += 180;
            break;
        }   
    }
    totalAngle = totalAngle % 360;
    switch(totalAngle)
    {
        case 0:
        path[pathLen - 3] = 'S';
        break;
        case 90:
        path[pathLen - 3] = 'R';
        break;
        case 180:
        path[pathLen - 3] = 'B';
        break;
        case 270:
        path[pathLen - 3] = 'L';
        break;
    }
    pathLen -= 2;
}

void wait() {
  digitalWrite(motorPower, LOW);
}

void rightTurn(){
    pathStore('R');
}

void leftTurn(){
    pathStore('L');
}

void straight(){
    pathStore('S');
}

void back(){
    pathStore('B');
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
        switch(path[ctr++]){
            case 'L':
                leftTurn();
                break;
            case 'R':
                rightTurn();
                break;
            case 'S':
                straight();
                break;
            case 'B':
                back();
                break;
        }
    }
}