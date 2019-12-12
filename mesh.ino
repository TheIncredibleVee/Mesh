#include <QTRSensors.h>

#include <Encoder.h>

#define Kp 0 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 0 // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define rightMaxSpeed 200 // max speed of the robot
#define leftMaxSpeed 200 // max speed of the robot
#define rightBaseSpeed 150 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 150  // this is the speed at which the motors should spin when the robot is perfectly on the line
#define NUM_SENSORS  8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 us for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 2

#define IR_TOP_PIN 
#define IR_LEFT_PIN 
#define IR_RIGHT_PIN 
#define rightMotor1 3
#define rightMotor2 4
#define rightMotorPWM 5
#define leftMotor1 12
#define leftMotor2 13
#define leftMotorPWM 11
#define motorPower 8

struct maze{
    char point;
    bool type;                                      //0 -> no decision node 1-> decision node
    char dir[4];
    bool dir_visit[4];
    byte x_cordinate;
    byte y_cordinate;
}points[15];

points[0].point='A';
points[0].type=0;                                       
points[0].x_cordinate=0;
points[0].y_cordinate=0;

char previous_point='A';

byte adj[len][len];

for(byte i=0;i<len;i++){
    for(byte j=0;j<len;j++){
        if (i==j)
            adj[i][i]=0;
        else
            adj[i][j]=32767;
    }
}

byte len=1;

Encoder myEnc(5, 6);

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

  byte i;
  for (i = 0; i < 100; i++)
    qtrrc.calibrate();
  delay(20);
  wait();
  delay(2000); 
}

byte point_pos=0;
byte previous_x=0;
byte previous_y=0;
byte temp_x=0;
byte temp_y=0;
bool angle;             //1 means 90 and 0 means 135

char curr_dir='N';
char last_turn='S';
char prev_dir='S';
int encoder_start=myEnc.read();
int encoder_stop;
int lastError = 0;

void loop(){
    unsigned int sensors[8];
    int position = qtrrc.readLine(sensors,QTR_EMITTERS_ON,1); // get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
    int error = position - 3500;
    bool IR_TOP = digiatlread(IR_TOP_PIN);
    bool IR_LEFT = digiatlread(IR_LEFT_PIN);
    bool IR_RIGHT = digiatlread(IR_RIGHT_PIN);
    if(position<6000 && position>1000){
        int motorSpeed = Kp * error + Kd * (error - lastError);
        lastError = error;
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
    else if(IR_LEFT==1 && IR_RIGHT==1 && IR_TOP==1) {             //90 deg cross
        encoder_stop=myEnc.read();
        assign_temp_point();
        point_pos=check();
        if(point_pos==-1){
            points[len].x_cordinate=temp_x;
            points[len].y_cordinate=temp_y;
            points[len].type=1;
            points[len].dir[0]=curr_dir;
            if(curr_dir== 'N'){
                points[len].dir[1]='E';
                points[len].dir[2]='S';
                points[len].dir[3]='W';
            }
            if (curr_dir=='S'){
                points[len].dir[1]='W';
                points[len].dir[2]='N';
                points[len].dir[3]='E';
            }
            if (curr_dir=='E'){
                points[len].dir[1]='S';
                points[len].dir[2]='W';
                points[len].dir[3]='N';
            }
            if(curr_dir=='W'){
                points[len].dir[1]='N';
                points[len].dir[2]='E';
                points[len].dir[3]='S';
            }
            points[len].dir_visit[1]=0;
            points[len].dir_visit[0]=1;
            points[len].dir_visit[2]=1;
            points[len].dir_visit[3]=1;
            points[len].point='A'+len;
            adj['A'-previous_point]['A'-points[len].point]=encoder_stop-encoder_start;
            adj['A'-points[len].point]['A'-previous_point]=encoder_stop-encoder_start;
            previous_point=points[len].point;
            len++;
            leftTurn();
        }
        else if(points[point_pos].dir_visit[2]==1 && points[point_pos].dir[2]!=curr_dir){
            
            decision(points[point_pos].dir[2]);
            points[point_pos].dir_visit[2]=0;
        }
        else if(points[point_pos].dir_visit[3]==1 && points[point_pos].dir[3]!=curr_dir){
            decision(points[point_pos].dir[3]);
            points[point_pos].dir_visit[3]=0;
        }
        else if(points[point_pos].dir_visit[0]==1 && points[point_pos].dir[0]!=curr_dir){
            decision(points[point_pos].dir[0]);
            points[point_pos].dir_visit[0]=0;
        }
        previous_x=temp_x;
        previous_y=temp_y;
        encoder_start=myEnc.read();
    }
    else if(IR_LEFT==1 && IR_RIGHT==1 && IR_TOP==0){           //90 deg T
        encoder_stop=myEnc.read();
        assign_temp_point();
        if(check()){
            points[len].x_cordinate=temp_x;
            points[len].y_cordinate=temp_y;
            points[len].type=1;
            points[len].point='A'+len;
            adj['A'-previous_point]['A'-points[len].point]=encoder_stop-encoder_start;
            adj['A'-points[len].point]['A'-previous_point]=encoder_stop-encoder_start;
            previous_point=points[len].point;
            len++;
        }
        leftTurn();
        dir='L';
        previous_x=temp_x;
        previous_y=temp_y;
        encoder_start=myEnc.read();
    }
    else if(IR_LEFT==1 && IR_RIGHT==0 && IR_TOP==1){           //left and straight
        encoder_stop=myEnc.read();
        assign_temp_point();
        if(check()){
            points[len].x_cordinate=temp_x;
            points[len].y_cordinate=temp_y;
            points[len].type=1;
            points[len].point='A'+len;

            adj['A'-previous_point]['A'-points[len].point]=encoder_stop-encoder_start;
            adj['A'-points[len].point]['A'-previous_point]=encoder_stop-encoder_start;
            previous_point=points[len].point;
            len++;
        }
        leftTurn();
        dir='L';
        previous_x=temp_x;
        previous_y=temp_y;
        encoder_start=myEnc.read();
    }
    else if(IR_LEFT==1 && IR_RIGHT==0 && IR_TOP==0){           //left only
        encoder_stop=myEnc.read();
        assign_temp_point();
        if(check()){
            points[len].x_cordinate=temp_x;
            points[len].y_cordinate=temp_y;
            points[len].type=0;
            points[len].point='A'+len;

            adj['A'-previous_point]['A'-points[len].point]=encoder_stop-encoder_start;
            adj['A'-points[len].point]['A'-previous_point]=encoder_stop-encoder_start;
            previous_point=points[len].point;
            len++;
        }
        leftTurn();
        dir='L';
        previous_x=temp_x;
        previous_y=temp_y;
        encoder_start=myEnc.read();
    }
    else if(IR_LEFT==0 && IR_RIGHT==1 && IR_TOP==1){           //right and straight
        encoder_stop=myEnc.read();
        assign_temp_point();
        if(check()){
            points[len].x_cordinate=temp_x;
            points[len].y_cordinate=temp_y;
            points[len].type=1;
            points[len].point='A'+len;

            adj['A'-previous_point]['A'-points[len].point]=encoder_stop-encoder_start;
            adj['A'-points[len].point]['A'-previous_point]=encoder_stop-encoder_start;
            previous_point=points[len].point;
            len++;
        }
        straight();
        dir='S';
        previous_x=temp_x;
        previous_y=temp_y;
        encoder_start=myEnc.read();
    }
    else if(IR_LEFT==1 && IR_RIGHT==0 && IR_TOP==0){           //right only
        encoder_stop=myEnc.read();
        assign_temp_point();
        if(check()){
            points[len].x_cordinate=temp_x;
            points[len].y_cordinate=temp_y;
            points[len].type=1;
            points[len].point='A'+len;

            adj['A'-previous_point]['A'-points[len].point]=encoder_stop-encoder_start;
            adj['A'-points[len].point]['A'-previous_point]=encoder_stop-encoder_start;
            previous_point=points[len].point;
            len++;
        }
        previous_x=temp_x;
        previous_y=temp_y;
        rightTurn();
        dir='R';
        encoder_start=myEnc.read();
    }
    else if(IR_LEFT==0 && IR_RIGHT==0 && IR_TOP==0){           //180
        encoder_stop=myEnc.read();
        assign_temp_point();
        if(check()){
            points[len].x_cordinate=temp_x;
            points[len].y_cordinate=temp_y;
            points[len].type=0;
            points[len].point='A'+len;

            adj['A'-previous_point]['A'-points[len].point]=encoder_stop-encoder_start;
            adj['A'-points[len].point]['A'-previous_point]=encoder_stop-encoder_start;
            previous_point=points[len].point;
            len++;
        }   
        Back();
        dir='B';
        previous_x=temp_x;
        previous_y=temp_y;
        encoder_start=myEnc.read();
    }    
}

void assign_temp_point(){
    switch(dir){
        case 'S':
            temp_x= previous_x;
            temp_y= previous_y + (encoder_stop-encoder_start);
            break;
        case 'L':
            if(angle){
                temp_x= previous_x - (encoder_stop-encoder_start);
                temp_y= previous_y;
            }
            else{
                temp_x= previous_x - 0.707*(encoder_stop-encoder_start);
                temp_y= previous_y + 0.707*(encoder_stop-encoder_start);
            }
            break;
        case 'R':
            if(angle){
                temp_x= previous_x + (encoder_stop-encoder_start);
                temp_y= previous_y;
            }
            else{
                temp_x= previous_x + 0.707*(encoder_stop-encoder_start);
                temp_y= previous_y + 0.707*(encoder_stop-encoder_start);
            }
            break;
        case 'B':
            temp_x= previous_x;
            temp_y= previous_y - (encoder_stop-encoder_start);
            break;

    }
}

bool check(){
    for(byte i=0;i<len;++i){
        if(points[i].x_cordinate==temp_x && points[i].y_cordinate==temp_y){
            adj['A'-previous_point]['A'-points[i].point]=encoder_stop-encoder_start;
            adj['A'-points[i].point]['A'-previous_point]=encoder_stop-encoder_start;
            previous_point=points[i].point;
            return i;
        }
    }
    return -1;
}

void wait() {
  digitalWrite(motorPower, LOW);
}

void leftTurn(){
    switch(last_turn){
        case 'R':
            curr_dir='S';
            break;
        case 'L':
            curr_dir='N';
            break;
        case 'S':
            curr_dir='E';
            break;
        case 'B':
            curr_dir='W';
            break;
        }
    last_turn='L';
}

void rightTurn(){
    switch(last_turn){
        case 'R':
            curr_dir='N';
            break;
        case 'L':
            curr_dir='S';
            break;
        case 'S':
            curr_dir='W';
            break;
        case 'B':
            curr_dir='E';
            break;
    }

    last_turn='R';
}

void straight(){
}

void back(){
    switch(curr_dir){
        case 'N':
            curr_dir='S';
            break;
        case 'S':
            curr_dir='N';
            break;
        case 'E':
            curr_dir='W';
            break;
        case 'W':
            curr_dir='E';
            break;
    }
    switch (last_turn)
    {
    case 'R':
        last_turn='L';
        break;
    case 'L':
        last_turn='R';
        break;
    }
}

void decision(char x){
    switch(x){
        case 'S':
            switch (curr_dir)
            {
                case 'N':
                    straight();
                    break;
                case 'E':
                    rightTurn();
                    break;
                case 'W':
                    leftTurn();
                    break;
            }
            break;
        case 'N':
            switch (curr_dir)
            {
                case 'S':
                    straight();
                    break;
                case 'W':
                    rightTurn();
                    break;
                case 'E':
                    leftTurn();
                    break;
            }
            break;
        case 'W':
            switch (curr_dir)
            {
                case 'E':
                    straight();
                    break;
                case 'N':
                    rightTurn();
                    break;
                case 'S':
                    leftTurn();
                    break;
            }
            break;
        case 'E':
            switch (curr_dir)
            {
                case 'W':
                    straight();
                    break;
                case 'S':
                    rightTurn();
                    break;
                case 'N':
                    leftTurn();
                    break;
            }
            break;
    }
}
