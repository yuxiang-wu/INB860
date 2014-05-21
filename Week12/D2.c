#pragma config(Sensor, S3,     Light,          sensorLightActive)
#pragma config(Motor,  motorA,          Right,         tmotorNormal, PIDControl, encoder)
#pragma config(Motor,  motorB,          Left,          tmotorNormal, PIDControl, encoder)

#define timeout 550
#define beepThreshold 6
#define beepInterval 1000
#define sampleFreq 100
#define cspeed 20
#define rotateSpeed 25
#define targetCalib 3
#define KP 1.1
#define KD 0.3
#define KI 0

// Left or right edge follower
#define LEFT_EDGE true
#define RIGHT_EDGE false
// Modes of PID
#define DETECT_ROWS 1
#define DETECT_COLUMNS 2
#define STRAIGHT_FORWARD 3
// Parameters for grid
#define MAX_ROWS 5
#define MAX_COLUMNS 5
#define MAX_CELL 25
#define FIRST_ROW_OFFSET 183
#define FIRST_COL_OFFSET 183
// A* algorithm
#define INFINITY 200000

// PID
float k; //expressed by r/d
short threshold;
// Grid
int numOfRows, numOfColumns;
float height[MAX_ROWS];
float width[MAX_COLUMNS];
bool greyPatch[MAX_ROWS][MAX_COLUMNS];
// A* algorithm
float weight[MAX_ROWS][MAX_COLUMNS];
float heuristic[MAX_ROWS][MAX_COLUMNS];
int explored[MAX_ROWS][MAX_COLUMNS];
int curRow, curCol;


#include "D2.h"
#include "priority_queue.h"

void PID(short target, bool isLeftEdgeFollower, int mode){
  float e, e_dot, old_e = target - SensorValue[Light], E = 0;
  short u;

  time1[T1] = 0; // T1 is to determine whether the robot has go off the line
  time1[T2] = 0; // T2 is for compass sampling
  float pre_compass = compass();

  if(mode == DETECT_ROWS || mode == DETECT_COLUMNS){
      nMotorEncoder[Left] = 0;
      nMotorEncoder[Right] = 0;
  }
  float pre_encoder;
  if(mode == DETECT_ROWS)
      pre_encoder = nMotorEncoder[Left];
  else if(mode == DETECT_COLUMNS)
      pre_encoder = nMotorEncoder[Right];

  while(true){
    if(time1[T1] < timeout){
        e = threshold - SensorValue[Light];
        e_dot = e - old_e;
        E += e;
        u = (short)(KP * e + KD * e_dot + KI * E);
        if(!isLeftEdgeFollower)
            u = -1 * u;
        old_e = e;
        motor[Left]=cspeed - u;
        motor[Right]=cspeed + u;
        if(SensorValue[Light] < target){ //target can be make more conservative than threshold
          time1[T1] = 0;
        }

        if(time1[T2] > sampleFreq && SensorValue[Light] < target ){ //the light sensor should be on the tape when it beeps
            float cur_compass = compass();
            if(time1[T3] > beepInterval && ((cur_compass - pre_compass) > beepThreshold || (pre_compass - cur_compass) > beepThreshold)){
            // T3 is for interval between the beeps so that it will not beep intermediately
                PlayTone(1175, 50);
                if(mode == DETECT_ROWS){
                    if(numOfRows == 0){
                        height[numOfRows] = FIRST_ROW_OFFSET + nMotorEncoder[Left] - pre_encoder;
                    }
                    else{
                        height[numOfRows] = nMotorEncoder[Left] - pre_encoder;
                    }
                    numOfRows++;
                    pre_encoder = nMotorEncoder[Left];
                }
                else if(mode == DETECT_COLUMNS){
                    if(numOfColumns == 0){
                        width[numOfColumns] = FIRST_COL_OFFSET + nMotorEncoder[Right] - pre_encoder;
                    }
                    else{
                        width[numOfColumns] = nMotorEncoder[Right] - pre_encoder;
                    }
                    numOfColumns++;
                    pre_encoder = nMotorEncoder[Right];
                }
                else if(mode == STRAIGHT_FORWARD){
                }
                time1[T1] = 0;
                time1[T3] = 0;
                cur_compass = compass();
            }

            pre_compass = cur_compass;
            time1[T2] = 0;
        }
    }
    else{
      //PlayTone(1175, 50);
      stop();
      return;

      time1[T1] = 0;
      time1[T3] = 0; // Don's miss these two lines!!!
      time1[T2] = 0;
    }

  }
}

task main(){
  if(!readCalib()) return; //reads calibration informations, return if it fails.
  init();

  // Move backwards and turn right until find the line
  control(-cspeed, -cspeed);
  while(SensorValue[Light] > threshold){}
  stop();
  control(cspeed, 2, 500); // Turn right
  while(SensorValue[Light] > threshold){}
  stop();

  int target = threshold + targetCalib;
  // Detect the columns and row of the grid
  PID(target, LEFT_EDGE, STRAIGHT_FORWARD);
  turn(150);

  PID(target, LEFT_EDGE, DETECT_ROWS);
  turn(-250);

  PID(target, LEFT_EDGE, STRAIGHT_FORWARD);
  turn(-150);

  PID(target, RIGHT_EDGE, DETECT_COLUMNS);
  turn(250);

  PID(target, RIGHT_EDGE, STRAIGHT_FORWARD);
  turn(150);

  // Start A* algorithm
  initAStar();

  return;
}
