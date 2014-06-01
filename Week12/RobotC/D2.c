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
#define MAX_ROWS 10
#define MAX_COLUMNS 10
#define MAX_CELL 25
#define FIRST_ROW_OFFSET 130
#define FIRST_COL_OFFSET 130

// PID
float k; //expressed by r/d
short threshold;
// Grid
int numOfRows, numOfColumns;
float height[MAX_ROWS];
float width[MAX_COLUMNS];

#include "D2.h"

// write the height of rows, width of columns into a text file
void writeLogFile(){
    const string filename ="grid.txt";
    TFileIOResult nIoResult;
    TFileHandle filehandle;
    Delete(filename, nIoResult);
    int filesize = 1000;
    OpenWrite(filehandle, nIoResult, filename, filesize);
    string str;

    for(int i = 0; i < numOfRows; i++){
        StringFormat(str, "%f ", height[i]);
        WriteString(filehandle, nIoResult, str);
    }
    WriteString(filehandle, nIoResult, "\n");

    for(int i = 0; i < numOfColumns; i++){
        StringFormat(str, "%f ", width[i]);
        WriteString(filehandle, nIoResult, str);
    }
    Close(filehandle, nIoResult);
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
  PID(target, LEFT_EDGE, STRAIGHT_FORWARD);
  turn(150);

  // Detect the rows of the grid
  PID(target, LEFT_EDGE, DETECT_ROWS);
  turn(-250);

  // Return to start
  PID(target, LEFT_EDGE, STRAIGHT_FORWARD);
  turn(-150);

  // Detect the columns of the grid
  PID(target, RIGHT_EDGE, DETECT_COLUMNS);
  turn(250);

  // Return to start
  PID(target, RIGHT_EDGE, STRAIGHT_FORWARD);
  turn(150);

  writeLogFile();

  return;
}
