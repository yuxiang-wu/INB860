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
//used in navigating to a cell
#define EAST 1
#define SOUTH 2
#define WEST 3
#define NORTH 4
#define TO_EAST 5
#define TO_SOUTH 6
#define TO_WEST 7
#define TO_NORTH 8

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
int front[MAX_ROWS][MAX_COLUMNS];
int curRow, curCol;
cell frontier[MAX_CELL];
int frontierCount;
int succ[4][2] = {{0, -1}, {-1, 0}, {0, 1}, {1, 0}};
//used in navigating to a cell
int edgePos, dir; // edge position can be EAST, SOUTH, WEST, NORTH. direction can be TO_EAST, TO_SOUTH, TO_WEST, TO_NORTH


#include "D2.h"
#include "priority_queue.h"

void go2Cell(int row, int col){
	
}

void AStar(){
	updateFrontier(makeCell(0, 0, heuristic[0][0]));
	while(frontierCount){
		cell s = popLeastWeight();
		if(s.row == numOfRows - 1 && s.col == numOfColumns - 1){
		// return solution
		}
		else{
			explored[s.row][s.col] = 1;
			for(int i = 0; i < 4; i++){
				int row = s.row + succ[i][0];
				int col = s.col + succ[i][1];
				if(explored[row][col])
					continue;
				else{
					// explore the cell (row, col) and update the frontier with it
				}
			}
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
  AStar();

  return;
}
