/* This function reads 'threshold' and 'k' from background calibration
 * file "libCalib.dat" and wheel calibration file "compassCalib.dat" */
bool readCalib(){
  //read the max and min from file "libCalib.dat"
  TFileIOResult nIoResult;
  short filehandle;
  const string filename1 = "libCalib.dat";
  short filesize = 1000;
  OpenRead(filehandle, nIoResult, filename1, filesize);
  short min, max;
  ReadShort(filehandle, nIoResult, min);
  ReadShort(filehandle, nIoResult, max);

  //if file not existed, beep!
  if(nIoResult){
    nxtDisplayStringAt(0,31,"Background Calib Missing");
    PlayTone(1175,15);
    wait1Msec(2000);
    return false;
  }
  Close(filehandle, nIoResult);
  threshold = (min + max) / 2; //threshold is a global variable

  //read k from file "compassCalib.dat"
  const string filename2 = "compassCalib.dat";
  OpenRead(filehandle, nIoResult, filename2, filesize);
  ReadFloat(filehandle, nIoResult, k); //k is a global variable

  //if file not existed, beep!
  if(nIoResult){
    nxtDisplayStringAt(0,31,"Compass Calib Missing");
    PlayTone(1175,15);
    wait1Msec(2000);
    return false;
  }
  Close(filehandle,nIoResult);

  return true;
}

float compass(){
  return (nMotorEncoder[Right] - nMotorEncoder[Left])*k;
}

void control(int left, int right, int duration = 0){
    motor[Left] = left;
    motor[Right] = right;
    if(duration)
        wait1Msec(duration);
}

void stop(){
    control(0, 0, 50);
}

// Turn an certain angle until it finds a dark line
void turn(int angle){
  int startCompass = compass();
  if(angle > 0){
    control(-rotateSpeed, rotateSpeed, 100);
    while(compass() - startCompass < angle && SensorValue[Light] > threshold){}
  }
  else{
    control(rotateSpeed, -rotateSpeed, 100);
    while(compass() - startCompass > angle && SensorValue[Light] > threshold){}
  }
  stop();
}

void init(){
  numOfColumns = 0;
  numOfRows = 0;
  for(int i = 0; i < numOfRows; i++){
    for(int j = 0; j < numOfColumns; j++){
      greyPatch[i][j] = false;
    }
  }
}

cell makeCell(int row, int col, float weight){
	cell tmp;
	tmp.row = row;
	tmp.col = col;
	tmp.weight = weight;
	return tmp;
}

void initAStar(){
  for(int i = 0; i < numOfRows; i++){
    for(int j = 0; j < numOfColumns; j++){
      weight[i][j] = INFINITY;
      explored[i][j] = 0;
    }
  }

  float sumRow = 0;
  for(int i = numOfRows - 1; i >= 0; i--){
    float sumCol = 0;
    for(int j = numOfColumns - 1; j >= 0; j--){
        heuristic[i][j] = sqrt(sumRow * sumRow + sumCol * sumCol);
        sumCol += width[j];
    }
    sumRow += height[i];
  }

  curRow = 0;
  curCol = 0;
  frontierCount = 0;
  
  updateFrontier(makeCell(0, 0, heuristic[0][0]));
}
