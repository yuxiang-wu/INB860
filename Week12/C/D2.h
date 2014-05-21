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
	  front[i][j] = 0;
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

}
