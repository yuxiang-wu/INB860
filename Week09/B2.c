#pragma config(Sensor, S3,     Light,          sensorLightActive)
#pragma config(Motor,  motorA,          Right,         tmotorNormal, PIDControl, encoder)
#pragma config(Motor,  motorB,          Left,          tmotorNormal, PIDControl, encoder)

float k; //expressed by r/d
short threshold;


#define timeout 550
#define swipeAngle 350
#define beepThreshold 12
#define sampleFreq 650
#define cspeed 20
#define rotateSpeed 25
#define targetCalib 4
#define beepInterval 1300
#define Kp 1.1
#define Kd 0.3
#define Ki 0

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

// Turn on the intersaction to find how many branched it has
int getNumberOfBranches(){
  motor[Left] = 0;
  motor[Right] = 0;
  wait1Msec(100);

  motor[Left] = -rotateSpeed;
  motor[Right ] = rotateSpeed;

  int count = 0;
  int cur_compass = compass();

  bool wasDark;
  if(SensorValue[Light] < threshold)
    wasDark = true;
  else
    wasDark = false;

  while(compass() - cur_compass < 360){
    if(SensorValue[Light] < threshold){
      if(!wasDark){
        wasDark = true;
        count++;
      }
    }
    else{
      wasDark = false;
    }
  }
  nxtDisplayTextLine(3, "#branches = %d", count);
  return count;
}

int logbook[100][2];
int size = 0;
bool wasUTurn = false;
bool finished = false;
int nodeCounter = 1;  // the root node

int driver(int n){
  if(finished)
    return -1;

  int c = 1 % n;

  if(wasUTurn){
    if(c == 0)
      wasUTurn = false;
    else{
      int tmp = (c + logbook[size - 1][0]) % n;
      if(tmp != 0){
        logbook[size - 1][0] = tmp;
        wasUTurn = false;
      }
      else{
        logbook[size - 1][0] = 0;
        logbook[size - 1][1] = 0;
        size--;
      }
    }
  }
  else{
    if(c == 0)
      wasUTurn = true;
    else{
      logbook[size][0] = c;
      logbook[size][1] = n;
      size++;
    }
    nodeCounter++;
  }

  // eraseDisplay();
  // for(int i = 0; i < size; i++){
    // nxtDisplayTextLine(i + 2, "%d  %d", logbook[i][0], logbook[i][1]);
    // wait1Msec(1000);
  // }
  // nxtDisplayTextLine(size + 3, "%d", size);

  if(size == 0)
    finished = true;

  return 1;
}

bool chooseBranch(int n){
  motor[Left] = 0;
  motor[Right] = 0;

  int count = 0;
  int cur_compass = compass();
  int c = driver(n); // can be modified by driver
  if(c == -1)
    return false;

  motor[Left] = -rotateSpeed;
  motor[Right ] = rotateSpeed;

  bool wasDark;
  if(SensorValue[Light] < threshold)
    wasDark = true;
  else
    wasDark = false;

  while(compass() - cur_compass < 360){
    if(SensorValue[Light] < threshold){
      if(!wasDark){
        wasDark = true;
        count++;
      }
    }
    else{
      wasDark = false;
    }
    if(count == c){
      motor[Left] = 0;
      motor[Right] = 0;
      wait1Msec(100);
      break;
    }
  }
  return true;
}

void PIDDriver(short cruise_speed, short target,  float KP, float KD, float KI){
  float e, e_dot, old_e = threshold - SensorValue[Light], E = 0;
  short u;

  time1[T1] = 0;
  time1[T2] = 0;
  float pre_compass = compass();
  while(true){
    if(time1[T1] < timeout){
        e = threshold - SensorValue[Light];
        e_dot = e - old_e;
        E += e;
        u = (short)(KP * e + KD * e_dot + KI * E);
        old_e = e;
        motor[Left]=cruise_speed - u;
        motor[Right]=cruise_speed + u;
        if(SensorValue[Light] < target){ //target can be make more conservative than threshold
          time1[T1] = 0;
        }
        if(time1[T2] > sampleFreq && SensorValue[Light] < target ){ //the light sensor should be on the tape when it beeps
            float cur_compass = compass();
            if(time1[T3] > beepInterval && ((cur_compass - pre_compass) > beepThreshold || (pre_compass - cur_compass) > beepThreshold)){
                PlayTone(1175, 50);
                int numberOfBranches = getNumberOfBranches();
                short tmp = compass();
                while(compass() - tmp < 210){};
                //motor[Left] = 0;
                //motor[Right] = 0;
                //wait1Msec(0);
                if(!chooseBranch(numberOfBranches)) return;
                time1[T1] = 0;
                time1[T3] = 0;
                cur_compass = compass();
            }

            pre_compass = cur_compass;
            time1[T2] = 0;
        }
    }
    else{
      int numberOfBranches = getNumberOfBranches();
      short tmp = compass();
      while(compass() - tmp < 240){};
      //motor[Left] = 0;
      //motor[Right] = 0;
      //wait1Msec(100);
      if(!chooseBranch(numberOfBranches)) return;
      time1[T1] = 0;
      time1[T3] = 0; // Don's miss these two lines!!!
      time1[T2] = 0;
      //break;
    }

  }
}

task main(){
  if(!readCalib()) return; //reads calibration informations, return if it fails.

  nMotorEncoder[Left]=0;
  nMotorEncoder[Right]=0;

  //use PID controller to follow the line
  PIDDriver(cspeed, threshold + targetCalib, Kp, Kd, Ki);

  eraseDisplay();
  nxtDisplayTextLine(3, "#nodes = %d", nodeCounter);
  wait1Msec(10000);
  return;
}
