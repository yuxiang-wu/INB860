#pragma config(Sensor, S3,     Light,          sensorLightActive)
#pragma config(Motor,  motorA,          Right,         tmotorNormal, PIDControl, encoder)
#pragma config(Motor,  motorB,          Left,          tmotorNormal, PIDControl, encoder)

float k; //expressed by r/d
short threshold;

#define timeout 550
#define swipeAngle 350
#define beepThreshold 10
#define sampleFreq 600
#define cspeed 20
#define rotateSpeed 25
#define targetCalib 4
#define beepInterval 1500
#define Kp 1.1
#define Kd 0.3
#define Ki 0

void control(int left, int right, int duration = 0){
    motor[Left] = left;
    motor[Right] = right;
    if(duration)
        wait1Msec(duration);
}

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

void findAndStop(int offset){
  control(cspeed, cspeed);
  while(SensorValue[Light] > threshold){}
  nMotorEncoder[Left]=0;
  nMotorEncoder[Right]=0;
  nMotorEncoderTarget[Left] = offset;
  nMotorEncoderTarget[Right] = offset;
  control(cspeed, cspeed);
  while(nMotorRunState[Left] != runStateIdle){}
  control(0, 0, 50);
}

void turn(int angle){
  int startCompass = compass();
  if(angle > 0){
    control(-rotateSpeed, rotateSpeed);
    while(compass() - startCompass < angle){}
  }
  else{
    control(rotateSpeed, -rotateSpeed);
    while(compass() - startCompass > angle){}
  }
  control(0, 0, 50);
}

// Turn on the intersaction to find how many branched it has
int getNumberOfBranches(){
  control(-cspeed, cspeed);

  int count = 0;
  int cur_compass = compass();

  bool wasDark;
  if(SensorValue[Light] < threshold)
    wasDark = true;
  else
    wasDark = false;

  while(compass() - cur_compass < 356){
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


task main(){
  if(!readCalib()) return; //reads calibration informations, return if it fails.


  // Go straight forward until the center hit the line
  findAndStop(200);

  // Turn left 90 degrees
  turn(90);

  // Move forward
  findAndStop(70);

  // Turn right 90 degrees
  turn(-88);

  while(1){
      findAndStop(130);
      int n = getNumberOfBranches();
      if(n == 4)
          continue;
      else if(n == 3)
          turn(88);
      else if(n == 2)
          break;
  }

  control(10, 15, 1000);
  return;
}
