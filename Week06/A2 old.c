#pragma config(Sensor, S3,     Light,          sensorLightActive)
#pragma config(Motor,  motorA,          Right,         tmotorNormal, PIDControl, encoder)
#pragma config(Motor,  motorB,          Left,          tmotorNormal, PIDControl, encoder)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

const string filename1 ="libCalib.dat";
const string filename2 ="compassCalib.dat";
TFileIOResult nIoResult;
int fileSize=1000;
TFileHandle filehandle;

float k; //expressed by r/d

task logger(){
  const string logFile = "compassLog.txt";
  TFileIOResult yaIoResult;
  int yaFileSize=1000000;
  TFileHandle yaFileHandle;

  Delete(logFile, yaIoResult);
  OpenWrite(yaFileHandle, yaIoResult, logFile, yaFileSize);
  string str;
  while(true){
    float compass = (float)(nMotorEncoder[Right] - nMotorEncoder[Left])/k;
    StringFormat(str, "%f\n", compass);
    WriteString(yaFileHandle, yaIoResult, str);
    nxtScrollText(str);
    wait1Msec(700);
  }
  Close(yaFileHandle, yaIoResult);
  return;
}

const float KP=1.2, KD=0.3, KI=0;
float e, e_dot, old_e, E;
short u;

task main(){
  //read the max and min from file "libCalib.dat"
  OpenRead(filehandle, nIoResult, filename1, fileSize);
  short min, max;
  ReadShort(filehandle, nIoResult, min);
  ReadShort(filehandle, nIoResult, max);
  //if file not existed, warn!
  if(nIoResult){
    nxtDisplayStringAt(0,31,"Please run background calibration program.");
    PlayTone(1175,15);
    wait1Msec(2000);
    return;
  }
  Close(filehandle, nIoResult);
  short threshold = (min + max) / 2;


  //read k from file "compassCalib.dat"
  OpenRead(filehandle, nIoResult, filename2, fileSize);
  ReadFloat(filehandle, nIoResult, k);
  //if file not existed, warn!
  if(nIoResult){
    nxtDisplayStringAt(0,31,"Please run compass calibration program.");
    PlayTone(1175,15);
    wait1Msec(2000);
    return;
  }
  Close(filehandle,nIoResult);

  nMotorEncoder[Left]=0;
  nMotorEncoder[Right]=0;

  StartTask(logger);

  E=0;
  old_e = threshold - SensorValue[Light];

  time1[T1]=0;
  short cruise_speed = 15;
  time1[T1] = 0;
  while(true){
    if(SensorValue[Light] < threshold+5) //make more conservative threshold
      time1[T1] = 0;
    else{
      if(time1[T1] > 500 && time1[T1] < 900){ //stop time threshold
        motor[Left]=-30;
        motor[Right]=30;
        //PlayTone(975,100);
        nxtDisplayBigStringAt(0,31,"%d", time1[T1]);
        continue;
      }
      else if(time1[T1]>=900 && time1[T1] < 1200){
        wait1Msec(360);
        continue;
      }
      else if(time1[T1] >= 2000){
        StopTask(logger);
        motor[Left]=0;
        motor[Right]=0;
        break;
      }
    }

    e = threshold - SensorValue[Light];
    e_dot = e - old_e;
    //wait1Msec(400);
    E += e;
    u = (short)(KP * e + KD * e_dot + KI * E);
    old_e = e;
    motor[Left]=cruise_speed + u;
    motor[Right]=cruise_speed -u;
  }

  wait1Msec(10000);
  return;
}
