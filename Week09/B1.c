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
#define beepInterval 1200
#define Kp 1.2
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
    nxtDisplayStringAt(0,31,"Please run background calibration program.");
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
    nxtDisplayStringAt(0,31,"Please run compass calibration program.");
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

void turnAndDisplay(){
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
  motor[Left] = 0;
  motor[Right] = 0;
  nxtDisplayTextLine(3, "#branches = %d", count);
}



/*turn on the spot if the timer expires, return false if it could not find another branch*/
bool turnOnSpot(short target){
  short wheelEncoderTurnDot = (short)(swipeAngle / k)/2;
  nMotorEncoderTarget[Left] = nMotorEncoder[Left] - wheelEncoderTurnDot;
  nMotorEncoderTarget[Right] = nMotorEncoder[Right] + wheelEncoderTurnDot;
  float cur_compass = compass();

  //start rotating
  motor[Left] = 0;
  motor[Right] = 0;
  wait1Msec(15);
  motor[Left] = -rotateSpeed;
  motor[Right] = rotateSpeed;

//  while(nMotorRunState[Right] != runStateIdle){
  while(compass() - cur_compass < 360){
    float diff = compass() - cur_compass;
    if(SensorValue[Light] < target && (diff < 180 || diff > 250)){
      time1[T1] = 0;
      PlayTone(1175,20);
      turnAndDisplay();
      return true;
    }
  }
  motor[Left]=0;
  motor[Right]=0;
  return false;
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
                PlayTone(1175, 20);
                turnAndDisplay();
                break;
                //time1[T3] = 0;
            }

            pre_compass = cur_compass;
            time1[T2] = 0;
        }
    }
    else{
      turnAndDisplay(); return;
    }

  }
}

task main(){
  if(!readCalib()) return; //reads calibration informations, return if it fails.

  nMotorEncoder[Left]=0;
  nMotorEncoder[Right]=0;

  //use PID controller to follow the line
  PIDDriver(cspeed, threshold + targetCalib, Kp, Kd, Ki);

  wait1Msec(10000);
  return;
}
