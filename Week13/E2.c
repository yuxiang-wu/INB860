#pragma config(Sensor, S3,     Light,          sensorLightActive)
#pragma config(Motor,  motorA,          Right,         tmotorNormal, PIDControl, encoder)
#pragma config(Motor,  motorB,          Left,          tmotorNormal, PIDControl, encoder)

float k; //expressed by r/d
short threshold;
int adjustCount = 0;

// Log file
TFileIOResult logResultIO;
TFileHandle logHandle;

#define timeout 650
#define swipeAngle 350
#define beepThreshold 10
#define sampleFreq 660
#define cruise_speed 20
#define rotateSpeed 25
#define targetCalib 4
#define beepInterval 1500
#define KP 1.1
#define KD 0.3
#define KI 0
#define adjustAmount 6
#define CALIB_CENTER 600

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
    nxtDisplayStringAt(0,31,"Please run background.");
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
    nxtDisplayStringAt(0,31,"Please run compass.");
    PlayTone(1175,15);
    wait1Msec(2000);
    return false;
  }
  Close(filehandle,nIoResult);

  return true;
}

void logCompass(float value){
  string str = "";
  StringFormat(str, "%.2f ", value);
  WriteString(logHandle, logResultIO, str);
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

float compass(){
  return (nMotorEncoder[Right] - nMotorEncoder[Left])*k;
}

float signedCompass(){
  float comp = compass() + adjustAmount *adjustCount;
  while(comp > 180.0){
    comp -= 360.0;
  }
  while(comp < -180.0){
    comp += 360.0;
  }
  return comp;
}

/*turn on the spot if the timer expires, return false if it could not find another branch*/
bool turnOnSpot(short target, bool greyPatch){

  float cur_compass = compass();

  //start rotating
  stop();
  control(-rotateSpeed, rotateSpeed);

  while(compass() - cur_compass < 390){
    float diff = compass() - cur_compass;
    if(SensorValue[Light] < target && (diff < 180 || diff > 250)){
      time1[T1] = 0;
      PlayTone(1175,20);
      stop();
      adjustCount++;
      return true;
    }
    //nxtDisplayStringAt(0,31,"%f", signedCompass());
  }
  stop();
  if(!greyPatch){
      control(-rotateSpeed, rotateSpeed);
      while(SensorValue[Light] > target){}
      stop();
  }
  return false;
}

void PIDDriver(short target){
  float e, e_dot, old_e = threshold - SensorValue[Light], E = 0;
  short u;

  time1[T1] = 0;
  time1[T2] = 0;
  time1[T4] = 0;
  float pre_compass = compass();
  bool greyPatch = false;
  while(true){
    if(time1[T1] < timeout){ // T1 for timeout
        e = threshold - SensorValue[Light];
        e_dot = e - old_e;
        E += e;
        u = (short)(KP * e + KD * e_dot + KI * E);
        old_e = e;
        motor[Left] = cruise_speed - u;
        motor[Right] = cruise_speed + u;
        if(SensorValue[Light] < target){ //target can be make more conservative than threshold
          time1[T1] = 0;
        }

        // When grey patch detected
        if(SensorValue[Light] > 38 && SensorValue[Light] < 43){
            if(time1[T4] > 300){
                greyPatch = true;
                stop();
                PlayTone(1500, 40);
                wait1Msec(800);
                PlayTone(1500, 40);
                wait1Msec(800);
                PlayTone(1500, 40);
                wait1Msec(800);
                control(rotateSpeed, -rotateSpeed, 1000); // 800 is for getting rid of the grey patch
                while(SensorValue[Light] > threshold){}
                stop();
                time1[T1] = 0;
                time1[T2] = 0;
                time1[T3] = 0;
                nMotorEncoder[Left]=0;
                nMotorEncoder[Right]=0;
                pre_compass = compass();
                continue;
            }
        }
        else{
            time1[T4] = 0;
        }

        if(time1[T2] > sampleFreq && SensorValue[Light] < target){ //the light sensor should be on the tape when it beeps
            float cur_compass = compass();
            if(time1[T3] > beepInterval && ((cur_compass - pre_compass) > beepThreshold || (pre_compass - cur_compass) > beepThreshold)){
                PlayTone(1175, 20);
                time1[T3] = 0; // T3 for beepInterval
            }

            pre_compass = cur_compass;
            time1[T2] = 0; // T2 for sampling frequency
            if(greyPatch){
              float tmp = signedCompass();
              nxtDisplayStringAt(0,31,"%f", tmp);
              logCompass(tmp);
            }
        }
    }
    else{
        if(greyPatch){
          if(!turnOnSpot(target, greyPatch)) return;
        }
        else{
          turnOnSpot(target, greyPatch);
        }
    }
  }
}

// Move forward until get to the pattern and put light sensor onto it
void moveToPattern(){
  control(cruise_speed, cruise_speed);
  while(SensorValue[Light] > threshold){ }
  wait1Msec(CALIB_CENTER);
  stop();
  control(rotateSpeed, -rotateSpeed);
  while(SensorValue[Light] > threshold){ }
  stop();
}

task main(){
  if(!readCalib()) return; //reads calibration informations, return if it fails.

  // initialize the log file
  const string logName = "E1Log.txt";
  short logSize = 2000;
  Delete(logName, logResultIO);
  OpenWrite(logHandle, logResultIO,logName, logSize);

  moveToPattern();

  //use PID controller to follow the line
  PIDDriver(threshold + targetCalib);

  Close(logHandle, logResultIO);
  return;
}
