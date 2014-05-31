#pragma config(Sensor, S3,     Light,          sensorLightActive)
#pragma config(Motor,  motorA,          Right,         tmotorNormal, PIDControl, encoder)
#pragma config(Motor,  motorB,          Left,          tmotorNormal, PIDControl, encoder)

float k; //expressed by r/d
short threshold;

#define timeout 550
#define beepThreshold 5
#define beepInterval 1000
#define sampleFreq 100
#define cspeed 20 // former 20
#define rotateSpeed 25
#define targetCalib 4
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

void stop(){
    control(0, 0, 50);
}

int cellCount = 0;
int turnCount = 0;

bool probe(){
    int pre_compass = compass();
    control(cspeed, cspeed / 3, 50);
    while(SensorValue[Light] > threshold && pre_compass - compass() < 40){}
    stop();
    if(SensorValue[Light] < threshold){
      cellCount++;
      return false;
    }
    else return true;
}

void PIDDriver(short cruise_speed, short target,  float KP, float KD, float KI){
  float e, e_dot, old_e = threshold - SensorValue[Light], E = 0;
  short u;

  time1[T1] = 0; // T1 is to determine whether the robot has go off the line
  time1[T2] = 0; // T2 is for compass sampling
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
            // T3 is for interval between the beeps so that it will not beep intermediately
                PlayTone(1175, 50);
                cellCount++;
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
      turnCount++;
      if(turnCount > 2){
        control(cspeed / 2, cspeed * 2, 1300);
        stop();
        return;
      }
      control(-cspeed, cspeed, 300);
      while(SensorValue[Light] > threshold){}
      stop();

      time1[T1] = 0;
      time1[T3] = 0; // Don's miss these two lines!!!
      time1[T2] = 0;
    }

  }
}

task main(){
  if(!readCalib()) return; //reads calibration informations, return if it fails.

  // Move backwards and turn right until find the line
  control(-cspeed, -cspeed);
  while(SensorValue[Light] > threshold){}
  stop();
  control(cspeed, 2, 500); // Turn right
  while(SensorValue[Light] > threshold){}
  stop();

  // PID
  PIDDriver(cspeed, threshold + targetCalib, Kp, Kd, Ki);
  nxtDisplayString(3, "#Cells = %d", cellCount - 2);
  nxtDisplayString(4, "#Turns = %d", turnCount);
  wait1Msec(10000);
  return;
}
