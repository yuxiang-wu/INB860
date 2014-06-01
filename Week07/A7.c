#pragma config(Sensor, S3,     Light,          sensorLightActive)
#pragma config(Motor,  motorA,          Right,         tmotorNormal, PIDControl, encoder)
#pragma config(Motor,  motorB,          Left,          tmotorNormal, PIDControl, encoder)

float k; //expressed by r/d
short threshold;

#define offlineTimeout 300
#define cruiseTimeout 2000
#define cspeed 20
#define targetCalib 4
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

float compass(){
  return (nMotorEncoder[Right] - nMotorEncoder[Left])*k;
}

bool PIDDriver(short cruise_speed, short target,  float KP, float KD, float KI){
  float e, e_dot, old_e = threshold - SensorValue[Light], E = 0;
  short u;

  time1[T1] = 0;
  time1[T2] = 0;
  while(true){
    if(time1[T1] < offlineTimeout){
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
        if(time1[T2] > cruiseTimeout){
            return false;
        }
    }
    else{
        // it timer times out, just return
      return true;
    }
  }
}

// this helper function is used to control the motor speed and then wait for waittime miliseconds
void control(short left, short right, int waittime = 0){
  motor[Left] = left;
  motor[Right] = right;
  if(waittime){
    wait1Msec(waittime);
  }
}

void turnAndMove(short target){
  //turn 90 degrees and move back and forth
  float cur_compass = compass();
  control(-10, 10, 0);
  while(compass() - cur_compass < 88){}
  control(0, 0, 50);

  control(-20, -20, 20);
  while(SensorValue[Light] > target){}
  control(0, 0, 50);
  nMotorEncoder[Left]=0;
  nMotorEncoder[Right]=0;
  short startEncoderValueL = nMotorEncoder[Left];
  short startEncoderValueR = nMotorEncoder[Right];
  control(20, 20, 1000);
  while(SensorValue[Light] > target){}
  control(0, 0, 50);
  short endEncoderValueL = nMotorEncoder[Left];
  short endEncoderValueR = nMotorEncoder[Right];

  nMotorEncoderTarget[Left] = (startEncoderValueL + endEncoderValueL)/2;
  nMotorEncoderTarget[Right] = (startEncoderValueR + endEncoderValueR)/2;
  control(-20, -20, 200);
  while(nMotorRunState[Left] != runStateIdle && nMotorRunState[Right] != runStateIdle){}
  control(0, 0, 50);
}

task main(){
  if(!readCalib()) return; //reads calibration informations, return if it fails.
  short target = threshold + targetCalib;

  control(20, 20, 0);

  while(SensorValue[Light] > target){}
  wait1Msec(400);
  control(0, 0, 50);

  //rotate on the spot to find the dark line
  control(-20, 20, 0);
  while(SensorValue[Light] > target){}

  //stop when finds the line
  control(0, 0, 50);

  //use PID controller to follow the line
  while(PIDDriver(cspeed, target, Kp, Kd, Ki)){
    control(0, 0, 50);
    control(-20, 20, 0);

    while(SensorValue[Light] > target){}
    control(0, 0, 50);
  }

  turnAndMove(target);
  turnAndMove(target);

  //turn 45 degrees and move forward to the center
  float cur_compass = compass();
  control(10, -10, 0);
  while(cur_compass- compass() < 43){}
  control(0, 0, 50);
  control(20,20,400);
  control(0,0);


  return;
}
